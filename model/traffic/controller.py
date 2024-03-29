# Modelo de controlador para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import pika  # type: ignore
import time
import json
import threading
import traceback
from typing import Dict, List
from copy import deepcopy
from PikaBus.PikaBusSetup import PikaBusSetup
# Imports de módulos específicos da aplicação
from model.network.traffic_light import TLState
from model.traffic.traffic_plan import TrafficPlan
from model.messages.semaphores import SemaphoresMessage
from model.messages.controllerack import ControllerAckMessage
from model.messages.setpoints import SetpointsMessage
from rich.console import Console
console = Console()

MIN_GREEN = 10
MIN_AMBER = 3
MIN_RED = 5
MIN_CYCLE_FIRST_INTERVAL = 10


class Controller:
    """
    Responsável por conferir a passagem de tempo e avançar com o plano
    semafórico, obedecendo aos setpoints de controle.
    """

    def __init__(self, node_id: str):
        self.should_exit = False
        self.id = node_id
        self.current_time = 0.0
        self.current_cycle = 0
        self.cycle_change_time = -120
        self.is_started = False
        self.time_since_last_ack = 0
        self.tl_ids: List[str] = []
        self.tl_states: Dict[str, TLState] = {}
        self.first_interval_tl_states: Dict[str, TLState] = {}
        self.last_interval_tl_states: Dict[str, TLState] = {}
        self.tl_change_time: Dict[str, float] = {}
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        self.init_semaphore_connection()
        self.init_set_connection()
        # Define a thread que envia acks por segurança, no caso de
        # mensagens perdidas
        self.ack_backup_thread = threading.Thread(target=self.ack_backup,
                                                  name=f"{self.id} AckBackup")

    def ack_backup(self):
        """
        Envia acks periodicamente para evitar travamento na simulação.
        """
        self.time_since_last_ack = time.time()
        while not self.should_exit:
            if time.time() - self.time_since_last_ack > 5.0:
                console.log(f"Ctrl {self.id}: Backup ACK")
                message = self._make_ack_message()
                self.ack_bus.Publish(payload=message.to_dict(),
                                     topic="controllers")
            time.sleep(1e-6)

    def _make_ack_message(self) -> ControllerAckMessage:
        """
        """
        self.time_since_last_ack = time.time()
        stg = self.traffic_plan.current_plan_stage(self.current_time)
        interval = self.traffic_plan.stages[stg].current_interval_idx
        return ControllerAckMessage(self.id,
                                    self.current_cycle,
                                    stg,
                                    interval,
                                    self.current_time)

    def _make_sem_message(self) -> SemaphoresMessage:
        """
        """
        # Prepara para publicar os estados atuais
        sem_str: Dict[str, str] = {}
        for sem_id, sem_state in self.tl_states.items():
            sem_str[sem_id] = str(sem_state)
        # Envia a mensagem com os estados atuais
        stg = self.traffic_plan.current_plan_stage(self.current_time)
        interval = self.traffic_plan.stages[stg].current_interval_idx
        return SemaphoresMessage(self.id,
                                 sem_str,
                                 self.current_cycle,
                                 stg,
                                 interval,
                                 self.current_time)

    def init_semaphore_connection(self):
        """
        Declara a exchange para pegar o tick do relógio e a relaciona com a
        fila exclusiva de relógio.
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        q_name = f'ctrl_{self.id}_sem_queue'
        self._sem_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue=q_name)
        self._sem_pika_bus.StartConsumers()
        self.sem_bus = self._sem_pika_bus.CreateBus()

    def init_clock_connection(self):
        """
        Declara a exchange para pegar o tick do relógio e a relaciona com a
        fila exclusiva de relógio.
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        q_name = f'ctrl_{self.id}_clk_queue'
        self._clk_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue=q_name,
                                          defaultSubscriptions='clock_tick')
        self._clk_pika_bus.AddMessageHandler(self.clock_cb)
        self._clk_pika_bus.StartConsumers()
        self.clk_bus = self._clk_pika_bus.CreateBus()

    def init_ack_connection(self):
        """
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        q_name = f'ctrl_{self.id}_ack_queue'
        self._ack_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue=q_name,
                                          defaultSubscriptions='controllers')
        self._ack_pika_bus.StartConsumers()
        self.ack_bus = self._ack_pika_bus.CreateBus()

    def init_set_connection(self):
        """
        Declara a exchange para atualizar setpoints de execução dos planos.
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        q_name = f'ctrl_{self.id}_set_queue'
        self._set_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue=q_name)
        self._set_pika_bus.AddMessageHandler(self.set_cb)
        self._set_pika_bus.StartConsumers()
        self.set_bus = self._set_pika_bus.CreateBus()

    def start(self, filepath: str) -> bool:
        """
        Inicializa as threads do controlador. A partir deste momento ele:
        1) Se inscreve no exchange 'clock_tick' e passa a ouvir o relógio.
        2) Começa a publicar no exchange 'semaphores' sempre que houver uma
        mudança de estado.
        3) Se inscreve no exchange 'setpoints' para alterar os seus
        parâmetros de plano conforme ordenado pelo tempo real.
        """
        # Carrega as configurações no arquivo especificado
        try:
            with open(filepath, "r") as filedata:
                data = json.load(filedata)
                # Carrega o plano e os grupos semafóricos
                self.tl_ids = data["traffic_light_ids"]
                self.traffic_plan = TrafficPlan.from_json(data["traffic_plan"])
                for tid, st in zip(self.tl_ids,
                                   self.traffic_plan.current_tl_states(0.0,
                                                                       True)):
                    self.tl_states[tid] = st
                    self.tl_change_time[tid] = 0
                # Guarda os estados dos grupos semafóricos no primeiro e
                # últimos intervalos
                first_interval = self.traffic_plan.stages[0].intervals[0]
                for i, tl_state in enumerate(first_interval.states):
                    tid = self.tl_ids[i]
                    self.first_interval_tl_states[tid] = tl_state
                last_interval = self.traffic_plan.stages[-1].intervals[-1]
                for i, tl_state in enumerate(last_interval.states):
                    tid = self.tl_ids[i]
                    self.last_interval_tl_states[tid] = tl_state
                message = self._make_sem_message()
                self.sem_bus.Publish(payload=message.to_dict(),
                                     topic="semaphores")
        except Exception:
            traceback.print_exc()
            return False
        # Inicia as threads internas do controlador
        self.is_started = True
        self.ack_backup_thread.start()
        self.init_ack_connection()
        self.init_clock_connection()
        self.init_set_connection()
        return True

    def clock_cb(self, **kwargs):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        controlador e verifica se houve alguma mudança de estado de semáforo
        para publicar.
        """
        self.time_since_last_ack = time.time()
        # Guarda os estados atuais de semáforos
        tl_states_backup = deepcopy(self.tl_states)
        # Atualiza o instante de tempo atual
        self.current_time = int(round((float(kwargs['payload']))))
        # Verifica mudanças nos estados dos semáforos e publica.
        self.check_semaphore_changes(tl_states_backup)
        # Publica o ACK de ter recebido o passo
        message = self._make_ack_message()
        self.ack_bus.Publish(payload=message.to_dict(),
                             topic="controllers")

    def set_cb(self, **kwargs):
        """
        Função responsável por atualizar o setpoint de execução dos planos
        semafóricos.
        """
        message = SetpointsMessage.from_dict(kwargs["payload"])
        console.log(f"Ctrl {self.id}: {message}")
        # Aplica o setpoint no plano atual
        self.traffic_plan.update(message.setpoint)

    def check_semaphore_changes(self, tl_states_backup: Dict[str, TLState]):
        """
        Verifica se houve mudanças nos estados dos semáforos e publica.
        """
        # Verifica se passou o mínimo de tempo após a entrada do ciclo
        # para avaliar uma mudança
        t = self.current_time
        if t - self.cycle_change_time < MIN_CYCLE_FIRST_INTERVAL:
            return
        # Senão, faz a verificação normalmente
        # Compara os novos estados de semáforo com os antigos
        for tl_id, st in zip(self.tl_ids,
                             self.traffic_plan.current_tl_states(t)):
            self.tl_states[tl_id] = st
        # Confere se mudou de ciclo ou não
        if self.check_changed_cycle(tl_states_backup):
            self.current_cycle += 1
            self.cycle_change_time = t
        # Procura alterações nos semáforos
        changed_sems: Dict[str, str] = {}
        for sem_id, sem_state in self.tl_states.items():
            if sem_state != tl_states_backup[sem_id]:
                changed_sems[sem_id] = str(sem_state)
                self.tl_change_time[sem_id] = self.current_time
        # Se algum mudou, publica a alteração
        if len(changed_sems.keys()) > 0:
            message = self._make_sem_message()
            self.sem_bus.Publish(payload=message.to_dict(),
                                 topic="semaphores")

    def check_safety_times(self, tl_id: str) -> bool:
        """
        """
        state = self.tl_states[tl_id]
        time_since_change = int(self.current_time -
                                self.tl_change_time[tl_id])
        if state == TLState.GREEN:
            return time_since_change >= MIN_GREEN
        elif state == TLState.AMBER:
            return time_since_change >= MIN_AMBER
        elif state == TLState.RED:
            return time_since_change >= MIN_RED
        return False

    def check_changed_cycle(self,
                            tl_states_backup: Dict[str, TLState]
                            ) -> bool:
        """
        """
        # Muda de ciclo se os estados atuais são iguais aos do primeiro
        # intervalo e os do backup são do último
        is_first = all([(self.tl_states[tl_id] ==
                         self.first_interval_tl_states[tl_id])
                        for tl_id in self.tl_ids])
        was_last = all([(tl_states_backup[tl_id] ==
                         self.last_interval_tl_states[tl_id])
                        for tl_id in self.tl_ids])

        return is_first and was_last

    def end(self):
        """
        """
        if self.is_started:
            self.should_exit = True
            self.ack_backup_thread.join()
            self._ack_pika_bus.Stop()
            self._ack_pika_bus.Stop()
            self._clk_pika_bus.Stop()
            self._set_pika_bus.Stop()

    def __del__(self):
        """
        Como esta classe instancia threads, deve ter os .join() explícitos no
        destrutor.
        """
        # Não faz sentido dar join numa thread que não foi iniciada
        if self.is_started:
            self.ack_backup_thread.join()
            self._ack_pika_bus.Stop()
            self._ack_pika_bus.Stop()
            self._clk_pika_bus.Stop()
            self._set_pika_bus.Stop()
