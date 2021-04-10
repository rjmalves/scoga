# Modelo de controlador para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import pika  # type: ignore
import sys
import time
import json
import threading
import traceback
import logging
from typing import Dict, List
from copy import deepcopy
from PikaBus.PikaBusSetup import PikaBusSetup
from sumolib.net import TLS
# Imports de módulos específicos da aplicação
from model.network.traffic_light import TLState
from model.traffic.traffic_plan import TrafficPlan
from model.traffic.setpoint import Setpoint
from model.messages.semaphores import SemaphoresMessage
from model.messages.controllerack import ControllerAckMessage
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
        self.logger = logging.getLogger(__name__)
        self.id = node_id
        self.current_time = 0.0
        self.current_cycle = 0
        self.cycle_change_time = -120
        self.is_started = False
        self.tl_ids: List[str] = []
        self.tl_states: Dict[str, TLState] = {}
        self.first_interval_tl_states: Dict[str, TLState] = {}
        self.last_interval_tl_states: Dict[str, TLState] = {}
        self.tl_change_time: Dict[str, float] = {}
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        self.init_semaphore_connection()
        self.init_set_connection()

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
                for tl_id, st in zip(self.tl_ids,
                                     self.traffic_plan.current_tl_states(0.0,
                                                                         True)):
                    self.tl_states[tl_id] = st
                    self.tl_change_time[tl_id] = 0
                # Guarda os estados dos grupos semafóricos no primeiro e
                # últimos intervalos
                first_interval = self.traffic_plan.stages[0].intervals[0]
                for i, tl_state in enumerate(first_interval.states):
                    tl_id = self.tl_ids[i]
                    self.first_interval_tl_states[tl_id] = tl_state
                last_interval = self.traffic_plan.stages[-1].intervals[-1]
                for i, tl_state in enumerate(last_interval.states):
                    tl_id = self.tl_ids[i]
                    self.last_interval_tl_states[tl_id] = tl_state
                # Prepara para publicar os estados atuais
                sem_str: Dict[str, str] = {}
                for sem_id, sem_state in self.tl_states.items():
                    sem_str[sem_id] = str(sem_state)
                # Envia a mensagem com os estados atuais
                stg = self.traffic_plan.current_plan_stage(self.current_time)
                interval = self.traffic_plan.stages[stg].current_interval_idx
                message = SemaphoresMessage(sem_str,
                                            self.current_cycle,
                                            stg,
                                            interval,
                                            self.current_time)
                self.sem_bus.Publish(payload=message.to_dict(),
                                     topic="semaphores")
        except Exception:
            traceback.print_exc()
            return False
        # Inicia as threads internas do controlador
        self.is_started = True
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
        # Guarda os estados atuais de semáforos
        tl_states_backup = deepcopy(self.tl_states)
        # Atualiza o instante de tempo atual
        self.current_time = int(round((float(kwargs['payload']))))
        # Verifica mudanças nos estados dos semáforos e publica.
        self.check_semaphore_changes(tl_states_backup)
        # Publica o ACK de ter recebido o passo
        stg = self.traffic_plan.current_plan_stage(self.current_time)
        interval = self.traffic_plan.stages[stg].current_interval_idx
        message = ControllerAckMessage(self.id,
                                       self.current_cycle,
                                       stg,
                                       interval,
                                       self.current_time)
        self.ack_bus.Publish(payload=message.to_dict(),
                             topic="controllers")

    def set_cb(self, **kwargs):
        """
        Função responsável por atualizar o setpoint de execução dos planos
        semafóricos.
        """
        # Processa o conteúdo do corpo da mensagem
        body_str = kwargs["payload"]
        # Constroi o objeto setpoint a ser aplicado
        setpoint = Setpoint.from_json(body_str)
        str_log = (f"Ctrl {self.id} " +
                   f"Stage Lengths = {setpoint.generate_stage_times()} " +
                   f"Offset = {setpoint.offset}")
        console.log(str_log)
        # Aplica o setpoint no plano atual
        self.traffic_plan.update(setpoint)

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
            console.log(f"Ctrl {self.id} no ciclo {self.current_cycle} - t = {t}")
        # Procura alterações nos semáforos
        changed_sems: Dict[str, str] = {}
        for sem_id, sem_state in self.tl_states.items():
            if sem_state != tl_states_backup[sem_id]:
                changed_sems[sem_id] = str(sem_state)
                self.tl_change_time[sem_id] = self.current_time
        # Se algum mudou, publica a alteração
        if len(changed_sems.keys()) > 0:
            stg = self.traffic_plan.current_plan_stage(self.current_time)
            interval = self.traffic_plan.stages[stg].current_interval_idx
            message = SemaphoresMessage(changed_sems,
                                        self.current_cycle,
                                        stg,
                                        interval,
                                        self.current_time)
            console.log(f"Ctrl {self.id}: {message}")
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

    def stop_communication(self):
        """
        """
        if self.is_started:
            console.log(f"Terminando a comunicação no ctrl {self.id}")
            self._ack_pika_bus.StopConsumers()
            self._sem_pika_bus.StopConsumers()
            self._clk_pika_bus.StopConsumers()
            self._set_pika_bus.StopConsumers()
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
            self._ack_pika_bus.StopConsumers()
            self._sem_pika_bus.StopConsumers()
            self._clk_pika_bus.StopConsumers()
            self._set_pika_bus.StopConsumers()
            self._ack_pika_bus.Stop()
            self._ack_pika_bus.Stop()
            self._clk_pika_bus.Stop()
            self._set_pika_bus.Stop()


# Caso de teste do controlador.
# TODO - Substituir por uma rotina de testes decente usando pytest.
if __name__ == "__main__":
    try:
        # Confere se recebeu pelo menos dois argumentos na linha de comando
        if len(sys.argv) < 2:
            print("Por favor, informe um ID.")
            exit(0)
        # Assume que o parâmetro passado por linha de comando é o id
        controller_id = str(sys.argv[1])
        controller = Controller(controller_id)
        # Começa a escutar o relógio
        controller.start("config/controllers/1.json")
        # Aguarda todas as threads serem finalizadas
        while threading.active_count() > 0:
            time.sleep(1.0)
    except KeyboardInterrupt:
        # TODO - Substituir por um logging decente.
        print("Finalizando o teste do controlador {}!".format(controller_id))
        exit(0)
