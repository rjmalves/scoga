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
from typing import Dict, List
from copy import deepcopy
# Imports de módulos específicos da aplicação
from model.network.traffic_light import TLState
from model.traffic.traffic_plan import TrafficPlan


class Controller:
    """
    Responsável por conferir a passagem de tempo e avançar com o plano
    semafórico, obedecendo aos setpoints de controle.
    """

    def __init__(self, controller_id: str):
        self.id = controller_id
        self.current_time = 0.0
        self.is_started = False
        self.tl_ids: List[str] = []
        self.tl_states: Dict[str, TLState] = {}

        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria uma conexão com o broker bloqueante
        self.connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.channel = self.connection.channel()
        # Cria as exchanges e as filas específicas de cada serviço
        self.init_clock_exchange_and_queue()
        self.init_semaphore_exchange()

    def init_clock_exchange_and_queue(self):
        """
        Declara a exchange para pegar o tick do relógio e a relaciona com a
        fila exclusiva de relógio.
        """
        # Declara as exchanges
        self.channel.exchange_declare(exchange="clock_tick",
                                      exchange_type="fanout")
        # Cria as queues e realiza um bind no canal
        declare_result = self.channel.queue_declare(queue="", exclusive=True)
        self.clock_queue_name = declare_result.method.queue
        self.channel.queue_bind(exchange="clock_tick",
                                queue=self.clock_queue_name)

    def init_semaphore_exchange(self):
        """
        Declara a exchange onde serão postadas as mudanças de estado
        de semáforos sempre que acontecerem.
        """
        # Declara a exchange
        self.channel.exchange_declare(exchange="semaphores",
                                      exchange_type="topic")

    def start(self, filepath: str) -> bool:
        """
        Inicializa as threads do controlador. A partir deste momento ele:
        1) Se inscreve no exchange 'clock_tick' e passa a ouvir o relógio.
        2) Começa a publicar no exchange 'semaphores' sempre que houver uma
        mudança de estado.
        3) Se inscreve no exchange 'detectors' para poder ouvir quando
        houver detecção. TODO
        4) Se inscreve no exchange 'setpoints' para alterar os seus
        parâmetros de plano conforme ordenado pelo tempo real. TODO
        """
        # Carrega as configurações no arquivo especificado
        try:
            with open(filepath, "r") as filedata:
                data = json.load(filedata)
                self.tl_ids = data["traffic_light_ids"]
                self.traffic_plan = TrafficPlan.from_json(data["traffic_plan"])
                for tl_id, st in zip(self.tl_ids,
                                     self.traffic_plan.current_tl_states(0.0)):
                    self.tl_states[tl_id] = st
                # Publica forçadamente os estados atuais
                self.channel.basic_publish(exchange="semaphores",
                                           routing_key=str(self.id),
                                           body=str(self.tl_states))
        except Exception:
            traceback.print_exc()
            return False

        self.is_started = True
        self.clock_thread = threading.Thread(target=self.clock_listening,
                                             daemon=True)
        self.clock_thread.start()
        return True

    def clock_listening(self):
        """
        Função responsável pela thread que está inscrita para receber o tick
        do relógio da simulação.
        """
        # TODO - Substituir por um logging decente.
        print("Controlador {} começando a escutar o relógio!".format(self.id))
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.channel.basic_consume(queue=self.clock_queue_name,
                                       on_message_callback=self.clock_callback)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.clock_thread.join()

    def clock_callback(self,
                       ch: pika.adapters.blocking_connection.BlockingChannel,
                       method: pika.spec.Basic.Deliver,
                       property: pika.spec.BasicProperties,
                       body: bytes):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        controlador e verifica se houve alguma mudança de estado de semáforo
        para publicar.
        """
        # Guarda os estados atuais de semáforos
        tl_states_backup = deepcopy(self.tl_states)
        # Atualiza o instante de tempo atual
        self.current_time = float(body.decode())
        # TODO - Substituir por um logging decente.
        print("Controller {} - Clock Tick! Instante atual = {}"
              .format(self.id, self.current_time))
        # Verifica mudanças nos estados dos semáforos e publica.
        self.check_semaphore_changes(tl_states_backup)

    def check_semaphore_changes(self, tl_states_backup: Dict[str, TLState]):
        """
        Verifica se houve mudanças nos estados dos semáforos e publica.
        """
        # Compara os novos estados de semáforo com os antigos
        t = self.current_time
        for tl_id, st in zip(self.tl_ids,
                             self.traffic_plan.current_tl_states(t)):
            self.tl_states[tl_id] = st
        changed_sems: Dict[str, str] = {}
        for sem_id, sem_state in self.tl_states.items():
            if sem_state != tl_states_backup[sem_id]:
                changed_sems[sem_id] = str(sem_state)
        # Se algum mudou, publica a alteração
        if len(changed_sems.keys()) > 0:
            # TODO - Substituir por um logging decente.
            print("Mudou! ", str(changed_sems))
            self.channel.basic_publish(exchange="semaphore",
                                       routing_key=str(self.id),
                                       body=str(changed_sems))

    def __del__(self):
        """
        Como esta classe instancia threads, deve ter os .join() explícitos no
        destrutor.
        """
        # Não faz sentido dar join numa thread que não foi iniciada
        if self.is_started:
            self.clock_thread.join()


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
