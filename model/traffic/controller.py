# Modelo de controlador para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import ast
import pika  # type: ignore
import sys
import time
import json
import threading
import traceback
import logging
from typing import Dict, List
from copy import deepcopy
# Imports de módulos específicos da aplicação
from model.network.traffic_light import TLState
from model.traffic.traffic_plan import TrafficPlan
from model.traffic.setpoint import Setpoint


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
        self.is_started = False
        self.tl_ids: List[str] = []
        self.tl_states: Dict[str, TLState] = {}
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria as exchanges e as filas específicas de cada serviço
        self.init_clock_connection()
        self.init_semaphore_connection()
        self.init_set_connection()
        self.init_ack_connection()
        # Cria as threads que o controlador escuta
        self.clock_thread = threading.Thread(target=self.clock_listening,
                                             daemon=True)
        self.set_thread = threading.Thread(target=self.set_listening,
                                           daemon=True)

    def init_clock_connection(self):
        """
        Declara a exchange para pegar o tick do relógio e a relaciona com a
        fila exclusiva de relógio.
        """
        # Cria uma conexão com o broker bloqueante
        self.clock_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.clock_channel = self.clock_connection.channel()
        # Declara as exchanges
        self.clock_channel.exchange_declare(exchange="clock_tick",
                                            exchange_type="fanout")
        # Cria as queues e realiza um bind no canal
        declare_result = self.clock_channel.queue_declare(queue="",
                                                          exclusive=True)
        self.clock_queue_name = declare_result.method.queue
        self.clock_channel.queue_bind(exchange="clock_tick",
                                      queue=self.clock_queue_name)

    def init_set_connection(self):
        """
        Declara a exchange para atualizar setpoints de execução dos planos.
        """
        # Cria uma conexão com o broker bloqueante
        self.set_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.set_channel = self.set_connection.channel()
        # Declara as exchanges
        self.set_channel.exchange_declare(exchange="setpoints",
                                          exchange_type="topic")
        # Cria as queues e realiza um bind no canal
        declare_result = self.set_channel.queue_declare(queue="",
                                                        exclusive=True)
        self.set_queue_name = declare_result.method.queue
        self.set_channel.queue_bind(exchange="setpoints",
                                    queue=self.set_queue_name,
                                    routing_key=str(self.id))

    def init_semaphore_connection(self):
        """
        Declara a exchange onde serão postadas as mudanças de estado
        de semáforos sempre que acontecerem.
        """
        # Cria uma conexão com o broker bloqueante
        self.sem_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.sem_channel = self.sem_connection.channel()
        # Declara a exchange
        self.sem_channel.exchange_declare(exchange="semaphores",
                                          exchange_type="topic")

    def init_ack_connection(self):
        """
        Declara a exchange onde serão postadas os ACKs de recebimento
        dos passos de relógio.
        """
        # Cria uma conexão com o broker bloqueante
        self.ack_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.ack_channel = self.ack_connection.channel()
        # Declara a exchange
        self.ack_channel.exchange_declare(exchange="controllers",
                                          exchange_type="topic")

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
                                     self.traffic_plan.current_tl_states(0.0)):
                    self.tl_states[tl_id] = st
                sem_str: Dict[str, str] = {}
                for sem_id, sem_state in self.tl_states.items():
                    sem_str[sem_id] = str(sem_state)
                # Publica forçadamente os estados atuais
                self.sem_channel.basic_publish(exchange="semaphores",
                                               routing_key=str(self.id),
                                               body=str(sem_str))
        except Exception:
            traceback.print_exc()
            return False
        # Inicia as threads internas do controlador
        self.is_started = True
        self.clock_thread.start()
        self.set_thread.start()
        return True

    def clock_listening(self):
        """
        Função responsável pela thread que está inscrita para receber o tick
        do relógio da simulação.
        """
        # TODO - Substituir por um logging decente.
        self.logger.info("Controlador {} começando a escutar o relógio!"
                         .format(self.id))
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.clock_channel.basic_consume(queue=self.clock_queue_name,
                                             on_message_callback=self.clock_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.clock_channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.clock_thread.join()

    def clock_cb(self,
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
        # Verifica mudanças nos estados dos semáforos e publica.
        self.check_semaphore_changes(tl_states_backup)
        # Publica o ACK de ter recebido o passo
        self.ack_channel.basic_publish(exchange="controllers",
                                       routing_key=str(self.id),
                                       body=str(self.id))

    def set_listening(self):
        """
        Função responsável pela thread que está inscrita para receber mudanças
        nos setpoints dos planos.
        """
        # TODO - Substituir por um logging decente.
        self.logger.info("Controlador {} começando a escutar setpoints!"
                         .format(self.id))
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.set_channel.basic_consume(queue=self.set_queue_name,
                                           on_message_callback=self.set_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.set_channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.set_thread.join()

    def set_cb(self,
               ch: pika.adapters.blocking_connection.BlockingChannel,
               method: pika.spec.Basic.Deliver,
               property: pika.spec.BasicProperties,
               body: bytes):
        """
        Função responsável por atualizar o setpoint de execução dos planos
        semafóricos.
        """
        # Processa o conteúdo do corpo da mensagem
        body_str: dict = ast.literal_eval(body.decode())
        # Constroi o objeto setpoint a ser aplicado
        setpoint = Setpoint.from_json(body_str)
        # Aplica o setpoint no plano atual
        self.traffic_plan.update(setpoint)
        self.logger.info("Controlador {}: S: {}, C: {}s, O: {}s"
                         .format(self.id,
                                 setpoint.splits,
                                 setpoint.cycle,
                                 setpoint.offset))

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
            self.sem_channel.basic_publish(exchange="semaphores",
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
            self.set_thread.join()


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
