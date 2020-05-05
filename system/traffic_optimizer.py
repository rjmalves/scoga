# Otimizador de tráfego para uso integrado com simulação do SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 04 de Maio de 2020

# Imports gerais de módulos padrão
import ast
import pika  # type: ignore
import time
import json
import threading
import traceback
from copy import deepcopy
from typing import Dict, List
# Imports de módulos específicos da aplicação
from model.traffic.controller import Controller
from model.traffic.traffic_plan import TrafficPlan
from model.traffic.setpoint import Setpoint
from model.network.detector import Detector
from model.network.traffic_light import TrafficLight, TLState


class TrafficOptimizer:
    """
    Classe responsável por receber dados históricos de controladores e
    detectores para realizar a otimização para controle semafórico.
    Para isso, deve:
    - No momento de sua criação, obter as configurações da rede, dos
    controladores e detectores envolvidos.
    - Se inscrever nos tópicos detectors e trafficlights para acompanhar
    o estado do sistema continuamente.
    - Sempre que receber uma atualização de estado de algum semáforo,
    atualizar os históricos de ciclos internos.
    - Quando um novo histórico de ciclo for finalizado, chamar o otimizador.
    - Quando obtiver resposta da rotina de otimização, enviar os novos
    setpoints para os controllers publicando no tópico setpoints.
    """
    def __init__(self,
                 traffic_lights: Dict[str, TrafficLight],
                 detectors: Dict[str, Detector]):
        # Recebe os objetos da simulação.
        self.current_time = 0.0
        self.traffic_lights = deepcopy(traffic_lights)
        self.detectors = deepcopy(detectors)
        self.setpoints: Dict[str, Setpoint] = {}
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria as exchanges e as filas de cada serviço
        self.init_clock_connection()
        self.init_det_connection()
        self.init_semaphore_connection()
        self.init_setpoint_connection()
        # Cria as threads que escutam clock, semaphores e detectors
        self.clk_thread = threading.Thread(target=self.clock_listening,
                                           daemon=True)
        self.det_thread = threading.Thread(target=self.det_listening,
                                           daemon=True)
        self.sem_thread = threading.Thread(target=self.semaphores_listening,
                                           daemon=True)
        # Cria a thread que envia setpoints
        self.set_thread = threading.Thread(target=self.setpoints_sending,
                                           daemon=True)

        # TODO - DEBUG
        self.last_sent_time = self.current_time
        # TODO - gerar o grafo da rede com os IDs adequados e receber aqui

    def start(self, started_controllers: Dict[str, Controller]):
        """
        """
        try:
            # Gera os objetos Setpoint iniciais
            for ctrl_id, ctrl in started_controllers.items():
                plan = ctrl.traffic_plan
                self.setpoints[ctrl_id] = self.__setpoints_from_plan(plan)
            # Inicia a thread que escuta o relógio
            self.clk_thread.start()
            # Inicia a thread que escuta detectores
            self.det_thread.start()
            # Inicia a thread que escuta semáforos
            self.sem_thread.start()
            # Inicia a thread que envia setpoints
            self.set_thread.start()
        except Exception:
            traceback.print_exc()

    def __setpoints_from_plan(self, traffic_plan: TrafficPlan) -> Setpoint:
        """
        """
        stage_times = [stage.length for stage in traffic_plan.stages]
        offset = traffic_plan.offset
        return Setpoint(stage_times, offset)

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

    def init_det_connection(self):
        """
        Declara a exchange para atualizar estados de detectores e a relaciona
        com a fila exclusiva de detectores.
        """
        # Cria uma conexão com o broker bloqueante
        self.det_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.det_channel = self.det_connection.channel()
        # Declara a exchange
        self.det_channel.exchange_declare(exchange="detectors",
                                          exchange_type="topic")
        # Cria as queues e realiza um bind no canal
        declare_result = self.det_channel.queue_declare(queue="",
                                                        exclusive=True)
        self.det_queue_name = declare_result.method.queue
        self.det_channel.queue_bind(exchange="detectors",
                                    queue=self.det_queue_name)

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
        # Cria as queues e realiza um bind no canal
        declare_result = self.sem_channel.queue_declare(queue="",
                                                        exclusive=True)
        self.sem_queue_name = declare_result.method.queue
        self.sem_channel.queue_bind(exchange="semaphores",
                                    queue=self.sem_queue_name)

    def init_setpoint_connection(self):
        """
        Declara a exchange onde serão postadas as mudanças de setpoints
        de execução dos planos semafóricos para os controladores.
        """
        # Cria uma conexão com o broker bloqueante
        self.set_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.set_channel = self.set_connection.channel()
        # Declara a exchange
        self.set_channel.exchange_declare(exchange="setpoints",
                                          exchange_type="topic")

    def clock_listening(self):
        """
        Função responsável pela thread que está inscrita para receber o tick
        do relógio da simulação.
        """
        # TODO - Substituir por um logging decente.
        print("Otimizador começando a escutar o relógio!")
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

    def det_listening(self):
        """
        Função responsável pela thread que está inscrita para receber mudanças
        nos estados dos detectores.
        """
        # TODO - Substituir por um logging decente.
        print("Otimizador começando a escutar detectores!")
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.det_channel.basic_consume(queue=self.det_queue_name,
                                           on_message_callback=self.det_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.det_channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.det_thread.join()

    def semaphores_listening(self):
        """
        Função da thread que escuta mudanças nos semáforos para construir o
        histórico de ciclos.
        """
        # TODO - Substituir por um logging decente.
        print("Otimizador começou a escutar semaphores!")
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.sem_channel.basic_consume(queue=self.sem_queue_name,
                                           on_message_callback=self.sem_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.sem_channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.sem_thread.join()

    def setpoints_sending(self):
        """
        """
        try:
            # Faz o envio permanentemente
            while True:
                # Para testes, a cada 30 segundos
                if self.current_time - self.last_sent_time > 30.0:
                    self.last_sent_time = self.current_time
                    for c_id, setpoint in self.setpoints.items():
                        # Prepara o corpo da mensagem
                        body = json.dumps(setpoint.to_json())
                        # Publica, usando o ID do controlador como chave
                        self.set_channel.basic_publish(exchange="setpoints",
                                                       routing_key=str(c_id),
                                                       body=body)
                time.sleep(0.1)
        except Exception:
            traceback.print_exc()

    def clock_cb(self,
                 ch: pika.adapters.blocking_connection.BlockingChannel,
                 method: pika.spec.Basic.Deliver,
                 property: pika.spec.BasicProperties,
                 body: bytes):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        otimizador.
        """
        # Atualiza o instante de tempo atual
        self.current_time = float(body.decode())

    def sem_cb(self,
               ch: pika.adapters.blocking_connection.BlockingChannel,
               method: pika.spec.Basic.Deliver,
               property: pika.spec.BasicProperties,
               body: bytes):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza o histórico do objeto semáforo local.
        """
        # Processa o corpo da publicação recebida
        body_dict: dict = ast.literal_eval(body.decode())
        # Agrupa os semáforos que mudaram de estado num novo dict, agora por
        # objeto traffic_light do SUMO, depois por TrafficLight local
        sumo_tls = set([sumo_tl_id.split("-")[0]
                        for sumo_tl_id in list(body_dict.keys())])
        semaphores: Dict[str, Dict[str, int]] = {}
        for sumo_tl_id in sumo_tls:
            semaphores[sumo_tl_id] = {}
        for tl_id, state in body_dict.items():
            sumo_tl_id = tl_id.split("-")[0]
            semaphores[sumo_tl_id][tl_id] = int(state)
        # Atualiza os objetos semáforo locais
        t = self.current_time
        for sumo_tl_id, tl in semaphores.items():
            for tl_id, state in tl.items():
                self.traffic_lights[tl_id].update_state(TLState(state), t)

    def det_cb(self,
               ch: pika.adapters.blocking_connection.BlockingChannel,
               method: pika.spec.Basic.Deliver,
               property: pika.spec.BasicProperties,
               body: bytes):
        """
        Função responsável por atualizar o objeto detecção de cada detector com
        a última detecção ocorrida.
        """
        # Processa o conteúdo do corpo da mensagem
        body_list: List[tuple] = ast.literal_eval(body.decode())
        # O corpo é uma lista com tuplas da forma ("det_id", state)
        t = self.current_time
        for change in body_list:
            det_id = change[0]
            state = bool(change[1])
            self.detectors[det_id].update_detection_history(t, state)

    def __del__(self):
        """
        Interrompe as threads em execução quando o objeto é destruído.
        """
        # Interrompe as threads
        self.clk_thread.join()
        self.det_thread.join()
        self.sem_thread.join()
