# Controlador de tráfego para uso integrado com simulação do SUMO.
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
import logging
from copy import deepcopy
from pandas import DataFrame  # type: ignore
from typing import Dict, List
# Imports de módulos específicos da aplicação
from model.traffic.controller import Controller
from model.traffic.traffic_plan import TrafficPlan
from model.traffic.setpoint import Setpoint
from model.network.detector import Detector
from model.network.traffic_light import TLState
from model.network.traffic_light import TrafficLight
from model.network.network import Network
from model.optimization.node_history import NodeHistory
from system.optimization.scoot import ScootOptimizer


class TrafficController:
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
    def __init__(self, network: Network, tls: Dict[str, TrafficLight]):
        self.logger = logging.getLogger(__name__)
        # Prepara para receber os dispositivos na simulação
        self.current_time = 0.0
        self.detectors: Dict[str, Detector] = {}
        self.controllers: Dict[str, Controller] = {}
        # Prepara para gerar os setpoints de controle
        self.setpoints: Dict[str, Setpoint] = {}
        # Guarda o objeto que guarda informações da topologia
        self.network = network
        # Cria uma instância do otimizador
        self.optimizer = ScootOptimizer(self.network,
                                        self.setpoints,
                                        self.controllers,
                                        tls)
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

    def start(self,
              controllers: Dict[str, Controller],
              detectors: Dict[str, Detector]):
        """
        """
        try:
            # TODO - Faz uma cópia local dos controllers
            # (por enquanto é referência). Problema com threads.
            for node_id, ctrl in controllers.items():
                self.controllers[node_id] = ctrl
            # Faz uma cópia local dos detectors
            for det_id, det in detectors.items():
                self.detectors[det_id] = deepcopy(det)
            # Gera os objetos Setpoint e NodeHistory iniciais
            t = self.current_time
            for node_id, ctrl in self.controllers.items():
                plan = ctrl.traffic_plan
                self.setpoints[node_id] = self.__setpoints_from_plan(plan)
                n_hist = NodeHistory(node_id, ctrl.tl_ids, plan, t)
                self.network.nodes[node_id].add_history(n_hist)
            # Gera os objetos LaneHistory para cada Lane observada
            for edge_id, edge in self.network.edges.items():
                for lane_id, lane in edge.lanes.items():
                    # Procura para ver se algum detector está na Lane
                    detectors_in_lane: Dict[str, Detector] = {}
                    for det_id, det in self.detectors.items():
                        if det.lane_id == lane_id:
                            # TODO - fazer deixar de passar por referência
                            detectors_in_lane[det_id] = det
                    # Se a lista não está vazia, adiciona à Lane
                    if len(detectors_in_lane) > 0:
                        lane.history.add_detectors(detectors_in_lane)
            # Inicia a thread que escuta o relógio
            self.clk_thread.start()
            # Inicia a thread que escuta detectores
            self.det_thread.start()
            # Inicia a thread que escuta semáforos
            self.sem_thread.start()
            # Inicia a thread que envia setpoints
            self.set_thread.start()
            # Inicia o otimizador
            self.optimizer.start(self.setpoints)
        except Exception:
            traceback.print_exc()

    def __setpoints_from_plan(self, traffic_plan: TrafficPlan) -> Setpoint:
        """
        """
        stage_times = [int(stage.length) for stage in traffic_plan.stages]
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
                                    routing_key="*",
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
                                    routing_key="*",
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
        self.logger.info("Central inscrita em clock_tick!")
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
        self.logger.info("Central inscrita em detectors!")
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
        self.logger.info("Central inscrita em semaphores!")
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
                # Verifica com o otimizador se existem novos resultados
                # de otimização
                new_setpoints = self.optimizer.new_setpoints()
                for setpoint_dict in new_setpoints:
                    for c_id, setpoint in setpoint_dict.items():
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
        # Atualiza o objeto que armazena o histórico de cada interseção
        for sumo_tl_id, tl in semaphores.items():
            for tl_id, state in tl.items():
                # Encontra de qual nó é o semáforo que foi atualizado
                for n_id, ctrl in self.controllers.items():
                    if tl_id in ctrl.tl_ids:
                        # Encontrou o nó
                        t = self.current_time
                        self.network.nodes[n_id].history.update(tl_id,
                                                                TLState(state),
                                                                t)
                        break

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
        # TODO - por enquanto atualiza os detectores no escopo local.
        # Como eles são passados por referência para o objeto network,
        # tudo bem. Mas isso tem que deixar de ser feito!!!!!!
        for change in body_list:
            det_id = change[0]
            state = bool(change[1])
            self.detectors[det_id].update_detection_history(t, state)

    def export_node_histories(self) -> DataFrame:
        """
        """
        node_hists = DataFrame()
        for node_id, node in self.network.nodes.items():
            # Somente os nodes controlados possuem histórico
            if node.controlled:
                data = node.history.export(self.current_time)
                node_hists = DataFrame.append(node_hists,
                                              data,
                                              ignore_index=True,
                                              sort=False)
        return node_hists

    def export_edge_histories(self) -> DataFrame:
        """
        """
        edge_hists = DataFrame()
        for edge_id, edge in self.network.edges.items():
            # Todas as edges possuem histórico!
            data = edge.history.export_traffic_data()
            edge_hists = DataFrame.append(edge_hists,
                                          data,
                                          ignore_index=True,
                                          sort=False)
        return edge_hists

    def export_lane_histories(self) -> DataFrame:
        """
        """
        lane_hists = DataFrame()
        for edge_id, edge in self.network.edges.items():
            for lane_id, lane in edge.lanes.items():
                # Todas as lanes possuem histórico!
                data = lane.history.export_traffic_data()
                lane_hists = DataFrame.append(lane_hists,
                                              data,
                                              ignore_index=True,
                                              sort=False)
        return lane_hists

    @property
    def busy_optimizer(self) -> bool:
        return self.optimizer.busy

    def __del__(self):
        """
        Interrompe as threads em execução quando o objeto é destruído.
        """
        # Interrompe as threads
        self.clk_thread.join()
        self.det_thread.join()
        self.sem_thread.join()
