# Controlador de tráfego para uso integrado com simulação do SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 04 de Maio de 2020

# Imports gerais de módulos padrão
from multiprocessing.queues import Queue
from model.messages.cycle import CycleMessage
from model.messages.clocktick import ClockTickMessage
from model.messages.setpoints import SetpointsMessage
from model.messages.semaphores import SemaphoresMessage
import threading
import time
from copy import deepcopy
from pandas import DataFrame  # type: ignore
from typing import Dict
# Imports de módulos específicos da aplicação
from model.traffic.traffic_plan import TrafficPlan
from model.traffic.setpoint import Setpoint
from model.network.detector import Detector
from model.network.traffic_light import TrafficLight
from model.network.network import Network
from model.optimization.node_history import NodeHistory
from system.optimization.scoot import EnumOptimizationMethods
from system.optimization.scoot import ScootOptimizer
from system.optimization.itlc import ITLCOptimizer
from rich.console import Console
console = Console()


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
    def __init__(self,
                 network: Network,
                 tls: Dict[str, TrafficLight],
                 opt_method: EnumOptimizationMethods):
        self.should_exit = False
        # Prepara para receber os dispositivos na simulação
        self.current_time = 0.0
        self.detectors: Dict[str, Detector] = {}
        self.plans: Dict[str, TrafficPlan] = {}
        # Prepara para gerar os setpoints de controle
        self.setpoints: Dict[str, Setpoint] = {}
        # Guarda o objeto que guarda informações da topologia
        self.network = network
        # Cria uma instância do otimizador
        self.optimizer = ScootOptimizer(self.network,
                                        self.plans,
                                        self.setpoints,
                                        tls,
                                        opt_method)

        # Cria a thread de envio dos setpoints
        self.set_thread = threading.Thread(target=self.setpoints_sending,
                                           daemon=True)

    def start(self,
              plans: Dict[str, TrafficPlan],
              detectors: Dict[str, Detector],
              queues: Dict[str, Queue]):
        """
        """
        try:
            # TODO - Faz uma cópia local dos controllers
            # (por enquanto é referência). Problema com threads.
            for node_id, plan in plans.items():
                self.plans[node_id] = plan
            # Faz uma cópia local dos detectors
            for det_id, det in detectors.items():
                self.detectors[det_id] = deepcopy(det)
            # Gera os objetos Setpoint e NodeHistory iniciais
            t = self.current_time
            for node_id, plan in self.plans.items():
                self.setpoints[node_id] = self.__setpoints_from_plan(plan)
                n_hist = NodeHistory(node_id, t)
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
            self.controller_queues = queues
            # Inicia a thread que envia setpoints
            self.set_thread.start()
            # Inicia o otimizador
            self.optimizer.start(self.setpoints)
        except Exception:
            console.print_exception()

    def __setpoints_from_plan(self, traffic_plan: TrafficPlan) -> Setpoint:
        """
        """
        stage_times = [int(stage.length) for stage in traffic_plan.stages]
        offset = traffic_plan.offset
        return Setpoint(stage_times, offset)

    def setpoints_sending(self):
        """
        """
        try:
            # Faz o envio permanentemente
            while not self.should_exit:
                # Verifica com o otimizador se existem novos resultados
                # de otimização
                new_setpoints = self.optimizer.new_setpoints()
                for setpoint_dict in new_setpoints:
                    for ctrl_id, setpoint in setpoint_dict.items():
                        message = SetpointsMessage(ctrl_id,
                                                   setpoint)
                        self.controller_queues[ctrl_id].put(message)
                time.sleep(1e-6)
        except Exception:
            console.print_exception()

    def cycle_cb(self, mess: CycleMessage):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        otimizador.
        """
        self.optimizer.cycle_cb(mess)

    def clock_cb(self, tick: ClockTickMessage):
        """
        Função executada logo após uma atualização de relógio por meio do
        gerador de relógio. Atualiza o instante de tempo atual para o
        otimizador.
        """
        # Atualiza o instante de tempo atual
        try:
            self.current_time = tick.time
            self.optimizer.simulation_time = self.current_time
        except Exception:
            console.print_exception()

    def sem_cb(self, mess: SemaphoresMessage, queue: Queue):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza o histórico do objeto semáforo local.
        """
        try:
            # Agrupa os semáforos que mudaram de estado num novo dict,
            # agora por objeto traffic_light do SUMO,
            # depois por TrafficLight local
            sumo_tls = set([sumo_tl_id.split("-")[0]
                            for sumo_tl_id in
                            list(mess.changed_semaphores.keys())])
            semaphores: Dict[str, Dict[str, int]] = {}
            for sumo_tl_id in sumo_tls:
                semaphores[sumo_tl_id] = {}
            for tl_id, state in mess.changed_semaphores.items():
                sumo_tl_id = tl_id.split("-")[0]
                semaphores[sumo_tl_id][tl_id] = int(state)
            # Encontrou o nó
            self.network.update_node_history(mess, queue)
        except Exception:
            console.print_exception()

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

    def end(self):
        """
        """
        console.log("Terminando a comunicação na central")
        self.should_exit = True
        self.set_thread.join()
        # Termina a comunicação no otimizador
        self.optimizer.end()

    def __del__(self):
        """
        Interrompe as threads em execução quando o objeto é destruído.
        """
        self.set_thread.join()
