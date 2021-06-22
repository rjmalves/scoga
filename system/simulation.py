# Gerenciador da simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
from model.messages.shutdown import ShutdownMessage
from model.messages.cycle import CycleMessage
from model.messages.message import Message
from model.traffic.traffic_plan import TrafficPlan
from model.messages.controllerack import ControllerAckMessage
from model.messages.semaphores import SemaphoresMessage
from system.optimization.scoot import EnumOptimizationMethods
import json
import pickle
import time
import sumolib  # type: ignore
import traci  # type: ignore
import threading
import networkx as nx  # type: ignore
from pathlib import Path
from pandas import DataFrame  # type: ignore
from typing import Dict, List, Set, Tuple
from statistics import mean, stdev
from multiprocessing import Queue
# Imports de módulos específicos da aplicação
from system.clock_generator import ClockGenerator
from system.traffic_controller import TrafficController
from model.traffic.vehicle import Vehicle
from model.network.traffic_light import TrafficLight, TLState
from model.network.detector import Detector
from model.network.network import Network

from rich.console import Console
console = Console()


class Simulation:
    """
    Responsável por interfacear as informações dos dispositivos (controladores,
    detectores, etc.) com a simulação do SUMO via TraCI.
    """

    def __init__(self,
                 config_file_path: str,
                 opt_method: EnumOptimizationMethods):
        """
        Lê as configurações de uma simulação de um arquivo e inicia a parte de
        comunicação via RabbitMQ com os controladores e o algoritmo de controle
        de tráfego.
        """
        self.should_exit = False
        self.controller_configs: Dict[str, str] = {}
        self.plans: Dict[str, TrafficPlan] = {}
        self.traffic_lights: Dict[str, TrafficLight] = {}
        self.detectors: Dict[str, Detector] = {}
        self.vehicles: Dict[str, Vehicle] = {}
        # Lê as configurações da simulação
        self.load_simulation_config_file(config_file_path)
        # Cria os controladores
        for cid, ctrl_file in self.controller_configs.items():
            with open(ctrl_file, "r") as filedata:
                data = json.load(filedata)["traffic_plan"]
                self.plans[cid] = TrafficPlan.from_json(data)
        # Cria as variáveis para escutar os ACKs dos controladores
        self.controller_acks = {cid: False for
                                cid in self.controller_configs.keys()}

        # Cria a thread que controla a simulação
        self.sim_thread = threading.Thread(target=self.simulation_control,
                                           name="SimulationControl")

        # Cria a thread que lida com comunicação
        self.comm_thread = threading.Thread(target=self.communication_control,
                                            name="CommunicationControl")

        # Cria a lock para comunicar com a simulação
        self.traci_lock = threading.Lock()
        self.controller_ack_lock = threading.Lock()
        # Constroi o modelo em grafo da rede utilizada para a simulação.
        sumo_net = sumolib.net.readNet(self.net_file_path)
        self.network = Network.from_sumolib_net(sumo_net)
        # Cria o controlador de tráfego
        self.traffic_controller = TrafficController(self.network,
                                                    self.traffic_lights,
                                                    opt_method)
        self._central_queue = Queue()

    @property
    def central_queue(self) -> Queue:
        return self._central_queue

    def end(self):
        """
        """
        console.log("Terminando a comunicação na simulação")
        self.should_exit = True
        # Finaliza a conexão e interrompe as threads
        with self.traci_lock:
            traci.close()
        self.sim_thread.join()
        self.comm_thread.join()
        # Termina a comunicação na central de tráfego
        self.traffic_controller.end()
        # Terminando a comunicação na rede
        self.network.end()
        # Terminando os controladores
        for ctrl_id, q in self.controller_queues.items():
            q.put(ShutdownMessage())

    def __del__(self):
        """
        Finaliza a conexão via TraCI quando o objeto é destruído.
        """
        # Finaliza a conexão e interrompe as threads
        self.sim_thread.join()
        self.comm_thread.join()

    def start(self,
              controller_queues: Dict[str, Queue]):
        """
        Cria as threads para escutar as exchanges de interesse, inicia a
        comunicação com o SUMO via TraCI e lê os parâmetros básicos da
        simulação.
        """
        self.controller_queues = controller_queues
        try:
            # Inicia a comunicação com a TraCI
            self.init_sumo_communication()
            # Lê os parâmetros básicos da simulação
            self.read_simulation_params()
            # Cria um gerador de relógio para os controladores
            self.clock_generator = ClockGenerator(self.time_step)
            # Inicia o otimizador de tráfego
            self.traffic_controller.start(self.plans,
                                          self.detectors,
                                          self.controller_queues)
            # Inicia a thread que controla a simulação
            self.sim_thread.start()
            # Inicia a thread que cuida da comunicação
            self.comm_thread.start()
        except Exception:
            console.print_exception()

    def is_running(self) -> bool:
        with self.traci_lock:
            return (traci.simulation.getMinExpectedNumber() > 0
                    or self.should_exit)

    def load_simulation_config_file(self, config_file_path: str):
        """
        Lê o arquivo de configuração e cria as variáveis internas.
        """
        with open(config_file_path, "r") as json_file:
            data = json.load(json_file)
            # Configurações do SUMO
            self.sim_file_path = data["sim_file_path"]
            self.net_file_path = data["net_file_path"]
            self.result_files_dir = data["result_files_dir"]
            self.use_gui = data["use_gui"]
            # Configurações dos controladores
            for config_dict in data["controller_configs"]:
                for key, val in config_dict.items():
                    self.controller_configs[key] = val

    def init_sumo_communication(self):
        """
        Confere a existência do binário do SUMO e inicia a comunicação com a
        simulação via TraCI.
        """
        # Inicia a comunicação com a traci
        sumo_exec_str = "sumo-gui" if self.use_gui else "sumo"
        self.sumo_binary = sumolib.checkBinary(sumo_exec_str)
        with self.traci_lock:
            traci.start([self.sumo_binary,
                         "-c", self.sim_file_path])

    def read_simulation_params(self):
        """
        Lê os parâmetros da simulação que são relevantes para o controle de
        tráfego: passo, semáforos e detectores.
        """
        with self.traci_lock:
            # Pega o valor temporal do passo
            self.time_step = traci.simulation.getDeltaT()
            # Pega os objetos "traffic_light" do SUMO, que são uma interseção
            # para os controladores, e mapeia em objetos dos controladores.
            for tl_id in traci.trafficlight.getIDList():
                links = traci.trafficlight.getControlledLinks(tl_id)
                linkdicts: List[Dict[str, str]] = []
                for i in links:
                    d = {"from": i[0][0],
                         "to": i[0][1],
                         "internal": i[0][2]}
                    linkdicts.append(d)
                # Pega as conexões de lanes que conflitam com cada link
                foes = [set(traci.lane.getInternalFoes(link[0][2]))
                        for link in links]
                froms = [link[0][0] for link in links]
                # Desconsidera os conflitos por "início de movimento"
                for i, f in enumerate(foes):
                    for ld in linkdicts:
                        ld_in_foe = ld["internal"] in f
                        same_origin = ld["from"] == linkdicts[i]["from"]
                        if ld_in_foe and same_origin:
                            f.remove(ld["internal"])
                internals: List[str] = [d["internal"] for d in linkdicts]
                # Passar o "linkdicts"
                self.__load_sim_traffic_lights(tl_id,
                                               internals,
                                               foes,
                                               froms)
            # Pega os objetos "inductionloop" do SUMO, que são detectores.
            sim_detectors = traci.inductionloop.getIDList()
            for det_id in sim_detectors:
                lane = traci.inductionloop.getLaneID(det_id)
                edge = traci.lane.getEdgeID(lane)
                position = traci.inductionloop.getPosition(det_id)
                self.detectors[det_id] = Detector(det_id, edge, lane, position)

        return True

    def __load_sim_traffic_lights(self,
                                  traffic_light_in_sim_id: str,
                                  internals: List[str],
                                  foes_list: List[Set[str]],
                                  from_list: List[str]):
        """
        Processa as informações dos objetos semáforo na simulação e cria
        os objetos internos para mapeamento com os controladores.
        """
        # Monta o grafo de conflitos com base nos "foes"
        edge_list: List[Tuple[int, int]] = []
        n_foes = len(foes_list)
        for i in range(n_foes - 1):
            for j in range(i+1, n_foes):
                if internals[j] in foes_list[i]:
                    edge_list.append((i, j))
        conf_graph = nx.from_edgelist(edge_list)
        # Realiza a coloração do grafo
        d = nx.greedy_color(conf_graph)
        # Monta os grupos semafóricos com base nas atribuições de cores
        signal_groups: Dict[int, List[int]] = {v: [] for v in set(d.values())}
        for i, gidx in d.items():
            signal_groups[gidx].append(i)
        # Constroi os objetos TL
        for stg, gidx in signal_groups.items():
            lanes = set([from_list[i] for i in gidx])
            tl = TrafficLight(traffic_light_in_sim_id, str(stg), gidx, lanes)
            self.traffic_lights[tl.id_in_controller] = tl

    def simulation_control(self):
        """
        Função responsável por dar passos na simulação e sinalizar o tick do
        relógio.
        """
        # TODO - Substituir por um logging decente.
        console.log("Simulação iniciada!")
        try:
            # Enquanto houver veículos que ainda não chegaram ao destino
            while self.is_running() and not self.should_exit:
                with self.traci_lock:
                    traci.simulationStep()
                    self.detectors_updating()
                    self.network_updating()
                    self.vehicles_updating()
                self.clock_tick()
                # Aguarda todos os controladores
                optimizing = self.traffic_controller.busy_optimizer
                while optimizing and not self.should_exit:
                    optimizing = self.traffic_controller.busy_optimizer
                    time.sleep(1e-6)
        except Exception:
            console.print_exception()
            self.sim_thread.join()

    def communication_control(self):
        """
        Função responsável por lidar com as mensagens recebidas.
        """
        # TODO - Substituir por um logging decente.
        console.log("Comunicação iniciada!")
        try:
            # Enquanto houver veículos que ainda não chegaram ao destino
            while self.is_running() and not self.should_exit:
                mess = self._central_queue.get(block=True)
                if isinstance(mess, Message):
                    self.process_message(mess)
                else:
                    console.log(f"Mensagem inválida: {type(mess)}")
                    self.should_exit = True
                    break
        except Exception:
            console.print_exception()
            self.comm_thread.join()

    def process_message(self, mess: Message):
        if isinstance(mess, ControllerAckMessage):
            self.ack_cb(mess)
        elif isinstance(mess, SemaphoresMessage):
            self.sem_cb(mess)
            self.traffic_controller.sem_cb(mess, self._central_queue)
        elif isinstance(mess, CycleMessage):
            self.traffic_controller.cycle_cb(mess)
        else:
            raise ValueError(f"Mensagem inválida: {type(mess)}")

    def clock_tick(self):
        tick = self.clock_generator.clock_tick()
        if tick is not None:
            self.traffic_controller.clock_cb(tick)
            for c_id, q in self.controller_queues.items():
                q.put(tick)

            while not (self.check_controller_acks() or self.should_exit):
                time.sleep(1e-6)

            self.clear_controller_acks()

    def check_controller_acks(self) -> bool:
        """
        Verifica se todos os controladores já executaram seus passos e
        retorna a informação. Se sim, limpa as variáveis de ACK para o
        próximo passo da simulação.
        """
        current_time = self.clock_generator.current_sim_time
        is_integer = abs(current_time - int(round(current_time))) < 1e-3
        with self.controller_ack_lock:
            if not is_integer:
                return True
            elif all(self.controller_acks.values()):
                return True
            # Caso nem todos tenham respondido
            return False

    def clear_controller_acks(self):
        """
        Verifica se todos os controladores já executaram seus passos e
        retorna a informação. Se sim, limpa as variáveis de ACK para o
        próximo passo da simulação.
        """
        current_time = self.clock_generator.current_sim_time
        is_integer = abs(current_time - int(round(current_time))) < 1e-3
        with self.controller_ack_lock:
            if is_integer:
                # Limpa as variáveis
                for cid in self.controller_acks.keys():
                    self.controller_acks[cid] = False

    def sem_cb(self, mess: SemaphoresMessage):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza na simulação o estado do novo semáforo.
        """
        # Agrupa os semáforos que mudaram de estado num novo dict, agora por
        # objeto traffic_light do SUMO, depois por TrafficLight local
        sumo_tls = set([sumo_tl_id.split("-")[0]
                        for sumo_tl_id in
                        list(mess.changed_semaphores.keys())])
        semaphores: Dict[str, Dict[str, int]] = {}
        for sumo_tl_id in sumo_tls:
            semaphores[sumo_tl_id] = {}
        for tl_id, state in mess.changed_semaphores.items():
            sumo_tl_id = tl_id.split("-")[0]
            semaphores[sumo_tl_id][tl_id] = int(state)
        # Atualiza os objetos semáforo locais
        t = self.clock_generator.current_sim_time
        for sumo_tl_id, tl in semaphores.items():
            for tl_id, state in tl.items():
                self.traffic_lights[tl_id].update_state(TLState(state), t)
        # Pega o instante atual da simulação, para logging
        # sim_time = self.clock_generator.current_sim_time
        # Escreve o novo estado na simulação
        with self.traci_lock:
            for sumo_id, tl in semaphores.items():
                for tl_id, __ in tl.items():
                    tl_obj = self.traffic_lights[tl_id]
                    state = traci.trafficlight.getRedYellowGreenState(sumo_id)
                    new = tl_obj.update_intersection_string(state)
                    traci.trafficlight.setRedYellowGreenState(sumo_id, new)

    def ack_cb(self, mess: ControllerAckMessage):
        """
        Função de callback para quando chegar um ACK de controlador, para
        dar o próximo step na simulação.
        """
        # Processa o corpo da publicação recebida
        with self.controller_ack_lock:
            self.controller_acks[mess.controller_id] = True

    def detectors_updating(self):
        """
        Analisa todos os detectores da simulação para verificar se houve
        mudança de estado e publicar para os controladores interessados.
        """
        # Forma um dicionário com os IDs dos detectores e a ocupação no último
        # passo da simulação. (Para valores de passo pequenos, é 0 ou 1)
        det_ids = traci.inductionloop.getIDList()
        states: Dict[str, bool] = {}
        raw_states = []
        for det_id in det_ids:
            tmp_state = traci.inductionloop.getLastStepOccupancy(det_id)
            raw_states.append(tmp_state)
            states[det_id] = tmp_state > 0
        # Se o estado do detector for diferente do passo anterior, publica
        # e atualiza o estado
        changed: List[str] = []
        for det_id, det in self.detectors.items():
            new_state = states[det_id]
            if det.state != new_state:
                changed.append(det_id)
        # Para cada detector que mudou, publica as alterações
        to_send: List[str] = []
        for det_id, __ in self.detectors.items():
            if det_id in changed:
                to_send.append(det_id)
        if len(to_send) > 0:
            # Constroi o corpo da mensagem
            body = [(det_id, states[det_id]) for det_id in to_send]
            # Publica a mensagem
            self.det_bus.Publish(payload=str(body),
                                 topic="detectors")
        for det_id in changed:
            # Atualiza o estado dos detectores
            sim_time = self.clock_generator.current_sim_time
            self.detectors[det_id].update_detection_history(sim_time,
                                                            states[det_id])

    def network_updating(self):
        """
        Função responsável por varrer a rede simulada no SUMO e extrair, para
        cada elemento da rede (Node, Edge e Lane), variáveis de interesse. É a
        fonte de dados ideais, das quais os detectores servem como aproximação.
        """
        # Guarda o instante de tempo atual
        time_instant = self.clock_generator.current_sim_time
        # Para cada Edge na simulação
        for edge_id, edge in self.network.edges.items():
            # Adquire dados de tráfego
            average_speed = traci.edge.getLastStepMeanSpeed(edge_id)
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
            # waiting_time = traci.edge.getWaitingTime(edge_id)
            # halting_count = traci.edge.getLastStepHaltingNumber(edge_id)
            # travel_time = traci.edge.getTraveltime(edge_id)
            avg_occupancy = traci.edge.getLastStepOccupancy(edge_id)
            # Atualiza a Edge
            self.network.update_edge_traffic_data(edge_id,
                                                  time_instant,
                                                  average_speed,
                                                  vehicle_count,
                                                  #  waiting_time,
                                                  #  halting_count,
                                                  #  average_speed,
                                                  avg_occupancy)
            # Adquire os dados ambientais
            # CO2_emission = traci.edge.getCO2Emission(edge_id)
            # CO_emission = traci.edge.getCOEmission(edge_id)
            # HC_emission = traci.edge.getHCEmission(edge_id)
            # NOx_emission = traci.edge.getNOxEmission(edge_id)
            # PMx_emission = traci.edge.getPMxEmission(edge_id)
            # noise_emission = traci.edge.getNoiseEmission(edge_id)
            # fuel_consumption = traci.edge.getFuelConsumption(edge_id)
            # electricity = traci.edge.getElectricityConsumption(edge_id)
            # edge.history.update_environmental_data(CO2_emission,
            #                                        CO_emission,
            #                                        HC_emission,
            #                                        NOx_emission,
            #                                        PMx_emission,
            #                                        noise_emission,
            #                                        fuel_consumption,
            #                                        electricity)

            # Para cada Lane na Edge
            for lane_id, _ in edge.lanes.items():
                # Adquire dados de tráfego
                # Antes de 1 segundo, ignora o tempo de viagem
                average_speed = traci.lane.getLastStepMeanSpeed(lane_id)
                vehicle_count = traci.lane.getLastStepVehicleNumber(lane_id)
                # waiting_time = traci.lane.getWaitingTime(lane_id)
                # halting_count = traci.lane.getLastStepHaltingNumber(lane_id)
                # travel_time = traci.lane.getTraveltime(lane_id)
                avg_occupancy = traci.lane.getLastStepOccupancy(lane_id)
                # Atualiza a Lane
                self.network.update_lane_traffic_data(edge_id,
                                                      lane_id,
                                                      time_instant,
                                                      average_speed,
                                                      vehicle_count,
                                                      #  waiting_time,
                                                      #  halting_count,
                                                      #  travel_time,
                                                      avg_occupancy)
                # Adquire os dados ambientais
                # CO2_emission = traci.lane.getCO2Emission(lane_id)
                # CO_emission = traci.lane.getCOEmission(lane_id)
                # HC_emission = traci.lane.getHCEmission(lane_id)
                # NOx_emission = traci.lane.getNOxEmission(lane_id)
                # PMx_emission = traci.lane.getPMxEmission(lane_id)
                # noise_emission = traci.lane.getNoiseEmission(lane_id)
                # fuel_consumption = traci.lane.getFuelConsumption(lane_id)
                # electricity = traci.lane.getElectricityConsumption(lane_id)
                # lane.history.update_environmental_data(CO2_emission,
                #                                        CO_emission,
                #                                        HC_emission,
                #                                        NOx_emission,
                #                                        PMx_emission,
                #                                        noise_emission,
                #                                        fuel_consumption,
                #                                        electricity)

    def vehicles_updating(self):
        """
        Função responsável por obter os IDs dos veículos envolvidos na
        simulação, bem como seus instantes de tempo de início e de final
        de participação.
        """
        TOL_SPEED = 1e-1
        sim_time = self.clock_generator.current_sim_time
        # Cria os veículos que entraram agora
        loaded = traci.simulation.getLoadedIDList()
        for vid in loaded:
            self.vehicles[vid] = Vehicle(vid)
            self.vehicles[vid].departing_time = sim_time
        arrived = traci.simulation.getArrivedIDList()
        for vid in arrived:
            self.vehicles[vid].arriving_time = sim_time
        for vid, vehicle in self.vehicles.items():
            if vehicle.ended:
                continue
            if vehicle.started and not vehicle.ended:
                speed = traci.vehicle.getSpeed(vid)
                if not vehicle.accelerated and abs(speed) > TOL_SPEED:
                    self.vehicles[vid].accelerated = True
                if abs(speed) < TOL_SPEED:
                    self.vehicles[vid].stopped_time += self.time_step

    def export_histories(self):
        """
        Função para exportar todos os hitóricos de detecção para o diretório
        especificado no arquivo de configuração. São exportados os históricos
        de detectores e de estágios dos semáforos.
        """
        tt = [v.travel_time for v in self.vehicles.values()]
        console.log(f"TEMPO DE VIAGEM: TOTAL = {sum(tt)} -" +
                    f" MEDIA = {mean(tt)} - DESVIO = {stdev(tt)} -" +
                    f" MAX = {max(tt)} - MIN = {min(tt)}")

        current_time = str(int(time.time()))
        full_dir = self.result_files_dir + "/" + current_time + "/"
        # Verifica se o path existe e cria se não existir
        Path(full_dir).mkdir(parents=True, exist_ok=True)
        # Exporta os dados de veículos
        veh_hists = DataFrame()
        vehicles = [v for v in self.vehicles.values()]
        veh_hists["id"] = [v.id for v in vehicles]
        veh_hists["departing_time"] = [v.departing_time for v in vehicles]
        veh_hists["arriving_time"] = [v.arriving_time for v in vehicles]
        veh_hists["travel_time"] = [v.travel_time for v in vehicles]
        veh_hists["stopped_time"] = [v.stopped_time for v in vehicles]
        filename = full_dir + "vehicles.pickle"
        with open(filename, "wb") as f:
            pickle.dump(veh_hists, f)
        # Exporta os dados históricos dos grupos semafóricos
        filename = full_dir + "trafficlights.pickle"
        t = self.clock_generator.current_sim_time
        tl_hists = DataFrame()
        for tl_id, tl in self.traffic_lights.items():
            data = tl.export_state_history(t)
            tl_hists = DataFrame.append(tl_hists,
                                        data,
                                        ignore_index=True,
                                        sort=False)
        with open(filename, "wb") as f:
            pickle.dump(tl_hists, f)
        # Exporta os dados históricos de detectores
        filename = full_dir + "detectors.pickle"
        det_hists = DataFrame()
        for det_id, det in self.detectors.items():
            data = det.export_detection_history(t)
            det_hists = DataFrame.append(det_hists,
                                         data,
                                         ignore_index=True,
                                         sort=False)
        with open(filename, "wb") as f:
            pickle.dump(det_hists, f)
        # Exporta os dados históricos dos nós
        node_hists = self.traffic_controller.export_node_histories()
        filename = full_dir + "nodes.pickle"
        with open(filename, "wb") as f:
            pickle.dump(node_hists, f)
        # Exporta os dados históricos das vias
        edge_hists = self.traffic_controller.export_edge_histories()
        filename = full_dir + "edges.pickle"
        with open(filename, "wb") as f:
            pickle.dump(edge_hists, f)
        # Exporta os dados históricos das faixas
        lane_hists = self.traffic_controller.export_lane_histories()
        filename = full_dir + "lanes.pickle"
        with open(filename, "wb") as f:
            pickle.dump(lane_hists, f)
