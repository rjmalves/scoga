# Gerenciador da simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
from model.traffic.traffic_plan import TrafficPlan
from model.messages.controllerack import ControllerAckMessage
from model.messages.semaphores import SemaphoresMessage
from system.optimization.scoot import EnumOptimizationMethods
import pika  # type: ignore
from PikaBus.PikaBusSetup import PikaBusSetup
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
# Imports de módulos específicos da aplicação
from system.clock_generator import ClockGenerator
from system.traffic_controller import TrafficController
from model.traffic.vehicle import Vehicle
from model.traffic.controller import Controller
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

        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
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
        # Parâmetro para finalizar a simulação
        self.should_stop = False

    def stop_communication(self):
        """
        """
        console.log("Terminando a comunicação na simulação")
        self.should_stop = True
        # Finaliza a conexão e interrompe as threads
        time.sleep(1e-1)
        with self.traci_lock:
            traci.close()
        # Termina a comunicação na central de tráfego
        self.traffic_controller.stop_communication()
            
        self._ack_pika_bus.StopConsumers()
        self._det_pika_bus.StopConsumers()
        self._sem_pika_bus.StopConsumers()
        self._ack_pika_bus.Stop()
        self._det_pika_bus.Stop()
        self._sem_pika_bus.Stop()


    def __del__(self):
        """
        Finaliza a conexão via TraCI quando o objeto é destruído.
        """
        # Finaliza a conexão e interrompe as threads
        with self.traci_lock:
            traci.close()
        self._ack_pika_bus.StopConsumers()
        self._det_pika_bus.StopConsumers()
        self._sem_pika_bus.StopConsumers()
        self._ack_pika_bus.Stop()
        self._det_pika_bus.Stop()
        self._sem_pika_bus.Stop()
        self.sim_thread.join()

    def start(self):
        """
        Cria as threads para escutar as exchanges de interesse, inicia a
        comunicação com o SUMO via TraCI e lê os parâmetros básicos da
        simulação.
        """
        try:
            # Inicia a comunicação com a TraCI
            self.init_sumo_communication()
            # Lê os parâmetros básicos da simulação
            self.read_simulation_params()
            # Cria um gerador de relógio para os controladores
            self.clock_generator = ClockGenerator(self.time_step)
            # Inicia a exchange e a queue de semáforos, para escutar mudanças nos
            # estados dos semáforos.
            self.init_semaphore_connection()
            self.init_detector_connection()
            # Inicia a escuta de setpoints
            self.init_ack_connection()
            # Inicia o otimizador de tráfego
            self.traffic_controller.start(self.plans,
                                          self.detectors)
            # Inicia a thread que controla a simulação
            self.sim_thread.start()
        except Exception:
            console.print_exception()

    def is_running(self) -> bool:
        with self.traci_lock:
            return (traci.simulation.getMinExpectedNumber() > 0
                    or self.should_stop)

    def init_ack_connection(self):
        """
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        self._ack_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue='sim_ack_queue',
                                          defaultSubscriptions='controllers')
        self._ack_pika_bus.AddMessageHandler(self.ctrl_cb)
        self._ack_pika_bus.StartConsumers()
        self.ack_bus = self._ack_pika_bus.CreateBus()
        self.ack_bus.Subscribe('controllers')

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

    def init_semaphore_connection(self):
        """
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self._sem_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue='sim_sem_queue',
                                          defaultSubscriptions='semaphores')
        self._sem_pika_bus.AddMessageHandler(self.sem_cb)
        self._sem_pika_bus.StartConsumers()
        self.sem_bus = self._sem_pika_bus.CreateBus()

    def init_detector_connection(self):
        """
        """
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self._det_pika_bus = PikaBusSetup(self.parameters,
                                          defaultListenerQueue='sim_det_queue')
        self._det_pika_bus.StartConsumers()
        self.det_bus = self._det_pika_bus.CreateBus()

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
                    d = {"from": i[0][0], "to": i[0][1], "internal": i[0][2]}
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
        print(list(self.traffic_lights.keys()))

    def simulation_control(self):
        """
        Função responsável por dar passos na simulação e sinalizar o tick do
        relógio.
        """
        # TODO - Substituir por um logging decente.
        console.log("Simulação iniciada!")
        try:
            # Enquanto houver veículos que ainda não chegaram ao destino
            while self.is_running():
                with self.traci_lock:
                    traci.simulationStep()
                    self.detectors_updating()
                    self.network_updating()
                    self.vehicles_updating()
                time.sleep(1.5e-3)
                self.clock_generator.clock_tick()
                # Aguarda todos os controladores
                optimizing = self.traffic_controller.busy_optimizer
                while not self.check_controller_acks() or optimizing:
                    optimizing = self.traffic_controller.busy_optimizer
                    time.sleep(1.5e-3)
                # Se todos responderam, limpa as flags de ack
                self.clear_controller_acks()
        except Exception:
            console.print_exception()
            self.sim_thread.join()

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
                    # console.log("LIMPEI ACK")
                    self.controller_acks[cid] = False

    def sem_cb(self, **kwargs):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza na simulação o estado do novo semáforo.
        """
        # Processa o corpo da publicação recebida
        message = SemaphoresMessage.from_dict(kwargs["payload"])
        # Agrupa os semáforos que mudaram de estado num novo dict, agora por
        # objeto traffic_light do SUMO, depois por TrafficLight local
        sumo_tls = set([sumo_tl_id.split("-")[0]
                        for sumo_tl_id in list(message.changed_semaphores.keys())])
        semaphores: Dict[str, Dict[str, int]] = {}
        for sumo_tl_id in sumo_tls:
            semaphores[sumo_tl_id] = {}
        for tl_id, state in message.changed_semaphores.items():
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

    def ctrl_cb(self, **kwargs):
        """
        Função de callback para quando chegar um ACK de controlador, para
        dar o próximo step na simulação.
        """
        # Processa o corpo da publicação recebida
        with self.controller_ack_lock:
            message = ControllerAckMessage.from_dict(kwargs['payload'])
            self.controller_acks[message.controller_id] = True

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
        sim_time = self.clock_generator.current_sim_time
        # Cria os veículos que entraram agora
        loaded = traci.simulation.getLoadedIDList()
        for vid in loaded:
            self.vehicles[vid] = Vehicle(vid)
            self.vehicles[vid].departing_time = sim_time
        arrived = traci.simulation.getArrivedIDList()
        for vid in arrived:
            self.vehicles[vid].arriving_time = sim_time

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
