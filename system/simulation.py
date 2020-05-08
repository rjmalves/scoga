# Gerenciador da simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import ast
import pika  # type: ignore
import json
import pickle
import time
import sumolib  # type: ignore
import traci  # type: ignore
import threading
import traceback
from pathlib import Path
from typing import Dict, List, Set
# Imports de módulos específicos da aplicação
from system.clock_generator import ClockGenerator
from system.traffic_optimizer import TrafficOptimizer
from model.traffic.controller import Controller
from model.network.traffic_light import TrafficLight, TLState
from model.network.detector import Detector


class Simulation:
    """
    Responsável por interfacear as informações dos dispositivos (controladores,
    detectores, etc.) com a simulação do SUMO via TraCI.
    """

    def __init__(self, config_file_path: str):
        """
        Lê as configurações de uma simulação de um arquivo e inicia a parte de
        comunicação via RabbitMQ com os controladores e o algoritmo de controle
        de tráfego.
        """
        self.controller_configs: Dict[str, str] = {}
        self.controllers: Dict[str, Controller] = {}
        self.traffic_lights: Dict[str, TrafficLight] = {}
        self.detectors: Dict[str, Detector] = {}
        # Lê as configurações da simulação
        self.load_simulation_config_file(config_file_path)
        # Cria os controladores
        for ctrl_id, __ in self.controller_configs.items():
            self.controllers[ctrl_id] = Controller(ctrl_id)

        # Cria a thread que controla a simulação
        self.sim_thread = threading.Thread(target=self.simulation_control,
                                           daemon=True)

        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria uma conexão com o broker bloqueante
        self.connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.channel = self.connection.channel()

        # Inicia a exchange e a queue de semáforos, para escutar mudanças nos
        # estados dos semáforos.
        self.init_semaphore_exchange_and_queue()
        # Inicia a exchange de detectores
        self.init_detectors_exchange()
        # Cria a thread que escuta semaphores
        self.sem_thread = threading.Thread(target=self.semaphores_listening,
                                           daemon=True)

        # Cria a lock para comunicar com a simulação
        self.traci_lock = threading.Lock()

        # Cria o otimizador de tráfego
        self.traffic_optimizer = TrafficOptimizer(self.detectors)

    def __del__(self):
        """
        Finaliza a conexão via TraCI quando o objeto é destruído.
        """
        # Finaliza a conexão e interrompe as threads
        with self.traci_lock:
            traci.close()
        self.sim_thread.join()
        self.sem_thread.join()

    def start(self):
        """
        Cria as threads para escutar as exchanges de interesse, inicia a
        comunicação com o SUMO via TraCI e lê os parâmetros básicos da
        simulação.
        """
        try:
            # Inicia os controladores
            for ctrl_id, ctrl_file in self.controller_configs.items():
                self.controllers[ctrl_id].start(ctrl_file)
                det_ids = self.controllers[ctrl_id].det_ids
                for det_id in det_ids:
                    if det_id not in self.detectors.keys():
                        self.detectors[det_id] = Detector(det_id, ctrl_id)
                    else:
                        self.detectors[det_id].add_controller(ctrl_id)
            # Inicia a comunicação com a TraCI
            self.init_sumo_communication(self.use_gui)
            # Lê os parâmetros básicos da simulação
            self.read_simulation_params()
            # Cria um gerador de relógio para os controladores
            self.clock_generator = ClockGenerator(self.time_step)
            # Inicia o otimizador de tráfego
            self.traffic_optimizer.start(self.controllers)
            # Inicia a thread que escuta semáforos
            self.sem_thread.start()
            # Inicia a thread que controla a simulação
            self.sim_thread.start()
        except Exception:
            traceback.print_exc()

    def is_running(self) -> bool:
        with self.traci_lock:
            return traci.simulation.getMinExpectedNumber() > 0

    def load_simulation_config_file(self, config_file_path: str):
        """
        Lê o arquivo de configuração e cria as variáveis internas.
        """
        with open(config_file_path, "r") as json_file:
            data = json.load(json_file)
            # Configurações do SUMO
            self.sim_file_path = data["sim_file_path"]
            self.result_files_dir = data["result_files_dir"]
            self.use_gui = data["use_gui"]
            # Configurações dos controladores
            for config_dict in data["controller_configs"]:
                for key, val in config_dict.items():
                    self.controller_configs[key] = val

    def init_semaphore_exchange_and_queue(self):
        """
        Declara a exchange para pegar as atualizações de semáforos
        e a relaciona com a fila exclusiva.
        """
        # Declara as exchanges
        self.channel.exchange_declare(exchange="semaphores",
                                      exchange_type="topic")
        # Cria as queues e realiza um bind no canal
        declare_result = self.channel.queue_declare(queue="", exclusive=True)
        self.semaphores_queue_name = declare_result.method.queue
        self.channel.queue_bind(exchange="semaphores",
                                routing_key="*",
                                queue=self.semaphores_queue_name)

    def init_detectors_exchange(self):
        """
        Declara a exchange para publicar para o controlador sempre que houver
        uma mudança no estado de um detector.
        """
        # Declara a exchange
        self.channel.exchange_declare(exchange="detectors",
                                      exchange_type="topic")

    def init_sumo_communication(self, use_gui: bool):
        """
        Confere a existência do binário do SUMO e inicia a comunicação com a
        simulação via TraCI.
        """
        # Inicia a comunicação com a traci
        sumo_exec_str = "sumo-gui" if use_gui else "sumo"
        self.sumo_binary = sumolib.checkBinary(sumo_exec_str)
        with self.traci_lock:
            traci.start([self.sumo_binary, "-c", self.sim_file_path])

    def read_simulation_params(self):
        """
        Lê os parâmetros da simulação que são relevantes para o controle de
        tráfego: passo e semáforos.
        """
        with self.traci_lock:
            # Pega o valor temporal do passo
            self.time_step = traci.simulation.getDeltaT()
            # Pega os objetos "traffic_light" do SUMO, que são uma interseção
            # para os controladores, e mapeia em objetos dos controladores.
            for tl_id in traci.trafficlight.getIDList():
                links = traci.trafficlight.getControlledLinks(tl_id)
                # Pega as conexões de lanes que conflitam com cada link
                foes = [set(traci.lane.getInternalFoes(link[0][2]))
                        for link in links]
                self.__load_sim_traffic_lights(tl_id, foes)
            # Pega os objetos "inductionloop" do SUMO, que são detectores.
            sim_detectors = traci.inductionloop.getIDList()
            for det_id in sim_detectors:
                if det_id in self.detectors.keys():
                    lane = traci.inductionloop.getLaneID(det_id)
                    position = traci.inductionloop.getPosition(det_id)
                    print("{}, {}".format(lane, position))
                    self.detectors[det_id].add_sim_info(lane, position)

        return True

    def __load_sim_traffic_lights(self,
                                  traffic_light_in_sim_id: str,
                                  foes_list: List[Set[str]]):
        """
        Processa as informações dos objetos semáforo na simulação e cria
        os objetos internos para mapeamento com os controladores.
        """
        considered = [False] * len(foes_list)  # Lista dos grupos já vistos
        # Visita todos os conjuntos de conflito. Conjuntos de conflito
        # iguais são colocados no mesmo TrafficLight.
        for i in range(len(foes_list) - 1):
            if considered[i]:
                continue
            else:
                considered[i] = True
                to_consider = foes_list[i]
                group_idxs = [i]
                for j in range(i + 1, len(foes_list)):
                    if foes_list[j] == to_consider:
                        group_idxs.append(j)
                        considered[j] = True
                # Se considerou todos os grupos com conflitos iguais:
                tl = TrafficLight(traffic_light_in_sim_id, group_idxs)
                self.traffic_lights[tl.id_in_controller] = tl

    def simulation_control(self):
        """
        Função responsável por dar passos na simulação e sinalizar o tick do
        relógio.
        """
        # TODO - Substituir por um logging decente.
        print("Simulação iniciada!")
        try:
            # TODO - varre a simulação e constroi os objetos detectores para
            # mapear os existentes na simulação com os dos controladores.

            # Enquanto houver veículos que ainda não chegaram ao destino
            while self.is_running():
                with self.traci_lock:
                    traci.simulationStep()
                    self.detectors_updating()
                self.clock_generator.clock_tick()
                time.sleep(1e-3)
        except Exception:
            traceback.print_exc()
            self.sim_thread.join()

    def semaphores_listening(self):
        """
        Função da thread que escuta mudanças nos semáforos e atualiza a
        simulação.
        """
        # TODO - Substituir por um logging decente.
        print("Simulação começou a escutar semaphores!")
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            # Como é fanout, não precisa da binding key.
            self.channel.basic_consume(queue=self.semaphores_queue_name,
                                       on_message_callback=self.sem_callback)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.sem_thread.join()

    def sem_callback(self,
                     ch: pika.adapters.blocking_connection.BlockingChannel,
                     method: pika.spec.Basic.Deliver,
                     property: pika.spec.BasicProperties,
                     body: bytes):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza na simulação o estado do novo semáforo.
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
        t = self.clock_generator.current_sim_time
        for sumo_tl_id, tl in semaphores.items():
            for tl_id, state in tl.items():
                self.traffic_lights[tl_id].update_state(TLState(state), t)
        # Pega o instante atual da simulação, para logging
        sim_time = self.clock_generator.current_sim_time
        # Escreve o novo estado na simulação
        with self.traci_lock:
            for sumo_id, tl in semaphores.items():
                for tl_id, __ in tl.items():
                    # TODO - substituir por um logging decente
                    print("Semáforo atualizado: {} em {}"
                          .format(tl_id, sim_time))
                    tl_obj = self.traffic_lights[tl_id]
                    state = traci.trafficlight.getRedYellowGreenState(sumo_id)
                    new = tl_obj.update_intersection_string(state)
                    traci.trafficlight.setRedYellowGreenState(sumo_id, new)

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
        # Para cada controlador, faz a lista dos detectores a enviar
        for ctrl_id, ctrl in self.controllers.items():
            to_send: List[str] = []
            for det_id in ctrl.det_ids:
                if det_id in changed:
                    to_send.append(det_id)
            if len(to_send) > 0:
                # Constroi o corpo da mensagem
                body = [(det_id, states[det_id]) for det_id in to_send]
                # TODO - substituir por um logging decente
                sim_time = self.clock_generator.current_sim_time
                print("Detectores atualizados: {} em {}".format(to_send,
                                                                sim_time))
                # Publica a mensagem
                self.channel.basic_publish(exchange="detectors",
                                           routing_key=str(ctrl_id),
                                           body=str(body))
        for det_id in changed:
            # Atualiza o estado dos detectores
            sim_time = self.clock_generator.current_sim_time
            self.detectors[det_id].update_detection_history(sim_time,
                                                            states[det_id])

    def export_histories(self):
        """
        Função para exportar todos os hitóricos de detecção para o diretório
        especificado no arquivo de configuração. São exportados os históricos
        de detectores e de estágios dos semáforos.
        """
        current_time = str(int(time.time()))
        full_dir = self.result_files_dir + "/" + current_time + "/"
        # Verifica se o path existe e cria se não existir
        Path(full_dir).mkdir(parents=True, exist_ok=True)
        # Exporta os dados históricos de controladores
        for tl_id, tl in self.traffic_lights.items():
            filename = full_dir + "trafficlight_" + tl_id + ".pickle"
            history = tl.export_state_history()
            with open(filename, "wb") as f:
                pickle.dump(history, f)
        # Exporta os dados históricos de detectores
        for det_id, det in self.detectors.items():
            filename = full_dir + "detector_" + det_id + ".pickle"
            history = det.export_detection_history()
            with open(filename, "wb") as f:
                pickle.dump(history, f)
        # Exporta os dados históricos das interseções
        histories = self.traffic_optimizer.export_inter_histories()
        for inter_id, hist in histories.items():
            filename = full_dir + "intersection_" + inter_id + ".pickle"
            with open(filename, "wb") as f:
                pickle.dump(hist, f)
        # TODO - Exporta os históricos dos parâmetros de controle (SCO)
