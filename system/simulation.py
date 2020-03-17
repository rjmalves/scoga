# Gerenciador da simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import ast
import pika  # type: ignore
import json
import time
import sumolib  # type: ignore
import traci  # type: ignore
import threading
import traceback
from typing import Dict, List, Set
# Imports de módulos específicos da aplicação
from system.clock_generator import ClockGenerator
from model.traffic.controller import Controller
from model.network.traffic_light import TrafficLight, TLState


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
        # Cria a thread que escuta semaphores
        self.sem_thread = threading.Thread(target=self.semaphores_listening,
                                           daemon=True)
        # Cria a lock para comunicar com a simulação
        self.traci_lock = threading.Lock()

    def __del__(self):
        """
        Finaliza a conexão via TraCI quando o objeto é destruído.
        """
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
            # Inicia a comunicação com a TraCI
            self.init_sumo_communication(self.use_gui)
            # Lê os parâmetros básicos da simulação
            self.read_simulation_params()
            # Cria um gerador de relógio para os controladores
            self.clock_generator = ClockGenerator(self.time_step)
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
            # Enquanto houver veículos que ainda não chegaram ao destino
            while self.is_running():
                with self.traci_lock:
                    traci.simulationStep()
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

    def sem_callback(self, ch, method, properties, body):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza na simulação o estado do novo semáforo.
        """
        # TODO - especificar tipos no cabeçalho da função
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
        for sumo_tl_id, tl in semaphores.items():
            for tl_id, state in tl.items():
                self.traffic_lights[tl_id].state = TLState(state)
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
