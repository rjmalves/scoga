# Gerenciador da simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 11 de Março de 2020

# Imports gerais de módulos padrão
import pika  # type: ignore
import json
import sumolib
import traci
import threading
import traceback
from typing import Dict
# Imports de módulos específicos da aplicação
from system.clock_generator import ClockGenerator

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
        # Lê as configurações da simulação
        self.load_simulation_config_file(config_file_path)

        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria uma conexão com o broker bloqueante
        self.connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.channel = self.connection.channel()

        # Inicia a exchange e a queue de semáforos, para escutar mudanças nos
        # estados dos semáforos.
        self.init_semaphore_exchange_and_queue()

    def __del__(self):
        """
        Finaliza a conexão via TraCI quando o objeto é destruído.
        """
        traci.close()

    def start(self):
        """
        Cria as threads para escutar as exchanges de interesse, inicia a
        comunicação com o SUMO via TraCI e lê os parâmetros básicos da
        simulação.
        """
        # Inicia a comunicação com a TraCI
        self.init_sumo_communication(self.use_gui)
        # Lê os parâmetros básicos da simulação
        self.read_simulation_params()
        # Cria um gerador de relógio para os controladores
        self.clock_generator = ClockGenerator(self.time_step)

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
                                queue=self.semaphores_queue_name)

    def init_sumo_communication(self, use_gui: bool):
        """
        Confere a existência do binário do SUMO e inicia a comunicação com a
        simulação via TraCI.
        """
        # Inicia a comunicação com a traci
        sumo_exec_str = "sumo-gui" if use_gui else "sumo"
        self.sumo_binary = sumolib.checkBinary(sumo_exec_str)
        traci.start([self.sumo_binary, "-c", self.sim_file_path])

    def read_simulation_params(self) -> bool:
        """
        Lê os parâmetros da simulação que são relevantes para o controle de
        tráfego: passo e semáforos.
        """
        self.time_step = traci.simulation.getDeltaT()
        # TODO - já sei como obter os semáforos (interseções sinalizadas)
        # Agora tem que mapear para grupos semafóricos de fato.
        # TODO - o getControlledLinks retorna [from_lane, to_lane, int_lane].
        # Agora tem que mapear a string que precisa enviar para o SUMO, como
        # o GrGr do caso, para algo que diga que os caracteres 1 e 3 são um
        # grupo semafórico e os 2 e 4, outro.
        # TODO - acho melhor criar um padrão para nomear os semáforos nos
        # controladores. Algo como "TL_ID-S_GROUP_IDX".
        for tl_id in traci.trafficlight.getIDList():
            links = traci.trafficlight.getControlledLinks(tl_id)
            for link in links:
                print(link[0][2], traci.lane.getInternalFoes(link[0][2]))

        return True

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
            self.clock_thread.join()

    def sem_callback(self, ch, method, properties, body):
        """
        Função de callback para quando chegar uma atualização em estado de
        semáforo. Atualiza na simulação o estado do novo semáforo.
        """
        # TODO - especificar tipos no cabeçalho da função
        print("Estado de semáforo atualizado: {}".format(body))
