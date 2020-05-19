# Otimizador de tráfego para realizar o cálculo dos novos parâmetros de
# controle de cada controlador em uma rede do SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 14 de Maio de 2020

# Imports gerais de módulos padrão
import ast
import time
import pika  # type: ignore
import threading
import traceback
from queue import SimpleQueue
from typing import Dict, List
# Imports de módulos específicos da aplicação
from model.traffic.setpoint import Setpoint
from model.network.network import Network


class ScootOptimizer:
    """
    Responsável pela realização da otimização baseada em ciclos, a
    SCOOT. É instanciado dentro do TrafficControler e se inscreve
    para receber notificações sobre avanço de ciclos de cada nó
    controlador.
    """
    def __init__(self, network: Network):
        # Obtém uma referência para a rede, com históricos.
        self.network = network
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        # Cria a exchange e a fila de observação de ciclos
        self.init_cycle_connection()
        # Cria a thread que escuta cycle
        self.cycle_thread = threading.Thread(target=self.cycle_listening,
                                             daemon=True)
        # Cria a thread que realiza a otimização
        self.optimization_thread = threading.Thread(target=self.optimizing,
                                                    daemon=True)
        # Cria a fila que recebe as otimizações a serem realizadas
        self.optimization_queue = SimpleQueue()
        # Cria a fila que recebe os novos setpoints para serem
        # obtidos pelo controller
        self.setpoint_queue = SimpleQueue()

    def start(self, setpoints: Dict[str, Setpoint]):
        """
        """
        try:
            # Armazena os setpoints
            self.setpoints = setpoints
            # Inicia a thread que escuta os ciclos
            self.cycle_thread.start()
            # Inicia a thread de otimização
            self.optimization_thread.start()
        except Exception:
            traceback.print_exc()

    def init_cycle_connection(self):
        """
        """
        # Cria uma conexão com o broker bloqueante
        self.cycle_connection = pika.BlockingConnection(self.parameters)
        # Cria um canal dentro da conexão
        self.cycle_channel = self.cycle_connection.channel()
        # Declara as exchanges
        self.cycle_channel.exchange_declare(exchange="cycles",
                                            exchange_type="topic")
        # Cria as queues e realiza um bind no canal
        declare_result = self.cycle_channel.queue_declare(queue="",
                                                          exclusive=True)
        self.cycle_queue_name = declare_result.method.queue
        self.cycle_channel.queue_bind(exchange="cycles",
                                      routing_key="*",
                                      queue=self.cycle_queue_name)

    def cycle_listening(self):
        """
        """
        # TODO - Substituir por um logging decente.
        print("Otimizador começando a escutar ciclos!")
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            self.cycle_channel.basic_consume(queue=self.cycle_queue_name,
                                             on_message_callback=self.cycle_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.cycle_channel.start_consuming()
        except Exception:
            traceback.print_exc()
            self.cycle_thread.join()

    def cycle_cb(self,
                 ch: pika.adapters.blocking_connection.BlockingChannel,
                 method: pika.spec.Basic.Deliver,
                 property: pika.spec.BasicProperties,
                 body: bytes):
        """
        """
        # Processa o corpo da publicação recebida
        body_dict: dict = ast.literal_eval(body.decode())
        # Coloca o elemento na fila de otimizações
        self.optimization_queue.put(body_dict)

    def new_setpoints(self) -> List[Dict[str, Setpoint]]:
        """
        Retorna todos os novos setpoints resultados de otimização
        existentes na fila de resultados.
        """
        new_setpoints: List[Dict[str, Setpoint]] = []
        while not self.setpoint_queue.empty():
            new_setpoints.append(self.setpoint_queue.get())
        return new_setpoints

    def optimizing(self):
        """
        """
        # TODO - Substituir por um logging decente.
        print("Otimizador começando a otimizar!")
        while True:
            try:
                # Se existirem elementos na fila para otimizar
                if not self.optimization_queue.empty():
                    # Extrai e chama a otimização no elemento
                    opt_dict = self.optimization_queue.get()
                    print("Otimizando {}".format(opt_dict))
                    new_setpoint = self.optimize(opt_dict)
                    self.setpoint_queue.put(new_setpoint)
                else:
                    # Senão
                    time.sleep(0.1)
            except Exception:
                traceback.print_exc()
                self.optimization_thread.join()

    def optimize(self, opt_dict: dict) -> Dict[str, Setpoint]:
        """
        """
        # Comportamento padrão - retornar o setpoint atual
        # TODO - OTIMIZAR!
        node_id = opt_dict["id"]
        return {node_id: self.setpoints[node_id]}
