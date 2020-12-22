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
import logging
import random
import numpy as np
from statistics import mean
from queue import SimpleQueue
from typing import Dict, List
from deap import creator, base, tools
# Imports de módulos específicos da aplicação
from model.traffic.setpoint import Setpoint
from model.traffic.controller import Controller
from model.network.traffic_light import TrafficLight
from model.network.network import Network


random.seed(42)
creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMin)
toolbox = base.Toolbox()
toolbox.register("attr_float", random.random)


class ScootOptimizer:
    """
    Responsável pela realização da otimização baseada em ciclos, a
    SCOOT. É instanciado dentro do TrafficControler e se inscreve
    para receber notificações sobre avanço de ciclos de cada nó
    controlador.
    """
    def __init__(self,
                 network: Network,
                 setpoints: Dict[str, Setpoint],
                 controllers: Dict[str, Controller],
                 traffic_lights: Dict[str, TrafficLight]):
        self.logger = logging.getLogger(__name__)
        # Obtém uma referência para a rede, com históricos.
        self.network = network
        self.setpoints = setpoints
        self.controllers = controllers
        self.traffic_lights = traffic_lights
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
        self.optimization_queue: SimpleQueue = SimpleQueue()
        # Cria a fila que recebe os novos setpoints para serem
        # obtidos pelo controller
        self.setpoint_queue: SimpleQueue = SimpleQueue()

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
        self.logger.info("Otimizador inscrito em cycles!")
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
        self.logger.info("Otimizador iniciado!")
        while True:
            try:
                # Se existirem elementos na fila para otimizar
                if not self.optimization_queue.empty():
                    # Extrai e chama a otimização no elemento
                    opt_dict = self.optimization_queue.get()
                    self.logger.info("Otimizando Controlador {}: Ciclo #{}"
                                     .format(opt_dict["id"],
                                             opt_dict["cycle"]))
                    new_setpoint = self.optimize(opt_dict)
                    self.setpoint_queue.put(new_setpoint)
                else:
                    # Senão
                    time.sleep(0.1)
            except Exception:
                traceback.print_exc()
                self.optimization_thread.join()

    def get_individual_shape(self, node_id: str) -> int:
        """
        Gera o formato da lista que representa um indivíduo na
        população da metaheurística.
        """
        return self.controllers[node_id].traffic_plan.stage_count

    def get_current_opt_values(self, node_id: str) -> List[float]:
        """
        Extrai os valores dos parâmetros de otimização atualmente
        nos elementos da rede.
        """
        # Por enquanto trata somente de splits
        return self.setpoints[node_id].splits

    def get_desired_opt_values(self, node_id: str, cycle: int) -> List[float]:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema.
        """
        n = self.network
        # Verifica o histórico mais recente para obter os splits
        # desejados
        total_occ = 0.
        stages_occs: List[float] = []
        node_hist = self.network.nodes[node_id].history
        ti, tf = node_hist.get_cycle_time_boundaries(cycle)
        # Obtém os IDs dos semáforos de cada estágio
        stages_lanes: List[List[str]] = []
        for tl_id in self.controllers[node_id].tl_ids:
            a_id = ""
            # Obtém as lanes que chegam no nó por estágio
            for real_id, tl in self.traffic_lights.items():
                if tl.id_in_controller == tl_id:
                    a_id = real_id
                    break
            stages_lanes.append(self.traffic_lights[a_id].from_lanes)
        # Para cada estágio
        for _, lanes in enumerate(stages_lanes):
            stage_occ: List[float] = []
            for lane_id in lanes:
                # Encontra a edge da lane
                eid = ""
                for edge_id, e in n.edges.items():
                    if lane_id in e.lanes.keys():
                        eid = edge_id
                        break
                # Obtem os dados de tráfego da lane
                lane = self.network.edges[eid].lanes[lane_id]
                data = lane.history.get_average_traffic_data_in_time(ti, tf)
                stage_occ.append(data["occupancy"])
            total_occ += mean(stage_occ)
            stages_occs.append(mean(stage_occ))
        for i in range(len(stages_occs)):
            if total_occ != 0:
                stages_occs[i] /= total_occ
        # Por enquanto trata somente de splits
        return stages_occs

    def prepare_metaheuristic(self, node_id: str):
        """
        """
        INDIV_SIZE = self.get_individual_shape(node_id)
        toolbox.register("individual",
                         tools.initRepeat,
                         creator.Individual,
                         toolbox.attr_float,
                         n=INDIV_SIZE)
        toolbox.register("population",
                         tools.initRepeat,
                         list,
                         toolbox.individual)
        toolbox.register("mate", tools.cxTwoPoint)
        toolbox.register("mutate",
                         tools.mutGaussian,
                         mu=0,
                         sigma=1,
                         indpb=0.05)
        toolbox.register("select", tools.selTournament, tournsize=3)

    def optimize(self, opt_dict: dict) -> Dict[str, Setpoint]:
        """
        """
        print(opt_dict)
        node_id = opt_dict["id"]
        cycle = opt_dict["cycle"]

        # Define a função de avaliação e as demais ferramentas
        self.prepare_metaheuristic(node_id)

        def evaluate(individual):
            desired = self.get_desired_opt_values(node_id,
                                                  cycle)
            indv_fit = np.array(individual)
            desired_fit = np.array(desired)
            errors = np.abs(indv_fit - desired_fit)
            return np.linalg.norm(errors),
        toolbox.register("evaluate", evaluate)

        # Parâmetro do algoritmo genético
        NPOP, CXPB, MUTPB, NGEN = 30, 0.5, 0.2, 50

        # Define a população e inicializa as fitness
        pop = toolbox.population(n=NPOP)
        fitnesses = list(map(toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit
        # Itera pelas gerações
        g = 0
        best_inds = []
        best_values = []
        while g < NGEN:
            g += 1
            offspring = toolbox.select(pop, len(pop))
            offspring = list(map(toolbox.clone, offspring))

            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < CXPB:
                    toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values

            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
                    del mutant.fitness.values

            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = list(map(toolbox.evaluate, invalid_ind))
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit

            # Guarda o melhor indivíduo da geração
            best_ind = tools.selBest(pop, 1)
            best_inds.append(best_ind[0])
            best_values.append(best_ind[0].fitness.values[0])
            pop[:] = offspring
        # Obtém o melhor indivíduo
        best_ind = best_inds[best_values.index(min(best_values))]
        # Limites superior e inferior de segurança
        for i, b in enumerate(best_ind):
            if b <= 0.2:
                best_ind[i] = 0.2
            if b >= 0.8:
                best_ind[i] = 0.8
        # Garante soma unitária
        total = sum(best_ind)
        for i in range(len(best_ind)):
            best_ind[i] /= total
        print(best_ind)
        self.setpoints[node_id].splits = best_ind
        return {node_id: self.setpoints[node_id]}
