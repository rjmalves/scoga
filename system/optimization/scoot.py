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
import random
import numpy as np
from statistics import mean
from queue import SimpleQueue
from typing import Dict, List
from deap import creator, base, tools
from multiprocessing import Process, Array
# Imports de módulos específicos da aplicação
from model.traffic.setpoint import Setpoint
from model.traffic.controller import Controller
from model.network.traffic_light import TrafficLight
from model.network.network import Network
from rich.console import Console
console = Console()


random.seed(42)
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
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
        # Variável que sinaliza a atividade ou não do otimizador
        self._now_optimizing = False
        # Desabilita ou não a otimização
        self._fixed_time = False

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
            console.print_exception()

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
        console.log("Otimizador inscrito em cycles!")
        # Toda thread que não seja a principal precisa ter o traceback printado
        try:
            # Faz a inscrição na fila.
            self.cycle_channel.basic_consume(queue=self.cycle_queue_name,
                                             on_message_callback=self.cycle_cb)
            # Começa a escutar. Como a conexão é bloqueante, trava aqui.
            self.cycle_channel.start_consuming()
        except Exception:
            console.print_exception()
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
        console.log("Otimizador iniciado!")
        # Variável para armazenar o ciclo a ser analisado durante a otimização
        self._opt_cycles: Dict[str, int] = {c_id: 0 for c_id
                                            in self.controllers.keys()}
        # Variável para armazenar se os controladores já enviaram seus pedidos
        # de otimização para a fila
        self._opt_queue: Dict[str, bool] = {c_id: False for c_id
                                            in self.controllers.keys()}
        while True:
            try:
                # Se existirem elementos na fila para otimizar
                if not self.optimization_queue.empty():
                    # Atualiza as variáveis de controle da otimização
                    opt_dict = self.optimization_queue.get()
                    self._opt_cycles[opt_dict["id"]] = opt_dict["cycle"]
                    self._opt_queue[opt_dict["id"]] = True
                    # Se todos já terminaram 1 ciclo, otimiza
                    print(self._opt_queue)
                    if all(list(self._opt_queue.values())):
                        self._now_optimizing = True
                        ciclo = list(self._opt_cycles.values())[0]
                        console.log(f"OTIMIZANDO CICLO {ciclo}")
                        if self._fixed_time:
                            solution = self.get_current_opt_values()
                        else:
                            desired_values = self.get_desired_opt_values()
                            console.log(f"DESIRED: {desired_values}")
                            best_ind = Array('d', range(len(desired_values)))
                            p = Process(target=optimize,
                                        args=(desired_values, best_ind))
                            p.start()
                            p.join()
                            solution = list(best_ind)
                            console.log(f"SOLUTION: {solution}")
                        # Salva os setpoints novos para cada controlador
                        accum_idx = 0
                        keys = sorted(list(self.controllers.keys()))
                        for i, b in enumerate(solution):
                            if b < 0.2:
                                solution[i] = 0.2
                            if b > 0.8:
                                solution[i] = 0.8
                        for c_id in keys:
                            ini_idx = accum_idx
                            ctrl = self.controllers[c_id]
                            fin_idx = ini_idx + ctrl.traffic_plan.stage_count
                            ctrl_values = solution[ini_idx:fin_idx]
                            # Garante soma unitária
                            total = sum(ctrl_values)
                            for i in range(len(ctrl_values)):
                                ctrl_values[i] /= total
                            self.setpoints[c_id].splits = ctrl_values
                            accum_idx = fin_idx
                        # Adiciona os setpoints à fila
                        for c_id, setp in self.setpoints.items():
                            self.setpoint_queue.put({c_id: setp})
                        # Limpa as flags de proposição de otimização
                        for c_id in self._opt_queue.keys():
                            self._opt_queue[c_id] = False
                    self._now_optimizing = False
                else:
                    # Senão
                    time.sleep(0.1)
            except Exception:
                console.print_exception()
                self.optimization_thread.join()

    def get_individual_shape(self) -> int:
        """
        Gera o formato da lista que representa um indivíduo na
        população da metaheurística.
        """
        stage_sums = sum([c.traffic_plan.stage_count
                          for c in self.controllers.values()])
        return stage_sums

    def get_current_opt_values(self) -> List[float]:
        """
        Extrai os valores dos parâmetros de otimização atualmente
        nos elementos da rede.
        """
        # Por enquanto trata somente de splits
        splits: List[float] = []
        # Ordena as keys
        keys = sorted(list(self.controllers.keys()))
        for c_id in keys:
            splits += self.setpoints[c_id].splits
        return splits

    def get_desired_opt_values_for_node(self,
                                        node_id: str,
                                        cycle: int) -> List[float]:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema para um nó específico.
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
            total_occ += max(stage_occ)
            stages_occs.append(max(stage_occ))
        for i in range(len(stages_occs)):
            if total_occ != 0:
                stages_occs[i] /= total_occ
        # Por enquanto trata somente de splits
        return stages_occs

    def get_desired_opt_values(self) -> List[float]:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema.
        """
        # Por enquanto trata somente de splits
        occs: List[float] = []
        # Ordena as keys
        keys = sorted(list(self.controllers.keys()))
        for c_id in keys:
            cycle = self._opt_cycles[c_id]
            occs += self.get_desired_opt_values_for_node(c_id, cycle)
        return occs

    @property
    def busy(self):
        return self._now_optimizing


def optimize(desired_values: List[float],
             best_ind: Array) -> List[float]:
    """
    Realiza a otimização através de G.A.
    """
    INDIV_SIZE = len(desired_values)
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

    def evaluate(individual):
        desired = desired_values
        indv_fit = np.array(individual)
        desired_fit = np.array(desired)
        errors = np.abs(indv_fit - desired_fit)
        return np.linalg.norm(errors),
    toolbox.register("evaluate", evaluate)

    # Parâmetro do algoritmo genético
    NPOP, CXPB, MUTPB, NGEN = 50, 0.5, 0.2, 50

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
        best_gen_ind = tools.selBest(pop, 1)
        best_inds.append(best_gen_ind[0])
        best_values.append(best_gen_ind[0].fitness.values[0])
        pop[:] = offspring
    # Obtém o melhor indivíduo
    b = best_inds[best_values.index(min(best_values))]
    for i in range(len(best_ind)):
        best_ind[i] = b[i]
