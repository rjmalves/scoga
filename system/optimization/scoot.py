# Otimizador de tráfego para realizar o cálculo dos novos parâmetros de
# controle de cada controlador em uma rede do SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 14 de Maio de 2020

# Imports gerais de módulos padrão
from model.traffic.traffic_plan import TrafficPlan
from model.messages.cycle import CycleMessage
import time
from enum import Enum
import threading
import random
import numpy as np
from queue import SimpleQueue
from typing import Dict, List
from deap import creator, base, tools
from multiprocessing import Process, Array
# Imports de módulos específicos da aplicação
from model.traffic.setpoint import Setpoint
from model.network.traffic_light import TrafficLight
from model.network.network import Network
from rich.console import Console
console = Console()


creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)
toolbox = base.Toolbox()
toolbox.register("attr_float", random.random)


class EnumOptimizationMethods(Enum):
    """
    """
    FixedTime = 0
    Split = 1
    Cycle = 2
    Offset = 3
    SplitCycle = 4
    SplitOffset = 5
    CycleOffset = 6
    SplitCycleOffset = 7
    ITLC = 8


class ScootOptimizer:
    """
    Responsável pela realização da otimização baseada em ciclos, a
    SCOOT. É instanciado dentro do TrafficControler e se inscreve
    para receber notificações sobre avanço de ciclos de cada nó
    controlador.
    """
    def __init__(self,
                 network: Network,
                 plans: Dict[str, TrafficPlan],
                 setpoints: Dict[str, Setpoint],
                 traffic_lights: Dict[str, TrafficLight],
                 opt_method: EnumOptimizationMethods):
        self.should_exit = False
        # Obtém uma referência para a rede, com históricos.
        self.network = network
        self.plans = plans
        self.setpoints = setpoints
        self.traffic_lights = traffic_lights
        # Cria a thread que realiza a otimização
        self.optimization_thread = threading.Thread(target=self.optimizing,
                                                    daemon=True,
                                                    name="Optimization")
        # Cria a fila que recebe as otimizações a serem realizadas
        self.optimization_queue: SimpleQueue = SimpleQueue()
        # Cria a fila que recebe os novos setpoints para serem
        # obtidos pelo controller
        self.setpoint_queue: SimpleQueue = SimpleQueue()
        # Variável que sinaliza a atividade ou não do otimizador
        self._now_optimizing = False
        # Tipo de algoritmo de otimização usado
        self._opt_method = opt_method
        # Instante de tempo atual da simulação
        self.simulation_time = 0.

    def end(self):
        """
        """
        self.should_exit = True
        console.log("Terminando a comunicação no otimizador")
        self.optimization_thread.join()

    def __del__(self):
        """
        """
        self.optimization_thread.join()

    def start(self, setpoints: Dict[str, Setpoint]):
        """
        """
        try:
            # Armazena os setpoints
            self.setpoints = setpoints
            # Inicia a thread de otimização
            self.optimization_thread.start()
        except Exception:
            console.print_exception()

    def cycle_cb(self, mess: CycleMessage):
        """
        """
        # Coloca o elemento na fila de otimizações
        self.optimization_queue.put(mess)

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
                                            in self.plans.keys()}
        # Variável para armazenar se os controladores já enviaram seus pedidos
        # de otimização para a fila
        self._opt_queue: Dict[str, bool] = {c_id: False for c_id
                                            in self.plans.keys()}
        while not self.should_exit:
            try:
                # Se existirem elementos na fila para otimizar
                if not self.optimization_queue.empty():
                    # Atualiza as variáveis de controle da otimização
                    message: CycleMessage = self.optimization_queue.get()
                    self._opt_cycles[message.node_id] = message.cycle
                    self._opt_queue[message.node_id] = True
                    # Se todos já terminaram 1 ciclo, otimiza
                    if all(list(self._opt_queue.values())):
                        self._now_optimizing = True
                        ciclo = list(self._opt_cycles.values())[0]
                        if (ciclo == 1 or self._opt_method ==
                                EnumOptimizationMethods.FixedTime):
                            split_sol = self.get_current_split_opt_values()
                            cycle_sol = self.get_current_cycle_opt_values()
                            # offset_sol = []
                        elif (self._opt_method ==
                              EnumOptimizationMethods.Split):
                            desired_val = self.get_desired_split_opt_values()
                            best_ind = Array('d', range(len(desired_val)))
                            p = Process(target=split_optimize,
                                        args=(desired_val, best_ind))
                            p.start()
                            p.join()
                            split_sol = list(best_ind)
                            cycle_sol = self.get_current_cycle_opt_values()
                            # offset_sol = []
                        elif (self._opt_method ==
                              EnumOptimizationMethods.Cycle):
                            desired_val = self.get_desired_cycle_opt_values()
                            split_sol = self.get_current_split_opt_values()
                            cycle_sol = desired_val
                            # offset_sol = []
                        elif (self._opt_method ==
                              EnumOptimizationMethods.Offset):
                            desired_val = self.get_desired_offset_opt_values()
                            best_ind = Array('d', range(len(desired_val)))
                            p = Process(target=offset_optimize,
                                        args=(desired_val, best_ind))
                            p.start()
                            p.join()
                            split_sol = self.get_current_split_opt_values()
                            cycle_sol = self.get_current_cycle_opt_values()
                            # offset_sol = list(best_ind)
                        elif (self._opt_method ==
                              EnumOptimizationMethods.SplitCycle):
                            desired_val = self.get_desired_split_opt_values()
                            best_ind = Array('d', range(len(desired_val)))
                            p = Process(target=split_optimize,
                                        args=(desired_val, best_ind))
                            p.start()
                            p.join()
                            split_sol = list(best_ind)
                            cycle_sol = self.get_desired_cycle_opt_values()
                            # offset_sol = []
                        # Salva os setpoints novos para cada controlador
                        keys = sorted(list(self.plans.keys()))
                        # -- SPLITS --
                        # Garante a transição "suave" - não muda mais
                        # que 5% em relação ao atual
                        c = self.get_current_split_opt_values()
                        for i, b in enumerate(split_sol):
                            max_split_stg = c[i] + 0.05
                            min_split_stg = c[i] - 0.05
                            ub = min([max_split_stg, b])
                            lb = max([min_split_stg, ub])
                            split_sol[i] = lb
                        for i, b in enumerate(split_sol):
                            if b < 0.2:
                                split_sol[i] = 0.2
                            if b > 0.8:
                                split_sol[i] = 0.8
                        accum_idx = 0
                        for c_id in keys:
                            ini_idx = accum_idx
                            fin_idx = ini_idx + self.plans[c_id].stage_count
                            ctrl_values = split_sol[ini_idx:fin_idx]
                            # Garante soma unitária
                            total = sum(ctrl_values)
                            for i in range(len(ctrl_values)):
                                ctrl_values[i] /= total
                            self.setpoints[c_id].splits = ctrl_values
                            accum_idx = fin_idx
                        # -- CICLOS --
                        for c_id in keys:
                            # Faz a correção de offset para cada controlador
                            # Guarda o ciclo e offset anteriores
                            old_cycle = int(self.setpoints[c_id].cycle)
                            old_offset = int(self.setpoints[c_id].offset)
                            new_cycle = int(cycle_sol)
                            # Calcula o novo offset, para manter o ciclo
                            new_offset = self.offset_correction(old_cycle,
                                                                new_cycle,
                                                                old_offset)
                            # Atribui os novos valores
                            self.setpoints[c_id].cycle = new_cycle
                            self.setpoints[c_id].offset = int(new_offset)

                        # Adiciona os setpoints à fila
                        for c_id, setp in self.setpoints.items():
                            self.setpoint_queue.put({c_id: setp})
                        # Limpa as flags de proposição de otimização
                        for c_id in self._opt_queue.keys():
                            self._opt_queue[c_id] = False
                    self._now_optimizing = False
                else:
                    # Senão
                    time.sleep(1e-6)
            except Exception:
                console.print_exception()
                self.optimization_thread.join()

    def offset_correction(self,
                          old_cycle: int,
                          new_cycle: int,
                          old_offset: int) -> int:
        """
        """
        # Obtém o instante de tempo atual
        t = int(round(self.simulation_time))
        # Obtém o instante no ciclo atual
        cycle_time = (t - old_offset) % old_cycle
        # Calcula o novo offset
        new_offset = (t - cycle_time) % new_cycle
        return new_offset

    def get_individual_shape(self) -> int:
        """
        Gera o formato da lista que representa um indivíduo na
        população da metaheurística.
        """
        stage_sums = sum([p.stage_count
                          for p in self.plans.values()])
        return stage_sums

    def get_current_split_opt_values(self) -> List[float]:
        """
        Extrai os valores dos parâmetros de otimização atualmente
        nos elementos da rede.
        """
        # Por enquanto trata somente de splits
        splits: List[float] = []
        # Ordena as keys
        keys = sorted(list(self.plans.keys()))
        for c_id in keys:
            splits += self.setpoints[c_id].splits
        return splits

    def get_current_cycle_opt_values(self) -> List[float]:
        """
        Extrai os valores dos parâmetros de otimização atualmente
        nos elementos da rede.
        """
        # Ordena as keys
        keys = sorted(list(self.plans.keys()))
        return self.setpoints[keys[0]].cycle

    def get_desired_split_opt_values_for_node(self,
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
        for tl_id in [f"{node_id}-{i}" for i in range(2)]:
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

        return stages_occs

    def get_max_occupation_in_cycle_for_node(self,
                                             node_id: str,
                                             cycle: int) -> float:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema para um nó específico.
        """
        n = self.network
        # Verifica o histórico mais recente para obter os splits
        # desejados        total_occ = 0.
        stages_occs: List[float] = []
        node_hist = self.network.nodes[node_id].history
        ti, tf = node_hist.get_cycle_time_boundaries(cycle)
        # Obtém os IDs dos semáforos de cada estágio
        stages_lanes: List[List[str]] = []
        for tl_id in [f"{node_id}-{i}" for i in range(2)]:
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
                data = lane.history.get_max_traffic_data_in_time(ti, tf)
                stage_occ.append(data["occupancy"])
            stages_occs.append(max(stage_occ))

        return max(stages_occs)

    def get_desired_split_opt_values(self) -> List[float]:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema.
        """
        # Por enquanto trata somente de splits
        occs: List[float] = []
        # Ordena as keys
        keys = sorted(list(self.plans.keys()))
        for c_id in keys:
            cycle = self._opt_cycles[c_id]
            occs += self.get_desired_split_opt_values_for_node(c_id, cycle)
        return occs

    def get_desired_cycle_opt_values(self) -> List[float]:
        """
        Obtém os valores desejados dos parâmetros de cada
        variável do problema.
        """
        # Por enquanto trata somente de ciclo
        occs: List[float] = []
        # Ordena as keys
        keys = sorted(list(self.plans.keys()))
        current_cycle = self.setpoints[keys[0]].cycle
        for c_id in keys:
            cycle = self._opt_cycles[c_id]
            occs.append(self.get_max_occupation_in_cycle_for_node(c_id,
                                                                  cycle))
        # Regra do ciclo: se max(occs) > 50%, aumenta. Se < 20%, diminui.
        # A quantidade de variação depende da distância ao parâmetro.
        # Ciclo mínimo = 30s
        # Ciclo máximo = 120s
        MIN_CYCLE = 30
        MAX_CYCLE = 120
        LOWER_REF = 0.3
        UPPER_REF = 0.6
        # Aplica a regra do ciclo
        desired_cycle = current_cycle
        delta = 0
        if max(occs) < LOWER_REF:
            delta = max([int(round(10 * (LOWER_REF - max(occs)))), 1])
            new_cycle = current_cycle - delta
            desired_cycle = max([new_cycle, MIN_CYCLE])
        elif max(occs) > UPPER_REF:
            delta = max([int(round(10 * (max(occs) - UPPER_REF))), 1])
            new_cycle = current_cycle + delta
            desired_cycle = min([new_cycle, MAX_CYCLE])
        return desired_cycle

    @property
    def busy(self):
        return self._now_optimizing


def split_optimize(desired_values: List[float],
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
    NPOP, CXPB, MUTPB, NGEN = 20, 0.5, 0.2, 100

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


def offset_optimize(desired_values: List[float],
                    best_ind: Array) -> List[float]:
    pass
