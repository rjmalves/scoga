# Definição do histórico de operação dos semáforos em um nó da rede.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 05 de Maio de 2020

# Imports gerais de módulos padrão
from typing import Dict, List, Tuple
# Imports de módulos específicos da aplicação
from model.traffic.traffic_plan import TrafficPlan
from model.network.traffic_light import TLState


class NodeHistoryEntry:
    """
    """
    def __init__(self,
                 time_instant: float,
                 stage_idx: int,
                 interval_idx: int):
        self.time_instant = time_instant
        self.stage_idx = stage_idx
        self.interval_idx = interval_idx


class NodeHistory:
    """
    Classe de histórico responsável por armazenar informações sobre uma
    interseção da rede do SUMO. Permite que a ordem dos estágios do plano
    seja alterada, mas necessita que os estágios sejam os mesmos ao longo
    de toda a sua execução.
    """
    def __init__(self,
                 node_id: str,
                 tl_ids: List[str],
                 traffic_plan: TrafficPlan,
                 current_time: float):
        # Parâmetros gerais associados ao plano do qual é realizado o ciclo
        self.node_id = node_id
        # Os grupos semafóricos nunca mudam
        self.tl_ids = tl_ids
        # Os estágios nunca mudam (podem até mudar de ordem ou serem omitidos)
        self.traffic_plan = traffic_plan
        self.current_time = current_time
        self.history: List[NodeHistoryEntry] = []
        # Infere o estado atual dos semáforos baseado no instante de tempo
        tl_state_list = self.traffic_plan.current_tl_states(self.current_time)
        self.tl_states: Dict[str, TLState] = {}
        for tl_id, tl_state in zip(self.tl_ids, tl_state_list):
            self.tl_states[tl_id] = tl_state
        # Cria o primeiro objeto do history
        stage, interval = self.__infer_stage_and_interval_from_tls()
        self.current_stage = stage
        self.current_interval = interval
        self.history.append(NodeHistoryEntry(self.current_time,
                                             stage,
                                             interval))

    def __tl_states_as_list(self) -> List[TLState]:
        """
        Função que parte do dicionário que associa semáforo a estado
        e retorna uma lista de estados, seguindo a mesma ordem do plano.
        """
        tl_state_list: List[TLState] = []
        for tl_id in self.tl_ids:
            tl_state_list.append(self.tl_states[tl_id])
        return tl_state_list

    def __infer_stage_and_interval_from_tls(self) -> Tuple[int, int]:
        """
        Função responsável por inferir em que estágio e intervalo o controlador
        de uma interseção se encontra a partir do estado do seus semáforos.
        Deve ser chamada sempre que houver uma mudança em estado de semáforo.
        """
        # Obtém o estágio atual a partir do plano:
        stage_idx = self.traffic_plan.current_plan_stage(self.current_time)
        stage = self.traffic_plan.stages[stage_idx]
        # Descobre o intervalo dentro do estágio atual.
        for interval_idx, interval in enumerate(stage.intervals):
            if interval.states == self.__tl_states_as_list():
                self.current_stage = stage_idx
                self.current_interval = interval_idx
                return stage_idx, interval_idx
        # Se não encontrar, retorna (0, 0) por padrão
        return 0, 0

    def update(self, tl_id: str, state: TLState, time_instant: float):
        """
        Função para atualizar o estado dos semáforos da interseção armazenados
        e adicionar um novo objeto histórico à lista de históricos.
        """
        # Atualiza o objeto de estados de semáforos local se algo tiver mudado
        if state == self.tl_states[tl_id]:
            return
        self.current_time = time_instant
        self.tl_states[tl_id] = state
        # Infere o estágio e o intervalo
        stage, interval = self.__infer_stage_and_interval_from_tls()
        # Adiciona um novo objeto de histórico
        self.history.append(NodeHistoryEntry(self.current_time,
                                             stage,
                                             interval))

    def export(self) -> List[Tuple[float, int, int]]:
        """
        Função para exportar o histórico de um nó do SUMO.
        """
        first = (self.history[0].time_instant,
                 self.history[0].stage_idx,
                 self.history[0].interval_idx)
        node_history = [first]
        for i in range(1, len(self.history)):
            previous = (self.history[i].time_instant - 0.01,
                        self.history[i - 1].stage_idx,
                        self.history[i - 1].interval_idx)
            current = (self.history[i].time_instant,
                       self.history[i].stage_idx,
                       self.history[i].interval_idx)
            node_history += [previous, current]

        return node_history
