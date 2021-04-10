# Definição do histórico de operação dos semáforos em um nó da rede.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 05 de Maio de 2020

# Imports gerais de módulos padrão
from model.messages.cycle import CycleMessage
from model.messages.semaphores import SemaphoresMessage
import pika  # type: ignore
from PikaBus.PikaBusSetup import PikaBusSetup
from typing import Dict, List, Tuple
from pandas import DataFrame  # type: ignore
from numpy import arange  # type: ignore
# Imports de módulos específicos da aplicação
from model.traffic.traffic_plan import TrafficPlan
from model.network.traffic_light import TLState
from rich.console import Console
console = Console()


class NodeHistoryEntry:
    """
    """
    def __init__(self,
                 time_instant: float,
                 cycle_idx: int,
                 stage_idx: int,
                 interval_idx: int):
        self.cycle_idx = cycle_idx
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
        self.current_time = round(current_time, 1)
        self.current_cycle = 0
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
                                             self.current_cycle,
                                             stage,
                                             interval))
        # Define os parâmetros da conexão (local do broker RabbitMQ)
        self.parameters = pika.ConnectionParameters(host="localhost")
        self.init_cycle_connection()

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

    def init_cycle_connection(self):
        """
        Declara a exchange para pegar o tick do relógio e a relaciona com a
        fila exclusiva de relógio.
        """
        q_name = f'node_hist_{self.node_id}_cycle_clk_queue'
        self._cycle_pika_bus = PikaBusSetup(self.parameters,
                                            defaultListenerQueue=q_name)
        self._cycle_pika_bus.StartConsumers()
        self.cycle_bus = self._cycle_pika_bus.CreateBus()

    def update(self, message: SemaphoresMessage):
        """
        Função para atualizar o estado dos semáforos da interseção armazenados
        e adicionar um novo objeto histórico à lista de históricos.
        """
        # Backup do estágio atual
        if self.current_cycle != message.new_cycle:
            self.current_cycle = message.new_cycle
            # Se incrementou, publica no tópico de otimização
            cycle_message = CycleMessage(self.node_id,
                                         self.current_cycle)
            console.log(f"Node {self.node_id}: {cycle_message}")
            self.cycle_bus.Publish(payload=cycle_message.to_dict(),
                                   topic="cycles")
        # Adiciona um novo objeto de histórico
        self.current_time = message.current_time
        self.history.append(NodeHistoryEntry(self.current_time,
                                             self.current_cycle,
                                             message.new_stage,
                                             message.new_interval))

    def get_cycle_time_boundaries(self, cycle: int) -> Tuple[float, float]:
        """
        Retorna o instante de tempo de início e final de um ciclo executado.
        """
        if cycle > self.current_cycle:
            return (0.0, 0.0)
        else:
            # Extrai os históricos associados ao ciclo
            hists: List[NodeHistoryEntry] = []
            for i, h in enumerate(self.history):
                if h.cycle_idx == cycle - 1:
                    hists.append(h)
                # Adiciona o último após o ciclo para pegar o t_ini
                if h.cycle_idx == cycle:
                    hists.append(h)
                    break
            init_time = hists[0].time_instant
            end_time = hists[-1].time_instant
            return (init_time, end_time)

    def export(self, last_sim_t: float) -> DataFrame:
        """
        Função para exportar o histórico de um nó do SUMO. No momento da
        exportação, os dados de interseções, que são internamente
        apenas os dados das transições, são amostrados com um período de 0.1s.
        """
        # Intervalos de tempo de geração do histórico
        first_t = self.history[0].time_instant
        # Variáveis de interesse:
        sampling_times: List[float] = []
        cycles: List[int] = []
        stages: List[int] = []
        intervals: List[int] = []
        # Faz a amostragem de .1 em .1 segundo durante o tempo de funcionamento
        current = 0
        n_samples = len(self.history)
        for t in arange(first_t, last_sim_t, 0.1):
            sampling_times.append(t)
            cycles.append(self.history[current].cycle_idx)
            stages.append(self.history[current].stage_idx)
            intervals.append(self.history[current].interval_idx)
            # Verifica o índice do elemento que será amostrado em seguida
            next_sample = min([current + 1, n_samples - 1])
            if t >= self.history[next_sample].time_instant:
                current = next_sample

        history_df = DataFrame()
        history_df['sampling_time'] = sampling_times
        history_df['cycle'] = cycles
        history_df['stage'] = stages
        history_df['interval'] = intervals
        history_df['node_id'] = self.node_id

        return history_df

    def stop_communication(self):
        """
        """
        console.log(f"Terminando a comunicação no histórico do nó {self.node_id}")
        self._cycle_pika_bus.StopConsumers()
        self._cycle_pika_bus.Stop()

    def __del__(self):
        """
        Como esta classe instancia threads, deve ter os .join() explícitos no
        destrutor.
        """
        self._cycle_pika_bus.StopConsumers()
        self._cycle_pika_bus.Stop()
