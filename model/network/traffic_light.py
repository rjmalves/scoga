# Classe de semáforo para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from enum import Enum
from typing import List, Tuple
from pandas import DataFrame  # type: ignore
from numpy import arange  # type: ignore
# Imports de módulos específicos da aplicação


class TLState(Enum):
    RED = 0
    AMBER = 1
    GREEN = 2

    def __str__(self):
        return str(self.value)

    @classmethod
    def from_json(cls, json_data: str):
        for key, val in cls.__dict__.items():
            if json_data == key:
                return cls(val)
        # Se não encontrou nenhum compatível, retorna vermelho
        return cls.RED

    def map_to_sumo_char(self) -> str:
        """
        Dependendo do valor do enum, retorna o caractere que deve ser colocado
        na string que o SUMO utiliza para comandar o semáforo.
        """
        sumo_char = "r"  # O default é o vermelho
        if self.value == TLState.RED.value:
            sumo_char = "r"
        elif self.value == TLState.AMBER.value:
            sumo_char = "y"
        elif self.value == TLState.GREEN.value:
            sumo_char = "G"
        return sumo_char


class TrafficLight:
    """
    Classe que representa um semáforo no SUMO e guarda o mapeamento de como o
    semáforo é visto dentro do controlador para como deve ser alterado via
    TraCI.
    """
    def __init__(self, intersection_id: str, group_idxs: List[int]):
        # A partir do ID da interseção e do índices que tratam do grupo, é
        # gerado o ID que este semáforo deve ter para os controladores.
        self.intersection_id = intersection_id
        self.group_idxs = group_idxs
        self.id_in_controller = "{}-{}".format(self.intersection_id,
                                               self.group_idxs[0])
        self.state = TLState.RED  # Estado inicial sempre vermelho
        self.state_history: List[Tuple[float, TLState]] = []

    def __str__(self):
        trafficlight_str = ""
        for key, val in self.__dict__.items():
            trafficlight_str += "        {}: {}\n".format(key, val)
        return trafficlight_str

    def update_intersection_string(self, current_string: str) -> str:
        """
        Recebe uma string da interseção com os estado sendo executado
        atualmente e atualiza os caracteres que são de responsabilidade do
        objeto.
        """
        new_string = list(current_string)
        for idx in self.group_idxs:
            new_string[idx] = self.state.map_to_sumo_char()
        return "".join(new_string)

    def update_state(self, new_state: TLState, time_instant: float):
        """
        Função para atualizar o estado de um grupo semafórico, junto com o
        histórico de estados que este já assumiu.
        """
        self.state_history.append((time_instant, new_state))
        self.state = new_state

    def export_state_history(self, last_sim_t: float) -> DataFrame:
        """
        Função para exportar o histórico de estados do grupo semafórico. No
        momento da exportação, os dados de semáforos, que são internamente
        apenas os dados das transições, são amostrados com um período de 0.1s.
        """
        # Intervalos de tempo de geração do histórico
        first_t = float(self.state_history[0][0])
        # Variáveis de interesse:
        sampling_times: List[float] = []
        states: List[str] = []
        # Faz a amostragem de .1 em .1 segundo durante o tempo de funcionamento
        current = 0
        n_samples = len(self.state_history)
        for t in arange(first_t, last_sim_t, 0.1):
            sampling_times.append(t)
            if self.state_history[current][1] == TLState.RED:
                states.append('RED')
            elif self.state_history[current][1] == TLState.AMBER:
                states.append('AMBER')
            elif self.state_history[current][1] == TLState.GREEN:
                states.append('GREEN')
            next_sample = min([current + 1, n_samples - 1])
            if t >= self.state_history[next_sample][0]:
                current = next_sample

        history_df = DataFrame()
        history_df['sampling_time'] = sampling_times
        history_df['state'] = states
        history_df['tl_id'] = self.id_in_controller

        return history_df


if __name__ == "__main__":
    # Cria dois semáforos em uma interseção
    tl1 = TrafficLight("0", [0, 2])
    tl2 = TrafficLight("0", [1, 3])
    # Printa para conferir
    print(tl1)
    print(tl2)
    # Cria uma string de estado conhecida
    sumo_string = "GrGr"
    print("Antes: ", sumo_string)
    # Simula o semáforo 1 atualizando
    sumo_string = tl1.update_intersection_string(sumo_string)
    # Troca o semáforo 2 para verde e atualiza também
    tl2.state = TLState.GREEN
    sumo_string = tl2.update_intersection_string(sumo_string)
    # Printa o estado novo da interseção
    print("Depois: ", sumo_string)
