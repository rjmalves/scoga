# Classe de semáforo para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from enum import Enum
from typing import List
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
        if self.value == TLState.RED:
            sumo_char = "r"
        elif self.value == TLState.AMBER:
            sumo_char = "y"
        elif self.value == TLState.GREEN:
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
        new_string = current_string
        for idx in self.group_idxs:
            new_string[idx] = self.state.map_to_sumo_char()
        return new_string
