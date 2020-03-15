# Classe de semáforo para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from enum import Enum
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


class TrafficLight:
    """
    """
