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
    YELLOW = 1
    GREEN = 2

    def __str__(self):
        return str(self.value)


class TrafficLight:
    """
    """
