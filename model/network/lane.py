# Definição do modelo de uma faixa na rede para simulação no SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 09 de Maio de 2020

# Imports gerais de módulos padrão
import sumolib  # type: ignore


class Lane:
    """
    """
    def __init__(self, lane_id: str):
        self.id = lane_id

    @classmethod
    def from_sumolib_lane(cls, sumo_lane: sumolib.net.lane.Lane):
        """
        Converte um objeto obtido através da sumolib em uma Lane utilizado
        internamente para processamento.
        """
        lane_id = sumo_lane.getID()
        return cls(lane_id)
