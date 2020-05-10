# Definição do modelo de uma via na rede para simulação no SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 09 de Maio de 2020

# Imports gerais de módulos padrão
import sumolib  # type: ignore
from typing import Dict

# Imports específicos da aplicação
from model.network.lane import Lane
from model.optimization.lane_history import LaneHistory
from model.optimization.edge_history import EdgeHistory


class Edge:
    """
    """
    def __init__(self, edge_id: str, lanes: Dict[str, Lane]):
        self.id = edge_id
        self.lanes = lanes

    @classmethod
    def from_sumolib_edge(cls, sumo_edge: sumolib.net.edge.Edge):
        """
        Converte um objeto obtido através da sumolib em uma Edge utilizado
        internamente para processamento.
        """
        edge_id = sumo_edge.getID()
        lanes: Dict[str, Lane] = {}
        for lane in sumo_edge.getLanes():
            lanes[lane.getID()] = Lane.from_sumolib_lane(lane)
        return cls(edge_id, lanes)

    def add_history(self, hist: LaneHistory):
        """
        """
        self.lanes[hist.id].add_history(hist)
