# Definição do modelo de um nó na rede para simulação no SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 09 de Maio de 2020

# Imports gerais de módulos padrão
import sumolib  # type: ignore


class Node:
    """
    """
    def __init__(self, node_id: str):
        self.id = node_id

    @classmethod
    def from_sumolib_node(cls, sumo_node: sumolib.net.node.Node):
        """
        Converte um objeto obtido através da sumolib em um Node utilizado
        internamente para processamento.
        """
        node_id = sumo_node.getID()
        return cls(node_id)
