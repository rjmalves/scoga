# Definição do modelo de uma rede utilizada para simulação no SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 09 de Maio de 2020

# Imports gerais de módulos padrão
import sumolib  # type: ignore
import networkx as nx  # type: ignore
from typing import List, Tuple, Dict

# Imports de módulos específicos da aplicação
from model.network.node import Node
from model.network.edge import Edge


class Network:
    """
    Modela a topologia de uma rede utilizada para simulação de tráfego no SUMO.
    """

    def __init__(self,
                 topology: nx.DiGraph,
                 nodes: Dict[str, Node],
                 edges: Dict[str, Edge]):
        self.topology = topology
        self.nodes = nodes
        self.edges = edges

    @classmethod
    def from_sumolib_net(cls, net: sumolib.net.Net):
        """
        Converte um objeto obtido através da sumolib em uma rede utilizada
        internamente para processamento.
        """
        topology_nodes: List[sumolib.net.node.Node] = net.getNodes()
        topology_edges: List[sumolib.net.edge.Edge] = net.getEdges()
        # Constroi as arestas como Tuple[int, int] - origem, destino
        # e obtém as Lanes
        nx_edges: List[Tuple[str, str]] = []
        net_nodes: Dict[str, Node] = {}
        for n in topology_nodes:
            net_nodes[n.getID()] = Node.from_sumolib_node(n)
        net_edges: Dict[str, Edge] = {}
        for e in topology_edges:
            net_edges[e.getID()] = Edge.from_sumolib_edge(e)
            # Obtém os IDs dos nós do SUMO
            from_node: str = e.getFromNode().getID()
            to_node: str = e.getToNode().getID()
            # Adiciona os IDs à lista para criar a topologia
            nx_edges.append((from_node, to_node))
        # Constroi a topologia da rede
        topology = nx.DiGraph()
        topology.add_edges_from(nx_edges)

        return cls(topology, net_nodes, net_edges)
