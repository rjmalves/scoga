# Definição do modelo de uma rede utilizada para simulação no SUMO.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 09 de Maio de 2020

# Imports gerais de módulos padrão
from multiprocessing.queues import Queue
from model.messages.semaphores import SemaphoresMessage
import sumolib  # type: ignore
import networkx as nx  # type: ignore
from typing import List, Tuple, Dict
import threading

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
        self.node_history_lock = threading.Lock()
        self.edge_history_lock = threading.Lock()
        self.lane_history_lock = threading.Lock()

    def update_node_history(self,
                            message: SemaphoresMessage,
                            queue: Queue):
        with self.node_history_lock:
            self.nodes[message.controller_id].history.update(message,
                                                             queue)

    def update_edge_traffic_data(self,
                                 edge_id: str,
                                 time: float,
                                 avg_speed: float,
                                 veh_count: int,
                                 avg_occ: float):
        with self.edge_history_lock:
            self.edges[edge_id].history.update_traffic_data(time,
                                                            avg_speed,
                                                            veh_count,
                                                            avg_occ)

    def update_lane_traffic_data(self,
                                 edge_id: str,
                                 lane_id: str,
                                 time: float,
                                 avg_speed: float,
                                 veh_count: int,
                                 avg_occ: float):
        with self.lane_history_lock:
            edge = self.edges[edge_id]
            edge.lanes[lane_id].history.update_traffic_data(time,
                                                            avg_speed,
                                                            veh_count,
                                                            avg_occ)

    def end(self):
        """
        """
        # Termina a comunicação nos históricos de nó
        for _, n in self.nodes.items():
            if n.controlled:
                n.history.end()

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
