# Definição do histórico de ocupação de uma faixa, em uma via na rede.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 10 de Maio de 2020

# Imports gerais de módulos padrão
from typing import Dict
# import traci
# Imports de módulos específicos da aplicação
from model.network.detector import Detector


class LaneHistory:
    """
    Classe de histórico responsável por armazenar informações sobre uma
    faixa em uma via da rede do SUMO. Contém as informações de detecção e
    dos veículos que trafegaram pela faixa ao longo da simulação e é usada
    no processo de reconstrução e comparação com o estado real da faixa
    a partir de uma aproximação pelos dados de detecção.
    """
    def __init__(self, lane_id: str, detectors: Dict[str, Detector]):
        self.id = lane_id
        self.detectors = detectors
        # LEMBRAR QUE EU POSSO USAR A TRACI DAQUI
