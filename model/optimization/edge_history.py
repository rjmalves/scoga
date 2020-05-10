# Definição do histórico de ocupação de uma via na rede.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 10 de Maio de 2020

# Imports gerais de módulos padrão
# import traci


class EdgeHistory:
    """
    Esta classe é fundamentalmente diferente do NodeHistory e LaneHistory. Ela
    não armazena informações, apenas processa os históricos de Lanes de maneira
    adequada quando é solicitada.
    """
    def __init__(self, edge_id: str):
        # LEMBRAR QUE EU POSSO USAR A TRACI DAQUI
        pass
