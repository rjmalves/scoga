# Classe de detector para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 18 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
# Imports de módulos específicos da aplicação


class Detection:
    """
    Classe que representa as detecções de um detector. Contém um instante de
    tempo no qual ocorreu a modificação e o estado para o qual o detector foi.
    """
    def __init__(self, time_instant: float, state: bool):
        self.time_instant = time_instant
        self.state = state

    def __str__(self):
        detection_str = ""
        for key, val in self.__dict__.items():
            detection_str += "        {}: {}\n".format(key, val)
        return detection_str


class Detector:
    """
    Classe que representa o objeto detector indutivo localizado em uma Lane
    do SUMO.
    """
    def __init__(self,
                 detector_id: str,
                 controller_id: str):
        self.id = detector_id
        self.controller_ids = set(controller_id)
        self.detection_history: List[Detection] = []
        self.state = False
        # Adiciona, por default, um histórico inicial
        self.update_detection_history(0.0, False)

    def __str__(self):
        detector_str = ""
        for key, val in self.__dict__.items():
            if key == "detection_history":
                pass
            else:
                detector_str += "{}: {}\n".format(key, val)
        return detector_str

    def add_sim_info(self, lane_id: str, position: float):
        """
        Adiciona os atributos relevantes do detector na simulação.
        """
        self.lane_id = lane_id
        self.position = position

    def add_controller(self, controller_id: str):
        """
        Adiciona um controlador interessado em dados do detector.
        """
        self.controller_ids.add(controller_id)

    def update_detection_history(self, time_instant: float, state: bool):
        """
        Adiciona uma detecção ao detector para formar o histórico.
        """
        self.state = state
        self.detection_history.append(Detection(time_instant, state))


if __name__ == "__main__":
    # Cria um objeto detector para teste e printa
    test_det = Detector("MY_ID", "1")
    print(test_det)
    test_det.add_sim_info("LANE_ID", 25.96)
    test_det.add_controller("2")
    test_det.update_detection_history(1.0, True)
    print(test_det)