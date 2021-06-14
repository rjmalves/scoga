# Classe de detector para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 18 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
from pandas import DataFrame  # type: ignore
from numpy import arange  # type: ignore
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
                 edge_id: str,
                 lane_id: str,
                 position: float):
        self.id = detector_id
        self.edge_id = edge_id
        self.lane_id = lane_id
        self.position = position
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

    def update_detection_history(self, time_instant: float, state: bool):
        """
        Adiciona uma detecção ao detector para formar o histórico.
        """
        self.state = state
        self.detection_history.append(Detection(time_instant, state))

    def export_detection_history(self, last_sim_t: float) -> DataFrame:
        """
        Exporta o histórico de detecção na forma de uma lista onde são
        mostrados os instantes de mudança de estado dos detectores.
        No momento da exportação, os dados de detecção, que são internamente
        apenas os dados das transições, são amostrados com um período de 0.1s.
        """
        # Intervalos de tempo de geração do histórico
        first_t = self.detection_history[0].time_instant
        # Variáveis de interesse:
        sampling_times: List[float] = []
        states: List[int] = []
        # Faz a amostragem de .1 em .1 segundo durante o tempo de funcionamento
        current = 0
        n_samples = len(self.detection_history)
        for t in arange(first_t, last_sim_t, 0.1):
            sampling_times.append(t)
            states.append(int(self.detection_history[current].state))
            # Verifica o índice do elemento que será amostrado em seguida
            next_sample = min([current + 1, n_samples - 1])
            if t >= self.detection_history[next_sample].time_instant:
                current = next_sample

        history_df = DataFrame()
        history_df['sampling_time'] = sampling_times
        history_df['state'] = states
        history_df['det_id'] = self.id

        return history_df
