# Definição do histórico de ocupação de uma faixa, em uma via na rede.
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 10 de Maio de 2020

# Imports gerais de módulos padrão
from typing import Dict, List
from pandas import DataFrame  # type: ignore
from statistics import mean
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
    def __init__(self, lane_id: str):
        self.id = lane_id
        self.sampling_time: List[float] = []
        # Variáveis relacionadas a medidas de tráfego
        self.travel_time: List[float] = []  # Medido em s
        self.waiting_time: List[float] = []  # Medido em s
        self.average_occupancy: List[float] = []  # Medido em %
        self.vehicle_count: List[int] = []  # Quantidade absoluta
        self.halting_vehicle_count: List[int] = []  # Quantidade absoluta
        self.average_speed: List[float] = []  # Medido em m/s
        # Variáveis relacionadas ao impacto ambiental
        self.CO2_emission: List[float] = []  # Medido em mg
        self.CO_emission: List[float] = []  # Medido em mg
        self.HC_emission: List[float] = []  # Medido em mg
        self.NOx_emission: List[float] = []  # Medido em mg
        self.PMx_emission: List[float] = []  # Medido em mg
        self.noise_emission: List[float] = []  # Medido em db
        self.fuel_consumption: List[float] = []  # Medido em ml
        self.electricity_consumption: List[float] = []  # Medido em ??

    def add_detectors(self, detectors: Dict[str, Detector]):
        """
        Adiciona detectores ao histórico da Lane, se este possuir algum.
        """
        self.detectors = detectors

    def update_traffic_data(self,
                            time_instant: float,
                            average_speed: float,
                            vehicle_count: int,
                            # waiting_time: float,
                            # halting_vehicle_count: int,
                            # travel_time: float,
                            average_occupancy: float):
        """
        Atualiza as variáveis internas obtidas diretamente da simulação
        do SUMO e, portanto, considerada como valores ideais a serem
        perseguidos através dos dados obtidos pelos detectores.
        """
        # Atualiza os dados de tráfego
        self.sampling_time.append(time_instant)
        self.average_speed.append(average_speed)
        self.vehicle_count.append(vehicle_count)
        # self.waiting_time.append(waiting_time)
        # self.halting_vehicle_count.append(halting_vehicle_count)
        # self.average_speed.append(average_speed)
        self.average_occupancy.append(average_occupancy)

    def update_environmental_data(self,
                                  CO2_emission: float,
                                  CO_emission: float,
                                  HC_emission: float,
                                  NOx_emission: float,
                                  PMx_emission: float,
                                  noise_emission: float,
                                  fuel_consumption: float,
                                  electricity_cons: float):
        """
        Atualiza os dados ambientais adquiridos diretamente da simulação
        do SUMO. Serão utilizados como referência para trabalhos futuros.
        """
        # Atualiza os dados ambientais
        self.CO2_emission.append(CO2_emission)
        self.CO_emission.append(CO_emission)
        self.HC_emission.append(HC_emission)
        self.NOx_emission.append(NOx_emission)
        self.PMx_emission.append(PMx_emission)
        self.noise_emission.append(noise_emission)
        self.fuel_consumption.append(fuel_consumption)
        self.electricity_consumption.append(electricity_cons)

    def get_average_traffic_data_in_time(self,
                                         ti: float,
                                         tf: float) -> Dict[str, float]:
        """
        Retorna os valores médios dos dados de tráfego em um intervalo
        de tempo.
        """
        i_inic = 0
        i_final = 0
        inic_found = False
        final_not_found = True
        # Varre os tempos de amostragem para encontrar os índices
        for i, st in enumerate(self.sampling_time):
            if st > ti and not inic_found:
                i_inic = i
                inic_found = True
            elif st > tf:
                i_final = i
                final_not_found = False
                break
        if final_not_found:
            i_final = len(self.sampling_time) - 1
        # Obtém o valor médio de cada um dos dados de tráfego
        n_dados = len(self.sampling_time[i_inic:i_final])
        if n_dados == 0:
            return {"speed": 0.0,
                    "vehicle_count": 0.0,
                    "occupancy": 0.0}
        avg_speed = mean(self.average_speed[i_inic:i_final])
        avg_vc = mean(self.vehicle_count[i_inic:i_final])
        avg_occ = mean(self.average_occupancy[i_inic:i_final])
        return {"speed": avg_speed,
                "vehicle_count": avg_vc,
                "occupancy": avg_occ}

    def get_first_traffic_data_in_time(self,
                                       ti: float,
                                       tf: float) -> Dict[str, float]:
        """
        Retorna os valores médios dos dados de tráfego em um intervalo
        de tempo.
        """
        i_inic = 0
        i_final = 0
        inic_found = False
        final_not_found = True
        # Varre os tempos de amostragem para encontrar os índices
        for i, st in enumerate(self.sampling_time):
            if st > ti and not inic_found:
                i_inic = i
                inic_found = True
            elif st > tf:
                i_final = i
                final_not_found = False
                break
        if final_not_found:
            i_final = len(self.sampling_time) - 1
        # Obtém o valor médio de cada um dos dados de tráfego
        n_dados = len(self.sampling_time[i_inic:i_final])
        if n_dados == 0:
            return {"speed": 0.0,
                    "vehicle_count": 0.0,
                    "occupancy": 0.0}
        avg_speed = self.average_speed[i_inic:i_final][0]
        avg_vc = self.vehicle_count[i_inic:i_final][0]
        avg_occ = self.average_occupancy[i_inic:i_final][0]
        return {"speed": avg_speed,
                "vehicle_count": avg_vc,
                "occupancy": avg_occ}

    def get_last_traffic_data_in_time(self,
                                      ti: float,
                                      tf: float) -> Dict[str, float]:
        """
        Retorna os valores médios dos dados de tráfego em um intervalo
        de tempo.
        """
        i_inic = 0
        i_final = 0
        inic_found = False
        final_not_found = True
        # Varre os tempos de amostragem para encontrar os índices
        for i, st in enumerate(self.sampling_time):
            if st > ti and not inic_found:
                i_inic = i
                inic_found = True
            elif st > tf:
                i_final = i
                final_not_found = False
                break
        if final_not_found:
            i_final = len(self.sampling_time) - 1
        # Obtém o valor médio de cada um dos dados de tráfego
        n_dados = len(self.sampling_time[i_inic:i_final])
        if n_dados == 0:
            return {"speed": 0.0,
                    "vehicle_count": 0.0,
                    "occupancy": 0.0}
        avg_speed = self.average_speed[i_inic:i_final][-1]
        avg_vc = self.vehicle_count[i_inic:i_final][-1]
        avg_occ = self.average_occupancy[i_inic:i_final][-1]
        return {"speed": avg_speed,
                "vehicle_count": avg_vc,
                "occupancy": avg_occ}

    def get_max_traffic_data_in_time(self,
                                     ti: float,
                                     tf: float) -> Dict[str, float]:
        """
        Retorna os valores máximos dos dados de tráfego em um intervalo
        de tempo.
        """
        i_inic = 0
        i_final = 0
        inic_found = False
        final_not_found = True
        # Varre os tempos de amostragem para encontrar os índices
        for i, st in enumerate(self.sampling_time):
            if st > ti and not inic_found:
                i_inic = i
                inic_found = True
            elif st > tf:
                i_final = i
                final_not_found = False
                break
        if final_not_found:
            i_final = len(self.sampling_time) - 1
        # Obtém o valor médio de cada um dos dados de tráfego
        n_dados = len(self.sampling_time[i_inic:i_final])
        if n_dados == 0:
            return {"speed": 0.0,
                    "vehicle_count": 0.0,
                    "occupancy": 0.0}
        max_speed = max(self.average_speed[i_inic:i_final])
        max_vc = max(self.vehicle_count[i_inic:i_final])
        max_occ = max(self.average_occupancy[i_inic:i_final])
        return {"speed": max_speed,
                "vehicle_count": max_vc,
                "occupancy": max_occ}

    def export_traffic_data(self) -> DataFrame:
        """
        Função para exportar dados de tráfego associados à faixa na simulação.
        """

        # Converte para DataFrame
        history_df = DataFrame()
        history_df['sampling_time'] = self.sampling_time
        history_df['average_speed'] = self.average_speed
        history_df['vehicle_count'] = self.vehicle_count
        history_df['average_occupancy'] = self.average_occupancy
        history_df['lane_id'] = self.id

        return history_df
