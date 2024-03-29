# Classe de estágio para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
# Imports de módulos específicos da aplicação
from model.traffic.interval import Interval
from model.network.traffic_light import TLState


class Stage:
    """
    """
    MIN_INTERVALS = [5.0, 3.0, 2.0]

    def __init__(self, interval_count: int, intervals: List[Interval]):
        self.interval_count = interval_count
        self.intervals = intervals
        inter_lengths = [interval.length for interval in self.intervals]
        self.length = sum(inter_lengths)
        self.interval_starting_times = [sum(inter_lengths[:i])
                                        for i in range(len(inter_lengths))]
        self.current_interval_idx = 0
        self.last_interval_start = [0. for i in range(self.interval_count)]

    def __str__(self):
        stage_str = ""
        for key, val in self.__dict__.items():
            if key == "intervals":
                interval_str = "intervals: \n"
                for inter in self.intervals:
                    interval_str += "    " + str(inter)
                stage_str += interval_str
            else:
                stage_str += "{}: {}\n".format(key, val)
        return stage_str

    def update(self, new_length: float):
        """
        Atualiza a duração do estágio alterando o primeiro intervalo.
        """
        previous_length = self.length
        length_diff = new_length - previous_length
        new_main_interval_length = self.intervals[0].length + length_diff
        self.intervals[0].update(new_main_interval_length)
        inter_lengths = [interval.length for interval in self.intervals]
        self.length = sum(inter_lengths)
        self.interval_starting_times = [sum(inter_lengths[:i])
                                        for i in range(len(inter_lengths) + 1)]

    def current_tl_states(self,
                          current_time: float,
                          stage_time: float,
                          start: bool = False) -> List[TLState]:
        """
        Verifica qual intervalo do estágio está sendo executado no momento e
        retorna o estado dos grupos semafóricos.
        """
        new_interval_idx = 0
        interval_times = self.interval_starting_times + [self.length]
        for i in range(len(interval_times) - 1):
            previous = interval_times[i]
            current = interval_times[i + 1]
            if previous <= stage_time < current:
                new_interval_idx = i
                break

        if start:
            self.current_interval_idx = new_interval_idx
        else:
            # Confere a restrição de não saltar intervalos
            next_interval = ((self.current_interval_idx + 1)
                             % self.interval_count)
            if (new_interval_idx != next_interval and
                    new_interval_idx != self.current_interval_idx):
                new_interval_idx = next_interval

            # Confere a restrição de tempo de segurança
            if (current_time - self.last_interval_start[new_interval_idx]
                    >= Stage.MIN_INTERVALS[new_interval_idx]):
                self.current_interval_idx = new_interval_idx
                self.last_interval_start[new_interval_idx] = current_time

        return self.intervals[self.current_interval_idx].states

    @classmethod
    def from_json(cls, json_dict: dict):
        interval_count = json_dict["interval_count"]
        intervals = [Interval.from_json(i) for i in json_dict["intervals"]]
        return cls(interval_count, intervals)
