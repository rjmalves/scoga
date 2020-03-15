# Classe de estágio para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from typing import List, Dict
# Imports de módulos específicos da aplicação
from model.traffic.interval import Interval
from model.traffic.traffic_light import TLState


class Stage:
    """
    """
    def __init__(self, interval_count: int, intervals: List[Interval]):
        self.interval_count = interval_count
        self.intervals = intervals
        inter_lengths = [interval.length for interval in self.intervals]
        self.length = sum(inter_lengths)
        self.interval_starting_times = [sum(inter_lengths[:i])
                                        for i in range(len(inter_lengths) + 1)]

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

    def current_tl_states(self, current_time: float) -> Dict[str, TLState]:
        """
        Verifica qual intervalo do estágio está sendo executado no momento e
        retorna o estado dos grupos semafóricos.
        """
        current_interval_idx = 0
        for i in range(len(self.interval_starting_times) - 1):
            previous = self.interval_starting_times[i]
            current = self.interval_starting_times[i + 1]
            if previous < current_time <= current:
                current_interval_idx = i
                break

        return self.intervals[current_interval_idx].states

    @classmethod
    def from_json(cls, json_dict: dict):
        interval_count = json_dict["interval_count"]
        intervals = [Interval.from_json(i) for i in json_dict["intervals"]]
        return cls(interval_count, intervals)


if __name__ == "__main__":
    # Cria três objetos intervalo
    i1 = Interval(30.0, {"TL1": TLState.GREEN, "TL2": TLState.RED})
    i2 = Interval(3.0, {"TL1": TLState.YELLOW, "TL2": TLState.RED})
    i3 = Interval(2.0, {"TL1": TLState.RED, "TL2": TLState.RED})
    # Cria um estágio
    stage = Stage(3, [i1, i2, i3])
    # Printa o estágio para conferir:
    print(stage)
    # Atualiza o intervalo principal:
    stage.update(40.0)
    # Printa o estágio para conferir:
    print(stage)
    # Testa a função de adquirir estado de semáforos
    for test_time in [0, 36.0, 39.0]:
        print("{}: {}".format(test_time, stage.current_tl_states(test_time)))
