# Classe de plano semafórico para simulação
# de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
# Imports de módulos específicos da aplicação
from model.traffic.interval import Interval
from model.traffic.stage import Stage
from model.traffic.setpoint import Setpoint
from model.network.traffic_light import TLState


class TrafficPlan:
    """
    """
    def __init__(self,
                 stage_count: int,
                 offset: int,
                 stages: List[Stage]):
        self.stage_count = stage_count
        self.offset = offset
        self.stages = stages
        stage_lengths = [stage.length for stage in self.stages]
        self.cycle_length = sum(stage_lengths)
        self.stage_starting_times = [sum(stage_lengths[:i])
                                     for i in range(len(stage_lengths) + 1)]

    def __str__(self):
        plan_str = ""
        for key, val in self.__dict__.items():
            if key == "stages":
                stage_str = "stages: \n"
                for stage in self.stages:
                    stage_str += "    " + str(stage) + "\n"
                plan_str += stage_str
            else:
                plan_str += "{}: {}\n".format(key, val)
        return plan_str

    def update(self, setpoint: Setpoint):
        """
        Atualiza o plano quando recebe um novo comando de alterar setpoints do
        algoritmo de controle centralizado.
        """
        new_lengths = setpoint.generate_stage_times()
        for stage, new_length in zip(self.stages, new_lengths):
            stage.update(new_length)
        self.cycle_length = sum(new_lengths)
        self.stage_starting_times = [sum(new_lengths[:i])
                                     for i in range(len(new_lengths) + 1)]

    def current_plan_stage(self, current_time: float) -> int:
        """
        A partir do instante de tempo fornecido, retorna qual o devido estágio
        do plano deve ser executado.
        """
        current_cycle_time = (((current_time % self.cycle_length)
                              - self.offset) % self.cycle_length)
        current_stage_idx = 0
        for i in range(len(self.stage_starting_times) - 1):
            previous = self.stage_starting_times[i]
            current = self.stage_starting_times[i + 1]
            if previous < current_cycle_time <= current:
                current_stage_idx = i
                break

        return current_stage_idx

    def current_tl_states(self, current_time: float) -> List[TLState]:
        """
        A partir do instante de tempo fornecido, retorna qual o devido estado
        de cada semáforo baseado no estágio.
        """
        current_cycle_time = (((current_time % self.cycle_length)
                              - self.offset) % self.cycle_length)
        current_stage_idx = 0
        stage_time = 0.0
        for i in range(len(self.stage_starting_times) - 1):
            previous = self.stage_starting_times[i]
            current = self.stage_starting_times[i + 1]
            if previous < current_cycle_time <= current:
                current_stage_idx = i
                stage_time = current_cycle_time - previous
                break

        return self.stages[current_stage_idx].current_tl_states(stage_time)

    @classmethod
    def from_json(cls, json_dict: dict):
        """
        Construtor de TrafficPlan a partir de arquivo JSON.
        """
        stage_count = json_dict["stage_count"]
        stages = [Stage.from_json(s) for s in json_dict["stages"]]
        offset = json_dict["offset"]
        return cls(stage_count,
                   offset,
                   stages)


if __name__ == "__main__":
    # Cria os intervalos do primeiro estágio
    i1 = Interval(30.0, [TLState.GREEN, TLState.RED])
    i2 = Interval(3.0, [TLState.AMBER, TLState.RED])
    i3 = Interval(2.0, [TLState.RED, TLState.RED])
    # Cria o primeiro estágio
    s1 = Stage(3, [i1, i2, i3])
    # Cria os intervalos do segundo estágio
    i1 = Interval(30.0, [TLState.RED, TLState.GREEN])
    i2 = Interval(3.0, [TLState.RED, TLState.AMBER])
    i3 = Interval(2.0, [TLState.RED, TLState.RED])
    # Cria o segundo estágio
    s2 = Stage(3, [i1, i2, i3])
    # Cria o plano
    plan = TrafficPlan(2, 0, [s1, s2])
    # Printa o plano para conferir
    print(plan)
    # Faz uma atualização com setpoint
    setpoint = Setpoint([50, 40], 10)
    plan.update(setpoint)
    # Printa o plano novamente
    print(plan)
