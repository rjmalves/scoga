# Classe de setpoints de controle que alteram o plano de tráfego
# para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
# Imports de módulos específicos da aplicação


class Setpoint:
    """
    """
    def __init__(self, default_stage_lengths: List[int], offset: int):
        self.cycle = sum(default_stage_lengths)
        self.splits = [length / self.cycle for length in default_stage_lengths]
        self.offset = offset

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        setpoint_set = ""
        for key, val in self.__dict__.items():
            setpoint_set += "{}: {}  ".format(key, val)
        return setpoint_set

    def to_json(self) -> dict:
        """
        """
        json_data: dict = {}
        json_data["default_stage_lengths"] = self.generate_stage_times()
        json_data["offset"] = self.offset
        return json_data

    @classmethod
    def from_json(cls, json_data: dict):
        """
        """
        return cls(**json_data)

    def generate_stage_times(self) -> List[int]:
        """
        Calcula as novas durações dos estágios baseados no valor do split e do
        ciclo atuais.
        """
        # Gera os valores puros, não arredondados
        stage_lengths = [s * self.cycle for s in self.splits]
        # Arredonda os valores (duração sempre em segundos)
        stage_lengths_round = [int(s) for s in stage_lengths]
        # Garante que a soma dos estágios vale o ciclo adicionando ao primeiro
        # a diferença entre estes
        stage_lengths_round[0] += int(self.cycle - sum(stage_lengths_round))
        # Retorna o vetor
        return stage_lengths_round
