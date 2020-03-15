# Classe de intervalo para simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 12 de Março de 2020

# Imports gerais de módulos padrão
from typing import List
# Imports de módulos específicos da aplicação
from model.network.traffic_light import TLState


class Interval:
    """
    """
    def __init__(self, length: float, states: List[TLState]):
        self.length = length
        self.states = states

    def __str__(self):
        interval_str = ""
        for key, val in self.__dict__.items():
            interval_str += "        {}: {}\n".format(key, val)
        return interval_str

    def update(self, new_length: float):
        """
        Atualiza a duração do intervalo.
        """
        self.length = new_length

    @classmethod
    def from_json(cls, json_dict: dict):
        length = json_dict["length"]
        states = [TLState.from_json(s) for s in json_dict["states"]]
        return cls(length, states)


if __name__ == "__main__":
    # Cria um objeto intervalo
    i = Interval(30.0, [TLState.GREEN, TLState.RED])
    # Printa para conferir os atributos
    print(i)
    # Atualiza a duração
    i.update(40.0)
    # Printa novamente
    print(i)
