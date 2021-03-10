# Modelo de veículo para a simulação de tráfego com controle em tempo real
#
# Rogerio José Menezes Alves
# Mestrando em Engenharia Elétrica - Universidade Federal do Espírito Santo
# 30 de Dezembro de 2020

class Vehicle:
    """
    """
    def __init__(self,
                 veh_id: str):
        self.id = veh_id
        self._departing_time = 0.0
        self._arriving_time = 0.0

    @property
    def departing_time(self):
        return self._departing_time

    @departing_time.setter
    def departing_time(self, t: float):
        self._departing_time = t

    @property
    def arriving_time(self):
        return self._arriving_time

    @arriving_time.setter
    def arriving_time(self, t: float):
        self._arriving_time = t

    @property
    def travel_time(self):
        tt = 0.0
        if (self._departing_time != 0.
                and self._arriving_time != 0.):
            tt = self._arriving_time - self._departing_time
        return tt
