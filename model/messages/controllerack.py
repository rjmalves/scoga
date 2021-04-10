
from dataclasses import dataclass
from typing import Any


@dataclass
class ControllerAckMessage:
    """
    Mensagem enviada pelos controladores semafóricos
    em cada atualização de semáforo.
    """
    controller_id: str
    cycle: int
    stage: int
    interval: int
    time: float

    @staticmethod
    def from_dict(obj: Any) -> 'ControllerAckMessage':
        assert isinstance(obj, dict)
        controller_id = obj["controller_id"]
        cycle = obj["cycle"]
        stage = obj["stage"]
        interval = obj["interval"]
        time = obj["time"]
        return ControllerAckMessage(controller_id,
                                    cycle,
                                    stage,
                                    interval,
                                    time)

    def to_dict(self) -> dict:
        result = {}
        result["controller_id"] = self.controller_id
        result["cycle"] = self.cycle
        result["stage"] = self.stage
        result["interval"] = self.interval
        result["time"] = self.time
        return result
