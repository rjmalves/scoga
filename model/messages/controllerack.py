
from dataclasses import dataclass
from typing import Any


@dataclass
class ControllerAckMessage:
    """
    Mensagem enviada pelos controladores semafóricos
    em cada atualização de semáforo.
    """
    controller_id: str

    @staticmethod
    def from_dict(obj: Any) -> 'ControllerAckMessage':
        assert isinstance(obj, dict)
        controller_id = obj["controller_id"]
        return ControllerAckMessage(controller_id)

    def to_dict(self) -> dict:
        result = {}
        result["controller_id"] = self.controller_id
        return result
