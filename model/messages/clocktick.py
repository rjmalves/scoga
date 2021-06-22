from dataclasses import dataclass
from typing import Any

from model.messages.message import Message


@dataclass
class ClockTickMessage(Message):
    """
    Mensagem enviada pelos controladores semafóricos
    em cada instante de tempo.
    """
    time: float

    @staticmethod
    def from_dict(obj: Any) -> 'ClockTickMessage':
        assert isinstance(obj, dict)
        time = obj["time"]
        return ClockTickMessage(time)

    def to_dict(self) -> dict:
        result = {}
        result["time"] = self.time
        return result
