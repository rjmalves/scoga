from dataclasses import dataclass
from typing import Any

from model.messages.message import Message


@dataclass
class ShutdownMessage(Message):
    """
    Mensagem enviada para terminar a execução de
    um controlador semafórico.
    """
    ctrl_id: str

    @staticmethod
    def from_dict(obj: Any) -> 'ShutdownMessage':
        assert isinstance(obj, dict)
        time = obj["ctrl_id"]
        return ShutdownMessage(time)

    def to_dict(self) -> dict:
        result = {}
        result["ctrl_id"] = self.ctrl_id
        return result
