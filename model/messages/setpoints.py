
from dataclasses import dataclass
from model.traffic.setpoint import Setpoint
from typing import Any


@dataclass
class SetpointsMessage:
    """
    Mensagem enviada pelos históricos de nó avisando a
    mudança de ciclo.
    """
    ctrl_id: str
    setpoint: Setpoint

    @staticmethod
    def from_dict(obj: Any) -> 'SetpointsMessage':
        assert isinstance(obj, dict)
        ctrl_id = obj["ctrl_id"]
        setpoint = Setpoint.from_json(obj["setpoint"])
        return SetpointsMessage(ctrl_id,
                                setpoint)

    def to_dict(self) -> dict:
        result = {}
        result["ctrl_id"] = self.ctrl_id
        result["setpoint"] = self.setpoint.to_json()
        return result
