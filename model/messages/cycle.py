
from dataclasses import dataclass
from typing import Any


@dataclass
class CycleMessage:
    """
    Mensagem enviada pelos históricos de nó avisando a
    mudança de ciclo.
    """
    node_id: str
    cycle: int

    @staticmethod
    def from_dict(obj: Any) -> 'CycleMessage':
        assert isinstance(obj, dict)
        node_id = obj["node_id"]
        cycle = obj["cycle"]
        return CycleMessage(node_id,
                            cycle)

    def to_dict(self) -> dict:
        result = {}
        result["node_id"] = self.node_id
        result["cycle"] = self.cycle
        return result
