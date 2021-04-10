
from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class SemaphoresMessage:
    """
    Mensagem enviada pelos controladores semafóricos
    em cada atualização de semáforo.
    """
    changed_semaphores: Dict[str, str]
    new_cycle: int
    new_stage: int
    new_interval: int
    current_time: float

    @staticmethod
    def from_dict(obj: Any) -> 'SemaphoresMessage':
        assert isinstance(obj, dict)
        changed_semaphores = {}
        for k, v in obj["changed_semaphores"].items():
            changed_semaphores[k] = v
        new_cycle = obj["new_cycle"]
        new_stage = obj["new_stage"]
        new_interval = obj["new_interval"]
        current_time = obj["current_time"]
        return SemaphoresMessage(changed_semaphores,
                                 new_cycle,
                                 new_stage,
                                 new_interval,
                                 current_time)

    def to_dict(self) -> dict:
        result = {}
        result["changed_semaphores"] = self.changed_semaphores
        result["new_cycle"] = self.new_cycle
        result["new_stage"] = self.new_stage
        result["new_interval"] = self.new_interval
        result["current_time"] = self.current_time
        return result
