
from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class SemaphoresMessage:
    """
    Mensagem enviada pelos controladores semafóricos
    em cada atualização de semáforo.
    """
    changed_semaphores: Dict[str, str]

    @staticmethod
    def from_dict(obj: Any) -> 'SemaphoresMessage':
        assert isinstance(obj, dict)
        changed_semaphores = {}
        for k, v in obj["changed_semaphores"].items():
            changed_semaphores[k] = v 
        return SemaphoresMessage(changed_semaphores)

    def to_dict(self) -> dict:
        result = {}
        result["changed_semaphores"] = self.changed_semaphores
        return result


