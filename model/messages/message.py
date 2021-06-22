from dataclasses import dataclass
from abc import abstractmethod
from typing import Any


@dataclass
class Message:
    """
    """

    @staticmethod
    @abstractmethod
    def from_dict(obj: Any) -> 'Message':
        pass

    @abstractmethod
    def to_dict(self) -> dict:
        pass
