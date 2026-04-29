from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .damiao_can.protocol import CanFrame


@dataclass(frozen=True)
class ReceivedFrame:
    frame: CanFrame
    timestamp: float | None = None


class CanBus(Protocol):
    def send(self, frame: CanFrame) -> None: ...

    def recv(self, timeout: float | None = None) -> ReceivedFrame | None: ...

    def close(self) -> None: ...
