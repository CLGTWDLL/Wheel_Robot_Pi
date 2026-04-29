from __future__ import annotations

import time
from collections import deque

from .bus import ReceivedFrame
from .damiao_can.protocol import CanFrame


class FakeCanBus:
    def __init__(self, rx_frames: list[ReceivedFrame | CanFrame] | None = None) -> None:
        self.sent_frames: list[CanFrame] = []
        self._rx: deque[ReceivedFrame] = deque()
        self.closed = False
        for frame in rx_frames or []:
            self.inject_rx(frame)

    def send(self, frame: CanFrame) -> None:
        if self.closed:
            raise RuntimeError("bus is closed")
        self.sent_frames.append(frame)

    def recv(self, timeout: float | None = None) -> ReceivedFrame | None:
        if self.closed:
            raise RuntimeError("bus is closed")
        if self._rx:
            return self._rx.popleft()
        if timeout and timeout > 0:
            time.sleep(min(timeout, 0.001))
        return None

    def inject_rx(self, frame: ReceivedFrame | CanFrame) -> None:
        if isinstance(frame, ReceivedFrame):
            self._rx.append(frame)
        else:
            self._rx.append(ReceivedFrame(frame=frame))

    def close(self) -> None:
        self.closed = True
