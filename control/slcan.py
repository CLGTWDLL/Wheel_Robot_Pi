from __future__ import annotations

import glob
import time
from dataclasses import dataclass

import serial

from .bus import ReceivedFrame
from .damiao_can.protocol import CanFrame

BITRATE_CODES = {
    10000: "0",
    20000: "1",
    50000: "2",
    100000: "3",
    125000: "4",
    250000: "5",
    500000: "6",
    800000: "7",
    1000000: "8",
}


def encode_slcan_frame(frame: CanFrame) -> str:
    if len(frame.data) > 8:
        raise ValueError("SLCAN data length must be <= 8")
    return f"t{frame.arbitration_id:03X}{len(frame.data):X}{frame.data.hex().upper()}\r"


def decode_slcan_line(line: str | bytes) -> CanFrame:
    if isinstance(line, bytes):
        line = line.decode("ascii", errors="strict")
    line = line.strip()
    if len(line) < 5 or line[0] != "t":
        raise ValueError("only standard SLCAN data frames are supported")
    try:
        arbitration_id = int(line[1:4], 16)
        dlc = int(line[4], 16)
    except ValueError as error:
        raise ValueError("invalid SLCAN header") from error
    if dlc > 8:
        raise ValueError("invalid SLCAN dlc")
    payload = line[5:]
    if len(payload) != dlc * 2:
        raise ValueError("SLCAN payload length does not match dlc")
    try:
        data = bytes.fromhex(payload)
    except ValueError as error:
        raise ValueError("invalid SLCAN payload") from error
    return CanFrame(arbitration_id, data.ljust(8, b"\x00"))


def list_serial_ports() -> list[str]:
    return sorted(set(glob.glob("/dev/cu.*") + glob.glob("/dev/tty.*")))


@dataclass(frozen=True)
class SlcanConfig:
    port: str
    serial_baud: int = 115200
    can_bitrate: int = 1000000
    timeout: float = 0.1


class SlcanBus:
    def __init__(self, port: str, *, serial_baud: int = 115200, can_bitrate: int = 1000000, timeout: float = 0.1) -> None:
        self.config = SlcanConfig(port, serial_baud, can_bitrate, timeout)
        if can_bitrate not in BITRATE_CODES:
            raise ValueError(f"unsupported CAN bitrate: {can_bitrate}")
        try:
            self._serial = serial.Serial(port=port, baudrate=serial_baud, timeout=timeout, write_timeout=timeout)
        except serial.SerialException as error:
            raise RuntimeError(f"failed to open SLCAN port {port}: {error}") from error
        try:
            self._write_command("C")
            self._write_command("S" + BITRATE_CODES[can_bitrate])
            self._write_command("O")
        except Exception:
            self.close()
            raise

    def send(self, frame: CanFrame) -> None:
        self._write_ascii(encode_slcan_frame(frame))

    def recv(self, timeout: float | None = None) -> ReceivedFrame | None:
        old_timeout = self._serial.timeout
        if timeout is not None:
            self._serial.timeout = timeout
        try:
            raw = self._serial.readline()
        except serial.SerialException as error:
            raise RuntimeError(f"failed to read SLCAN frame: {error}") from error
        finally:
            if timeout is not None:
                self._serial.timeout = old_timeout
        if not raw:
            return None
        return ReceivedFrame(frame=decode_slcan_line(raw), timestamp=time.time())

    def close(self) -> None:
        if getattr(self, "_serial", None) and self._serial.is_open:
            try:
                self._write_command("C")
            finally:
                self._serial.close()

    def _write_command(self, command: str) -> None:
        self._write_ascii(command + "\r")

    def _write_ascii(self, text: str) -> None:
        try:
            self._serial.write(text.encode("ascii"))
            self._serial.flush()
        except serial.SerialException as error:
            raise RuntimeError(f"failed to write SLCAN command: {error}") from error
