from __future__ import annotations

import glob
import time
from dataclasses import dataclass

import serial

from .bus import ReceivedFrame
from .damiao_can.protocol import CanFrame

DEFAULT_SERIAL_BAUD = 921600
DEFAULT_CAN_BITRATE = 1000000
SEND_FRAME_LEN = 30
RECV_FRAME_LEN = 16
SEND_CMD_FORWARD_NO_STATUS = 0x03
RECV_CMD_CAN_SUCCESS = 0x11


def list_serial_ports() -> list[str]:
    return sorted(set(glob.glob("/dev/cu.*") + glob.glob("/dev/tty.*")))


def encode_damiao_usbcan_frame(frame: CanFrame) -> bytes:
    if not 0x000 <= frame.arbitration_id <= 0x7FF:
        raise ValueError("CAN arbitration_id must be in standard 11-bit range 0x000..0x7FF")
    if len(frame.data) != 8:
        raise ValueError("Damiao USB-CAN frames require exactly 8 data bytes")

    packet = bytearray(SEND_FRAME_LEN)
    packet[0:2] = b"\x55\xaa"
    packet[2] = SEND_FRAME_LEN
    packet[3] = SEND_CMD_FORWARD_NO_STATUS
    packet[4:8] = (1).to_bytes(4, "little")
    packet[8:12] = (10).to_bytes(4, "little")
    packet[12] = 0
    packet[13:17] = frame.arbitration_id.to_bytes(4, "little")
    packet[17] = 0
    packet[18] = 8
    packet[19] = 0
    packet[20] = 0
    packet[21:29] = frame.data
    packet[29] = 0
    return bytes(packet)


def decode_damiao_usbcan_packet(packet: bytes) -> CanFrame:
    if len(packet) != RECV_FRAME_LEN:
        raise ValueError("Damiao USB-CAN receive packet must be 16 bytes")
    if packet[0] != 0xAA:
        raise ValueError("invalid Damiao USB-CAN receive header")
    if packet[1] != RECV_CMD_CAN_SUCCESS:
        raise ValueError("Damiao USB-CAN packet is not a successful CAN receive frame")
    flags = packet[2]
    dlc = flags & 0x3F
    ide = bool(flags & 0x40)
    rtr = bool(flags & 0x80)
    if dlc != 8:
        raise ValueError("only 8-byte CAN data frames are supported")
    if ide:
        raise ValueError("extended CAN frames are not supported")
    if rtr:
        raise ValueError("remote CAN frames are not supported")
    can_id = int.from_bytes(packet[3:7], "little")
    if not 0x000 <= can_id <= 0x7FF:
        raise ValueError("CAN arbitration_id must be in standard 11-bit range 0x000..0x7FF")
    if packet[15] != 0x55:
        raise ValueError("invalid Damiao USB-CAN receive tail")
    return CanFrame(can_id, bytes(packet[7:15]))


@dataclass(frozen=True)
class DamiaoUsbCanConfig:
    port: str
    serial_baud: int = DEFAULT_SERIAL_BAUD
    can_bitrate: int = DEFAULT_CAN_BITRATE
    timeout: float = 0.1


class DamiaoUsbCanBus:
    def __init__(
        self,
        port: str,
        *,
        serial_baud: int = DEFAULT_SERIAL_BAUD,
        can_bitrate: int = DEFAULT_CAN_BITRATE,
        timeout: float = 0.1,
    ) -> None:
        self.config = DamiaoUsbCanConfig(port, serial_baud, can_bitrate, timeout)
        self._buffer = bytearray()
        if can_bitrate != DEFAULT_CAN_BITRATE:
            raise ValueError("Damiao USB-CAN backend does not modify device CAN bitrate; expected 1000000")
        try:
            self._serial = serial.Serial(port=port, baudrate=serial_baud, timeout=timeout, write_timeout=timeout)
        except (serial.SerialException, OSError) as error:
            raise RuntimeError(f"failed to open Damiao USB-CAN port {port}: {error}") from error

    def send(self, frame: CanFrame) -> None:
        try:
            self._serial.write(encode_damiao_usbcan_frame(frame))
            self._serial.flush()
        except (serial.SerialException, OSError) as error:
            raise RuntimeError(f"failed to write Damiao USB-CAN frame: {error}") from error

    def recv(self, timeout: float | None = None) -> ReceivedFrame | None:
        effective_timeout = self.config.timeout if timeout is None else timeout
        deadline = time.monotonic() + effective_timeout
        old_timeout = self._serial.timeout
        try:
            while True:
                decoded = self._try_decode_buffer()
                if decoded is not None:
                    return ReceivedFrame(frame=decoded, timestamp=time.time())

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._serial.timeout = remaining
                try:
                    chunk = self._serial.read(max(1, RECV_FRAME_LEN - len(self._buffer)))
                except (serial.SerialException, OSError) as error:
                    raise RuntimeError(f"failed to read Damiao USB-CAN frame: {error}") from error
                if chunk:
                    self._buffer.extend(chunk)
                elif time.monotonic() >= deadline:
                    return None
        finally:
            self._serial.timeout = old_timeout

    def close(self) -> None:
        if getattr(self, "_serial", None) and self._serial.is_open:
            self._serial.close()

    def _try_decode_buffer(self) -> CanFrame | None:
        while True:
            try:
                header_index = self._buffer.index(0xAA)
            except ValueError:
                self._buffer.clear()
                return None
            if header_index:
                del self._buffer[:header_index]
            if len(self._buffer) < RECV_FRAME_LEN:
                return None
            packet = bytes(self._buffer[:RECV_FRAME_LEN])
            if packet[15] != 0x55:
                del self._buffer[0]
                continue
            del self._buffer[:RECV_FRAME_LEN]
            try:
                return decode_damiao_usbcan_packet(packet)
            except ValueError:
                continue
