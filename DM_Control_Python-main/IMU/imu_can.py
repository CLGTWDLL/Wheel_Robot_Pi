from __future__ import annotations

# struct 用于解析 USB 帧中的 little-endian float。
import struct

# time 用于给最新 IMU 状态打时间戳。
import time

# dataclass/replace 用于维护不可变风格的状态更新。
from dataclasses import dataclass, replace

# Iterable 让 CAN payload 接口支持 list/bytes/bytearray。
from typing import Iterable

# pyserial 用于读取 COM8 上的 USB 虚拟串口。
import serial


# USB 主动输出帧头。
USB_HEADER = b"\x55\xAA"

# USB 主动输出帧尾。
USB_TAIL = 0x0A

# 手册定义的 USB 帧长度。
USB_FRAME_LENGTHS = {
    0x01: 19,  # accel: 3 floats
    0x02: 19,  # gyro: 3 floats
    0x03: 19,  # euler: 3 floats
    0x04: 23,  # quaternion: 4 floats
}

# CAN 加速度映射范围。
ACCEL_CAN_MIN = -235.2
ACCEL_CAN_MAX = 235.2

# CAN 角速度映射范围，单位 rad/s。
GYRO_CAN_MIN = -34.88
GYRO_CAN_MAX = 34.88

# CAN pitch 映射范围，单位 deg。
PITCH_CAN_MIN = -90.0
PITCH_CAN_MAX = 90.0

# CAN roll 映射范围，单位 deg。
ROLL_CAN_MIN = -180.0
ROLL_CAN_MAX = 180.0

# CAN yaw 映射范围，单位 deg。
YAW_CAN_MIN = -180.0
YAW_CAN_MAX = 180.0


@dataclass
class IMUState:
    """IMU 最近一次解析得到的状态缓存。"""

    # 最近一次更新的时间戳。
    timestamp: float = 0.0

    # IMU 帧中的模块 ID。
    imu_id: int = 0

    # roll，单位 deg。
    roll_deg: float | None = None

    # pitch，单位 deg。
    pitch_deg: float | None = None

    # yaw，单位 deg。
    yaw_deg: float | None = None

    # X 轴角速度，单位 rad/s。
    gyro_x_rad_s: float | None = None

    # Y 轴角速度，单位 rad/s。
    gyro_y_rad_s: float | None = None

    # Z 轴角速度，单位 rad/s。
    gyro_z_rad_s: float | None = None

    # X 轴加速度。
    accel_x: float | None = None

    # Y 轴加速度。
    accel_y: float | None = None

    # Z 轴加速度。
    accel_z: float | None = None

    @property
    def has_balance_data(self) -> bool:
        """判断 pitch/roll 和对应角速度是否已经齐全。"""
        return (
            self.roll_deg is not None
            and self.pitch_deg is not None
            and self.gyro_x_rad_s is not None
            and self.gyro_y_rad_s is not None
        )


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """按手册线性映射公式把无符号整数转成浮点数。"""
    span = x_max - x_min
    return float(x_int) * span / float((1 << bits) - 1) + x_min


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """计算 CRC16-CCITT，用于可选校验 USB 帧。"""
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


class DMIMUParser:
    """DM-IMU-L1 的 USB/CAN 数据解析器。"""

    def __init__(self, validate_crc: bool = False):
        """初始化解析器缓存和 CRC 校验开关。"""
        self.validate_crc = validate_crc
        self.buffer = bytearray()
        self.state = IMUState(timestamp=time.time())

    def feed_usb_bytes(self, data: bytes) -> list[IMUState]:
        """喂入任意长度串口字节，返回解析出的状态更新。"""
        self.buffer.extend(data)
        updates: list[IMUState] = []

        while True:
            start = self.buffer.find(USB_HEADER)
            if start < 0:
                self.buffer.clear()
                break
            if start > 0:
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break

            frame_type = self.buffer[3]
            frame_len = USB_FRAME_LENGTHS.get(frame_type)
            if frame_len is None:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                break
            if self.buffer[frame_len - 1] != USB_TAIL:
                del self.buffer[0]
                continue

            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]
            state = self.parse_usb_frame(frame)
            if state is not None:
                updates.append(state)

        return updates

    def parse_usb_frame(self, frame: bytes) -> IMUState | None:
        """解析单个 USB 主动输出帧。"""
        if len(frame) < 7 or frame[:2] != USB_HEADER or frame[-1] != USB_TAIL:
            return None
        if self.validate_crc and not self._crc_matches(frame):
            return None

        frame_type = frame[3]
        imu_id = frame[2]
        timestamp = time.time()

        if frame_type == 0x01:
            accel_x, accel_y, accel_z = struct.unpack("<fff", frame[4:16])
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                accel_x=accel_x,
                accel_y=accel_y,
                accel_z=accel_z,
            )
        elif frame_type == 0x02:
            gyro_x, gyro_y, gyro_z = struct.unpack("<fff", frame[4:16])
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                gyro_x_rad_s=gyro_x,
                gyro_y_rad_s=gyro_y,
                gyro_z_rad_s=gyro_z,
            )
        elif frame_type == 0x03:
            roll, pitch, yaw = struct.unpack("<fff", frame[4:16])
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                roll_deg=roll,
                pitch_deg=pitch,
                yaw_deg=yaw,
            )
        else:
            return None

        return self.state

    def update_from_can_payload(
        self, data: bytes | bytearray | Iterable[int], imu_id: int = 0
    ) -> IMUState | None:
        """解析 CAN 数据帧 payload；当前主程序不走 CAN，只保留备用。"""
        payload = bytes(data)
        if len(payload) != 8:
            return None

        frame_type = payload[0]
        timestamp = time.time()

        if frame_type == 0x01:
            accel_x = uint_to_float(_u16_le(payload[2:4]), ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16)
            accel_y = uint_to_float(_u16_le(payload[4:6]), ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16)
            accel_z = uint_to_float(_u16_le(payload[6:8]), ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16)
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                accel_x=accel_x,
                accel_y=accel_y,
                accel_z=accel_z,
            )
        elif frame_type == 0x02:
            gyro_x = uint_to_float(_u16_le(payload[2:4]), GYRO_CAN_MIN, GYRO_CAN_MAX, 16)
            gyro_y = uint_to_float(_u16_le(payload[4:6]), GYRO_CAN_MIN, GYRO_CAN_MAX, 16)
            gyro_z = uint_to_float(_u16_le(payload[6:8]), GYRO_CAN_MIN, GYRO_CAN_MAX, 16)
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                gyro_x_rad_s=gyro_x,
                gyro_y_rad_s=gyro_y,
                gyro_z_rad_s=gyro_z,
            )
        elif frame_type == 0x03:
            pitch = uint_to_float(_u16_le(payload[2:4]), PITCH_CAN_MIN, PITCH_CAN_MAX, 16)
            yaw = uint_to_float(_u16_le(payload[4:6]), YAW_CAN_MIN, YAW_CAN_MAX, 16)
            roll = uint_to_float(_u16_le(payload[6:8]), ROLL_CAN_MIN, ROLL_CAN_MAX, 16)
            self.state = replace(
                self.state,
                timestamp=timestamp,
                imu_id=imu_id,
                roll_deg=roll,
                pitch_deg=pitch,
                yaw_deg=yaw,
            )
        else:
            return None

        return self.state

    def _crc_matches(self, frame: bytes) -> bool:
        """检查 USB 帧 CRC；兼容大小端两种 CRC 字节序。"""
        expected = crc16_ccitt(frame[:-3])
        crc_bytes = frame[-3:-1]
        return expected in (
            int.from_bytes(crc_bytes, "little"),
            int.from_bytes(crc_bytes, "big"),
        )


class DMIMUSerialReader:
    """COM 口 IMU 读取器，内部使用 DMIMUParser。"""

    def __init__(
        self,
        port: str = "COM8",
        baudrate: int = 921600,
        timeout: float = 0.02,
        validate_crc: bool = False,
    ):
        """打开 IMU 串口并创建解析器。"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.parser = DMIMUParser(validate_crc=validate_crc)
        self.serial = serial.Serial(port, baudrate, timeout=timeout)

    def __enter__(self) -> "DMIMUSerialReader":
        """进入上下文时确保串口处于打开状态。"""
        if not self.serial.is_open:
            self.serial.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出上下文时关闭串口。"""
        self.close()
        return False

    def close(self) -> None:
        """关闭 IMU 串口。"""
        if self.serial.is_open:
            self.serial.close()

    def read_states(self) -> list[IMUState]:
        """读取当前串口缓存并返回所有状态更新。"""
        byte_count = self.serial.in_waiting or 1
        return self.parser.feed_usb_bytes(self.serial.read(byte_count))

    def send_usb_command(self, command: bytes | bytearray | Iterable[int]) -> None:
        """向 IMU 发送 USB 快捷指令。"""
        self.serial.write(bytes(command))

    def configure_usb_balance_stream(self, save: bool = False) -> None:
        """配置 IMU 只输出平衡控制需要的角速度和欧拉角。"""
        commands = [
            b"\xAA\x06\x01\x0D",  # enter setting mode
            b"\xAA\x01\x04\x0D",  # close accel output
            b"\xAA\x01\x15\x0D",  # open gyro output
            b"\xAA\x01\x16\x0D",  # open euler output
            b"\xAA\x01\x07\x0D",  # close quaternion output
        ]
        if save:
            commands.append(b"\xAA\x03\x01\x0D")
        commands.append(b"\xAA\x06\x00\x0D")

        for command in commands:
            self.send_usb_command(command)
            time.sleep(0.05)


def _u16_le(data: bytes) -> int:
    """按 little-endian 读取 uint16。"""
    return int.from_bytes(data, "little", signed=False)
