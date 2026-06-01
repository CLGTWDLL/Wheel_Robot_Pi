from __future__ import annotations

# math 用于角度和弧度转换。
import math

# dataclass 用于保存配置和观测量。
from dataclasses import dataclass

# 同时兼容直接运行和包导入。
try:
    from imu_can import IMUState
except ImportError:
    from .imu_can import IMUState


@dataclass
class BalanceSignalConfig:
    """IMU 到轮腿平衡控制坐标的转换配置。"""

    # pitch 零偏，单位 deg。
    pitch_zero_deg: float = 0.0

    # roll 零偏，单位 deg。
    roll_zero_deg: float = 0.0

    # pitch 角速度零偏，单位 rad/s。
    pitch_rate_bias_rad_s: float = 0.0

    # roll 角速度零偏，单位 rad/s。
    roll_rate_bias_rad_s: float = 0.0

    # pitch 方向符号。
    pitch_sign: float = 1.0

    # roll 方向符号。
    roll_sign: float = 1.0

    # pitch_rate 方向符号。
    pitch_rate_sign: float = 1.0

    # roll_rate 方向符号。
    roll_rate_sign: float = 1.0

    # 手册定义绕 Y 轴为 pitch，所以默认使用 gyro_y。
    pitch_rate_axis: str = "gyro_y_rad_s"

    # 手册定义绕 X 轴为 roll，所以默认使用 gyro_x。
    roll_rate_axis: str = "gyro_x_rad_s"


@dataclass
class BalanceObservation:
    """控制器使用的 IMU 平衡观测量。"""

    # 观测时间戳。
    timestamp: float

    # pitch，单位 rad。
    pitch_rad: float

    # roll，单位 rad。
    roll_rad: float

    # pitch，单位 deg。
    pitch_deg: float

    # roll，单位 deg。
    roll_deg: float

    # pitch 角速度，单位 rad/s。
    pitch_rate_rad_s: float

    # roll 角速度，单位 rad/s。
    roll_rate_rad_s: float

    def as_controller_vector(self) -> tuple[float, float, float, float]:
        """返回控制器常用顺序：pitch、roll、pitch_rate、roll_rate。"""
        return (
            self.pitch_rad,
            self.roll_rad,
            self.pitch_rate_rad_s,
            self.roll_rate_rad_s,
        )


class BalanceSignalProcessor:
    """把 IMU 原始状态转换成平衡控制观测。"""

    def __init__(self, config: BalanceSignalConfig | None = None):
        """保存坐标系和零偏配置。"""
        self.config = config or BalanceSignalConfig()

    def update(self, state: IMUState) -> BalanceObservation | None:
        """从一帧 IMUState 生成 BalanceObservation。"""
        if not state.has_balance_data:
            return None

        pitch_raw_deg = self.config.pitch_sign * state.pitch_deg
        roll_raw_deg = self.config.roll_sign * state.roll_deg
        pitch_deg = pitch_raw_deg - self.config.pitch_zero_deg
        roll_deg = roll_raw_deg - self.config.roll_zero_deg

        pitch_rate_raw = self.config.pitch_rate_sign * _read_axis(
            state, self.config.pitch_rate_axis
        )
        roll_rate_raw = self.config.roll_rate_sign * _read_axis(
            state, self.config.roll_rate_axis
        )

        return BalanceObservation(
            timestamp=state.timestamp,
            pitch_rad=math.radians(pitch_deg),
            roll_rad=math.radians(roll_deg),
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            pitch_rate_rad_s=pitch_rate_raw - self.config.pitch_rate_bias_rad_s,
            roll_rate_rad_s=roll_rate_raw - self.config.roll_rate_bias_rad_s,
        )

    def set_zero_from_state(self, state: IMUState) -> bool:
        """用当前单帧 IMU 数据设置零偏。"""
        if not state.has_balance_data:
            return False

        self.config.pitch_zero_deg = self.config.pitch_sign * state.pitch_deg
        self.config.roll_zero_deg = self.config.roll_sign * state.roll_deg
        self.config.pitch_rate_bias_rad_s = self.config.pitch_rate_sign * _read_axis(
            state, self.config.pitch_rate_axis
        )
        self.config.roll_rate_bias_rad_s = self.config.roll_rate_sign * _read_axis(
            state, self.config.roll_rate_axis
        )
        return True

    def set_zero_from_states(self, states: list[IMUState]) -> bool:
        """用多帧 IMU 数据平均设置零偏。"""
        valid = [state for state in states if state.has_balance_data]
        if not valid:
            return False

        self.config.pitch_zero_deg = sum(
            self.config.pitch_sign * state.pitch_deg for state in valid
        ) / len(valid)
        self.config.roll_zero_deg = sum(
            self.config.roll_sign * state.roll_deg for state in valid
        ) / len(valid)
        self.config.pitch_rate_bias_rad_s = sum(
            self.config.pitch_rate_sign * _read_axis(state, self.config.pitch_rate_axis)
            for state in valid
        ) / len(valid)
        self.config.roll_rate_bias_rad_s = sum(
            self.config.roll_rate_sign * _read_axis(state, self.config.roll_rate_axis)
            for state in valid
        ) / len(valid)
        return True


def _read_axis(state: IMUState, axis_name: str) -> float:
    """按配置读取指定陀螺仪轴。"""
    value = getattr(state, axis_name, None)
    if value is None:
        raise ValueError(f"IMU state does not contain {axis_name}")
    return value
