from __future__ import annotations

import math
from dataclasses import dataclass

from .motors import MotorConfig, get_motor

DEFAULT_TEST_TORQUE = 0.02
MAX_TEST_TORQUE = 0.05
MAX_NONZERO_DURATION = 0.5
DEFAULT_PERIOD = 0.05
GROUP_ZERO_DURATION = 1.0


@dataclass(frozen=True)
class GroupTorqueCommand:
    motor: MotorConfig
    torque: float


def _require_finite(name: str, value: float) -> float:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_nonzero_duration(duration: float) -> float:
    _require_finite("duration", duration)
    if duration <= 0:
        raise ValueError("duration must be positive")
    if duration > MAX_NONZERO_DURATION:
        raise ValueError(f"duration must be <= {MAX_NONZERO_DURATION}")
    return duration


def validate_zero_duration(duration: float) -> float:
    _require_finite("duration", duration)
    if duration <= 0:
        raise ValueError("duration must be positive")
    return duration


def validate_period(period: float) -> float:
    _require_finite("period", period)
    if period <= 0:
        raise ValueError("period must be positive")
    return period


def validate_test_torque(motor: MotorConfig, torque: float) -> float:
    require_torque_motor(motor)
    _require_finite("torque", torque)
    if abs(torque) > MAX_TEST_TORQUE:
        raise ValueError(f"test torque magnitude must be <= {MAX_TEST_TORQUE}")
    if abs(torque) > motor.t_max:
        raise ValueError(f"torque magnitude must be <= motor t_max {motor.t_max}")
    return torque


def require_torque_motor(motor: MotorConfig) -> MotorConfig:
    if motor.mode != "torque":
        raise ValueError(f"motor {motor.name} is mode={motor.mode}, not torque")
    return motor


def validate_mit_parameters(
    motor: MotorConfig,
    *,
    position: float = 0.0,
    velocity: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    torque: float = 0.0,
) -> None:
    if motor.mode != "mit":
        raise ValueError(f"motor {motor.name} is mode={motor.mode}, not mit")
    for name, value in {
        "position": position,
        "velocity": velocity,
        "kp": kp,
        "kd": kd,
        "torque": torque,
    }.items():
        _require_finite(name, value)
    if not -motor.p_max <= position <= motor.p_max:
        raise ValueError("position outside motor limit")
    if not -motor.v_max <= velocity <= motor.v_max:
        raise ValueError("velocity outside motor limit")
    if not motor.kp_min <= kp <= motor.kp_max:
        raise ValueError("kp outside motor limit")
    if not motor.kd_min <= kd <= motor.kd_max:
        raise ValueError("kd outside motor limit")
    if not -motor.t_max <= torque <= motor.t_max:
        raise ValueError("torque outside motor limit")


def conservative_group_torque(torque: float = DEFAULT_TEST_TORQUE) -> tuple[GroupTorqueCommand, ...]:
    left = get_motor("dmh6215_left")
    right = get_motor("dmh6215_right")
    return (
        GroupTorqueCommand(left, validate_test_torque(left, torque)),
        GroupTorqueCommand(right, validate_test_torque(right, torque)),
    )
