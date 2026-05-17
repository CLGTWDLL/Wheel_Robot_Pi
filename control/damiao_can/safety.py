from __future__ import annotations

import math

from .motors import MotorConfig

DEFAULT_TEST_TORQUE = 0.02
MAX_TEST_TORQUE = 0.05
MAX_NONZERO_DURATION = 10
DEFAULT_PERIOD = 0.002
GROUP_ZERO_DURATION = 1.0
DEFAULT_PAIR_MIT_VELOCITY = 0.2
MAX_PAIR_MIT_VELOCITY = 5.0
DEFAULT_PAIR_MIT_KD = 0.05
MAX_PAIR_MIT_KD = 0.1
DEFAULT_PAIR_MIT_TORQUE = 0.0
MAX_PAIR_MIT_TORQUE = 1.0
DEFAULT_PAIR_WHEEL_VELOCITY = 0.2
MAX_PAIR_WHEEL_VELOCITY = 5.0


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


def validate_pair_targets(dm4310_left: MotorConfig, dmh6215_left: MotorConfig) -> None:
    if dm4310_left.name != "dm4310_left":
        raise ValueError("pair MIT target must be dm4310_left")
    if dmh6215_left.name != "dmh6215_left":
        raise ValueError("pair wheel target must be dmh6215_left")
    if dm4310_left.mode != "mit":
        raise ValueError("dm4310_left must be MIT mode")
    # 2026-05-16: DMH6215 已切换到 MIT。配合 motors.py。
    if dmh6215_left.mode != "mit":
        raise ValueError("dmh6215_left must be MIT mode")


def validate_pair_mit_velocity(velocity: float) -> float:
    _require_finite("velocity", velocity)
    if abs(velocity) > MAX_PAIR_MIT_VELOCITY:
        raise ValueError(f"pair MIT velocity magnitude must be <= {MAX_PAIR_MIT_VELOCITY}")
    return velocity


def validate_pair_mit_kd(kd: float) -> float:
    _require_finite("kd", kd)
    if not 0.0 <= kd <= MAX_PAIR_MIT_KD:
        raise ValueError(f"pair MIT kd must be within [0.0, {MAX_PAIR_MIT_KD}]")
    return kd


def validate_pair_mit_torque(torque: float) -> float:
    _require_finite("torque", torque)
    if abs(torque) > MAX_PAIR_MIT_TORQUE:
        raise ValueError(f"pair MIT torque magnitude must be <= {MAX_PAIR_MIT_TORQUE}")
    return torque


def validate_pair_wheel_velocity(velocity: float) -> float:
    _require_finite("wheel_velocity", velocity)
    if abs(velocity) > MAX_PAIR_WHEEL_VELOCITY:
        raise ValueError(f"pair wheel velocity magnitude must be <= {MAX_PAIR_WHEEL_VELOCITY}")
    return velocity


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
