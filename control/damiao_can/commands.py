from __future__ import annotations

import importlib
import time
from collections.abc import Callable
from typing import Any

from .motors import get_motor
from .protocol import CanFrame, disable_frame, enable_frame, mit_frame, torque_frame


def _safety() -> Any:
    return importlib.import_module("control.damiao_can.safety")


DEFAULT_PAIR_MIT_KD = _safety().DEFAULT_PAIR_MIT_KD
DEFAULT_PAIR_MIT_TORQUE = _safety().DEFAULT_PAIR_MIT_TORQUE
DEFAULT_PAIR_MIT_VELOCITY = _safety().DEFAULT_PAIR_MIT_VELOCITY
DEFAULT_PAIR_WHEEL_VELOCITY = _safety().DEFAULT_PAIR_WHEEL_VELOCITY
DEFAULT_PERIOD = _safety().DEFAULT_PERIOD
DEFAULT_TEST_TORQUE = _safety().DEFAULT_TEST_TORQUE
GROUP_ZERO_DURATION = _safety().GROUP_ZERO_DURATION

GROUP_ORDER = ("dm4310_left", "dm4310_right", "dmh6215_left", "dmh6215_right")
PAIR_MIT_NAME = "dm4310_left"
PAIR_WHEEL_NAME = "dmh6215_left"


def group_zero_frames() -> tuple[CanFrame, ...]:
    frames: list[CanFrame] = []
    for name in GROUP_ORDER:
        motor = get_motor(name)
        if motor.mode == "mit":
            _safety().validate_mit_parameters(motor)
            frames.append(mit_frame(motor))
        elif motor.mode == "torque":
            frames.append(torque_frame(motor, 0.0))
        else:
            raise ValueError(f"unsupported motor mode: {motor.mode}")
    return tuple(frames)


def pair_zero_frames() -> tuple[CanFrame, CanFrame]:
    mit_motor = get_motor(PAIR_MIT_NAME)
    wheel_motor = get_motor(PAIR_WHEEL_NAME)
    _safety().validate_pair_targets(mit_motor, wheel_motor)
    _safety().validate_mit_parameters(mit_motor)
    _safety().validate_mit_parameters(wheel_motor)
    return (mit_frame(mit_motor), mit_frame(wheel_motor))


def send_frames(bus: Any, frames: tuple[CanFrame, ...] | list[CanFrame]) -> None:
    for frame in frames:
        bus.send(frame)


def run_group_zero(
    bus: Any,
    *,
    duration: float = GROUP_ZERO_DURATION,
    period: float = DEFAULT_PERIOD,
    telemetry: Callable[[Any], None] | None = None,
) -> None:
    duration = _safety().validate_zero_duration(duration)
    period = _safety().validate_period(period)
    _run_repeating(bus, group_zero_frames(), duration=duration, period=period, telemetry=telemetry)


def run_torque_test(
    bus: Any,
    motor_name: str,
    *,
    torque: float = DEFAULT_TEST_TORQUE,
    duration: float,
    period: float = DEFAULT_PERIOD,
) -> None:
    motor = get_motor(motor_name)
    torque = _safety().validate_test_torque(motor, torque)
    duration = _safety().validate_nonzero_duration(duration)
    period = _safety().validate_period(period)
    zero = torque_frame(motor, 0.0)
    try:
        bus.send(zero)
        bus.send(enable_frame(motor))
        _run_repeating(bus, [torque_frame(motor, torque)], duration=duration, period=period)
    finally:
        try:
            bus.send(zero)
        finally:
            bus.send(disable_frame(motor))


def run_group_conservative_test(
    bus: Any,
    *,
    torque: float = DEFAULT_TEST_TORQUE,
    duration: float,
    period: float = DEFAULT_PERIOD,
) -> None:
    _safety()._require_finite("torque", torque)
    duration = _safety().validate_nonzero_duration(duration)
    period = _safety().validate_period(period)
    cycle_frames = [
        mit_frame(get_motor("dm4310_left")),
        mit_frame(get_motor("dm4310_right")),
        mit_frame(get_motor("dmh6215_left")),
        mit_frame(get_motor("dmh6215_right")),
    ]
    try:
        send_frames(bus, group_zero_frames())
        _run_repeating(bus, cycle_frames, duration=duration, period=period)
    finally:
        send_frames(bus, group_zero_frames())


def run_pair_conservative_spin(
    bus: Any,
    *,
    velocity: float = DEFAULT_PAIR_MIT_VELOCITY,
    wheel_velocity: float = DEFAULT_PAIR_WHEEL_VELOCITY,
    torque: float = DEFAULT_PAIR_MIT_TORQUE,
    duration: float,
    period: float = DEFAULT_PERIOD,
    telemetry: Callable[[Any], None] | None = None,
) -> None:
    mit_motor = get_motor(PAIR_MIT_NAME)
    wheel_motor = get_motor(PAIR_WHEEL_NAME)
    _safety().validate_pair_targets(mit_motor, wheel_motor)
    velocity = _safety().validate_pair_mit_velocity(velocity)
    wheel_velocity = _safety().validate_pair_wheel_velocity(wheel_velocity)
    torque = _safety().validate_pair_mit_torque(torque)
    kd = _safety().validate_pair_mit_kd(DEFAULT_PAIR_MIT_KD)
    duration = _safety().validate_nonzero_duration(duration)
    period = _safety().validate_period(period)
    # 2026-05-16: DMH6215 也走 MIT 模式。用 kp=0、kd 阻尼跟踪 v_des 实现保守 spin。
    cycle_frames = [
        mit_frame(mit_motor, position=0.0, velocity=velocity, kp=0.0, kd=kd, torque=torque),
        mit_frame(wheel_motor, position=0.0, velocity=wheel_velocity, kp=0.0, kd=kd, torque=0.0),
    ]
    try:
        send_frames(bus, pair_zero_frames())
        bus.send(enable_frame(mit_motor))
        bus.send(enable_frame(wheel_motor))
        _run_repeating(bus, cycle_frames, duration=duration, period=period, telemetry=telemetry)
    finally:
        try:
            send_frames(bus, pair_zero_frames())
        finally:
            try:
                bus.send(disable_frame(mit_motor))
            finally:
                bus.send(disable_frame(wheel_motor))


def single_motor_command_frame(motor_name: str, action: str, *, torque: float = DEFAULT_TEST_TORQUE) -> CanFrame:
    motor = get_motor(motor_name)
    if action == "enable":
        return enable_frame(motor)
    if action == "disable":
        return disable_frame(motor)
    if action == "torque":
        return torque_frame(motor, _safety().validate_test_torque(motor, torque))
    raise ValueError(f"unsupported motor action: {action}")


def _run_repeating(
    bus: Any,
    frames: list[CanFrame] | tuple[CanFrame, ...],
    *,
    duration: float,
    period: float,
    telemetry: Callable[[Any], None] | None = None,
) -> None:
    deadline = time.monotonic() + duration
    telemetry_initial_timeout = min(0.01, max(0.0, period))
    while True:
        if time.monotonic() >= deadline:
            break
        send_frames(bus, frames)
        _drain_telemetry(bus, telemetry, initial_timeout=telemetry_initial_timeout, deadline=deadline)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(period, remaining))


def _drain_telemetry(
    bus: Any,
    telemetry: Callable[[Any], None] | None,
    *,
    initial_timeout: float,
    deadline: float,
) -> None:
    if telemetry is None:
        return
    timeout = initial_timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        received = bus.recv(timeout=min(timeout, remaining))
        if time.monotonic() >= deadline or received is None:
            return
        telemetry(received)
        if time.monotonic() >= deadline:
            return
        timeout = 0
