from __future__ import annotations

import time

from ..bus import CanBus
from .motors import get_motor
from .protocol import CanFrame, disable_frame, enable_frame, mit_frame, torque_frame
from .safety import (
    DEFAULT_PERIOD,
    DEFAULT_TEST_TORQUE,
    GROUP_ZERO_DURATION,
    conservative_group_torque,
    validate_mit_parameters,
    validate_nonzero_duration,
    validate_period,
    validate_test_torque,
    validate_zero_duration,
)

GROUP_ORDER = ("dm4310_left", "dm4310_right", "dmh6215_left", "dmh6215_right")


def group_zero_frames() -> tuple[CanFrame, ...]:
    frames: list[CanFrame] = []
    for name in GROUP_ORDER:
        motor = get_motor(name)
        if motor.mode == "mit":
            validate_mit_parameters(motor)
            frames.append(mit_frame(motor))
        elif motor.mode == "torque":
            frames.append(torque_frame(motor, 0.0))
        else:
            raise ValueError(f"unsupported motor mode: {motor.mode}")
    return tuple(frames)


def send_frames(bus: CanBus, frames: tuple[CanFrame, ...] | list[CanFrame]) -> None:
    for frame in frames:
        bus.send(frame)


def run_group_zero(bus: CanBus, *, duration: float = GROUP_ZERO_DURATION, period: float = DEFAULT_PERIOD) -> None:
    duration = validate_zero_duration(duration)
    period = validate_period(period)
    _run_repeating(bus, group_zero_frames(), duration=duration, period=period)


def run_torque_test(
    bus: CanBus,
    motor_name: str,
    *,
    torque: float = DEFAULT_TEST_TORQUE,
    duration: float,
    period: float = DEFAULT_PERIOD,
) -> None:
    motor = get_motor(motor_name)
    torque = validate_test_torque(motor, torque)
    duration = validate_nonzero_duration(duration)
    period = validate_period(period)
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
    bus: CanBus,
    *,
    torque: float = DEFAULT_TEST_TORQUE,
    duration: float,
    period: float = DEFAULT_PERIOD,
) -> None:
    duration = validate_nonzero_duration(duration)
    period = validate_period(period)
    commands = conservative_group_torque(torque)
    torque_by_name = {command.motor.name: command.torque for command in commands}
    cycle_frames = [
        mit_frame(get_motor("dm4310_left")),
        mit_frame(get_motor("dm4310_right")),
        torque_frame(get_motor("dmh6215_left"), torque_by_name["dmh6215_left"]),
        torque_frame(get_motor("dmh6215_right"), torque_by_name["dmh6215_right"]),
    ]
    try:
        send_frames(bus, group_zero_frames())
        _run_repeating(bus, cycle_frames, duration=duration, period=period)
    finally:
        send_frames(bus, group_zero_frames())


def single_motor_command_frame(motor_name: str, action: str, *, torque: float = DEFAULT_TEST_TORQUE) -> CanFrame:
    motor = get_motor(motor_name)
    if action == "enable":
        return enable_frame(motor)
    if action == "disable":
        return disable_frame(motor)
    if action == "torque":
        return torque_frame(motor, validate_test_torque(motor, torque))
    raise ValueError(f"unsupported motor action: {action}")


def _run_repeating(bus: CanBus, frames: list[CanFrame] | tuple[CanFrame, ...], *, duration: float, period: float) -> None:
    deadline = time.monotonic() + duration
    while True:
        send_frames(bus, frames)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(period, remaining))
