import math

import pytest

from control.damiao_can.motors import get_motor
from control.damiao_can.safety import (
    DEFAULT_TEST_TORQUE,
    MAX_NONZERO_DURATION,
    MAX_TEST_TORQUE,
    conservative_group_torque,
    require_torque_motor,
    validate_mit_parameters,
    validate_nonzero_duration,
    validate_test_torque,
    validate_zero_duration,
)


def test_duration_limits():
    assert validate_nonzero_duration(MAX_NONZERO_DURATION) == MAX_NONZERO_DURATION
    assert validate_zero_duration(1.0) == 1.0
    with pytest.raises(ValueError):
        validate_nonzero_duration(MAX_NONZERO_DURATION + 0.1)
    with pytest.raises(ValueError):
        validate_nonzero_duration(0)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_duration_rejects_non_finite(value):
    with pytest.raises(ValueError, match="finite"):
        validate_nonzero_duration(value)


def test_test_torque_only_allows_torque_mode_and_small_values():
    motor = get_motor("dmh6215_left")
    assert validate_test_torque(motor, DEFAULT_TEST_TORQUE) == DEFAULT_TEST_TORQUE
    with pytest.raises(ValueError, match="test torque"):
        validate_test_torque(motor, MAX_TEST_TORQUE + 0.01)
    with pytest.raises(ValueError, match="not torque"):
        validate_test_torque(get_motor("dm4310_left"), DEFAULT_TEST_TORQUE)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_torque_rejects_non_finite(value):
    with pytest.raises(ValueError, match="finite"):
        validate_test_torque(get_motor("dmh6215_left"), value)


def test_require_torque_motor_rejects_mit_motor():
    with pytest.raises(ValueError, match="not torque"):
        require_torque_motor(get_motor("dm4310_left"))


def test_validate_mit_parameters_rejects_non_finite_and_wrong_mode():
    validate_mit_parameters(get_motor("dm4310_left"), position=0, velocity=0, kp=0, kd=0, torque=0)
    with pytest.raises(ValueError, match="finite"):
        validate_mit_parameters(get_motor("dm4310_left"), torque=math.nan)
    with pytest.raises(ValueError, match="not mit"):
        validate_mit_parameters(get_motor("dmh6215_left"))


def test_conservative_group_torque_is_centralized_to_torque_motors():
    commands = conservative_group_torque()

    assert [command.motor.name for command in commands] == ["dmh6215_left", "dmh6215_right"]
    assert [command.torque for command in commands] == [DEFAULT_TEST_TORQUE, DEFAULT_TEST_TORQUE]
