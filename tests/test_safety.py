import importlib
import math
from contextlib import contextmanager

motors = importlib.import_module("control.damiao_can.motors")
safety = importlib.import_module("control.damiao_can.safety")

get_motor = motors.get_motor
DEFAULT_TEST_TORQUE = safety.DEFAULT_TEST_TORQUE
MAX_NONZERO_DURATION = safety.MAX_NONZERO_DURATION
MAX_PAIR_MIT_KD = safety.MAX_PAIR_MIT_KD
MAX_PAIR_MIT_TORQUE = safety.MAX_PAIR_MIT_TORQUE
MAX_PAIR_MIT_VELOCITY = safety.MAX_PAIR_MIT_VELOCITY
MAX_PAIR_WHEEL_VELOCITY = safety.MAX_PAIR_WHEEL_VELOCITY
require_torque_motor = safety.require_torque_motor
validate_mit_parameters = safety.validate_mit_parameters
validate_nonzero_duration = safety.validate_nonzero_duration
validate_pair_mit_kd = safety.validate_pair_mit_kd
validate_pair_mit_torque = safety.validate_pair_mit_torque
validate_pair_mit_velocity = safety.validate_pair_mit_velocity
validate_pair_wheel_velocity = safety.validate_pair_wheel_velocity
validate_test_torque = safety.validate_test_torque
validate_zero_duration = safety.validate_zero_duration


def test_default_period_is_500hz():
    assert safety.DEFAULT_PERIOD == 0.002


@contextmanager
def assert_raises(expected: type[BaseException], match: str | None = None):
    try:
        yield
    except expected as exc:
        if match is not None:
            assert match in str(exc)
    else:
        raise AssertionError(f"expected {expected.__name__}")


def test_duration_limits():
    assert validate_nonzero_duration(MAX_NONZERO_DURATION) == MAX_NONZERO_DURATION
    assert validate_zero_duration(1.0) == 1.0
    with assert_raises(ValueError):
        validate_nonzero_duration(MAX_NONZERO_DURATION + 0.1)
    with assert_raises(ValueError):
        validate_nonzero_duration(0)


def test_duration_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        with assert_raises(ValueError, match="finite"):
            validate_nonzero_duration(value)


def test_test_torque_only_allows_torque_mode_and_small_values():
    with assert_raises(ValueError, match="not torque"):
        validate_test_torque(get_motor("dm4310_left"), DEFAULT_TEST_TORQUE)
    with assert_raises(ValueError, match="not torque"):
        validate_test_torque(get_motor("dmh6215_left"), DEFAULT_TEST_TORQUE)


def test_require_torque_motor_rejects_mit_motor():
    with assert_raises(ValueError, match="not torque"):
        require_torque_motor(get_motor("dm4310_left"))


def test_validate_mit_parameters_rejects_non_finite_and_wrong_mode():
    validate_mit_parameters(get_motor("dm4310_left"), position=0, velocity=0, kp=0, kd=0, torque=0)
    with assert_raises(ValueError, match="finite"):
        validate_mit_parameters(get_motor("dm4310_left"), torque=math.nan)
    with assert_raises(ValueError, match="not mit"):
        validate_mit_parameters(get_motor("dmh6215_left"))


def test_pair_mit_velocity_and_kd_limits():
    assert MAX_PAIR_MIT_VELOCITY == 5.0
    assert validate_pair_mit_velocity(5.0) == 5.0
    assert validate_pair_mit_velocity(-5.0) == -5.0
    assert validate_pair_mit_kd(MAX_PAIR_MIT_KD) == MAX_PAIR_MIT_KD
    with assert_raises(ValueError, match="velocity"):
        validate_pair_mit_velocity(5.1)
    with assert_raises(ValueError, match="velocity"):
        validate_pair_mit_velocity(-5.1)
    with assert_raises(ValueError, match="kd"):
        validate_pair_mit_kd(MAX_PAIR_MIT_KD + 0.01)
    with assert_raises(ValueError, match="kd"):
        validate_pair_mit_kd(-0.01)


def test_pair_mit_velocity_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        with assert_raises(ValueError, match="finite"):
            validate_pair_mit_velocity(value)


def test_pair_mit_kd_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        with assert_raises(ValueError, match="finite"):
            validate_pair_mit_kd(value)


def test_pair_mit_torque_limits():
    assert MAX_PAIR_MIT_TORQUE == 1.0
    assert validate_pair_mit_torque(1.0) == 1.0
    assert validate_pair_mit_torque(-1.0) == -1.0
    with assert_raises(ValueError, match="torque"):
        validate_pair_mit_torque(1.1)
    with assert_raises(ValueError, match="torque"):
        validate_pair_mit_torque(-1.1)


def test_pair_mit_torque_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        with assert_raises(ValueError, match="finite"):
            validate_pair_mit_torque(value)


def test_pair_wheel_velocity_limits():
    assert MAX_PAIR_WHEEL_VELOCITY == 5.0
    assert validate_pair_wheel_velocity(5.0) == 5.0
    assert validate_pair_wheel_velocity(-5.0) == -5.0
    with assert_raises(ValueError, match="wheel velocity"):
        validate_pair_wheel_velocity(5.1)
    with assert_raises(ValueError, match="wheel velocity"):
        validate_pair_wheel_velocity(-5.1)


def test_pair_wheel_velocity_rejects_non_finite():
    for value in [math.nan, math.inf, -math.inf]:
        with assert_raises(ValueError, match="finite"):
            validate_pair_wheel_velocity(value)
