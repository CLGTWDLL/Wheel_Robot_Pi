import pytest

from control.damiao_can.motors import DEVICES, MOTORS, get_device, get_motor, get_motor_by_master_id


EXPECTED_DEVICES = {
    "dm_imu": {"can_id": 0x01, "master_id": 0x11, "mode": None},
    "dm4310_left": {"can_id": 0x02, "master_id": 0x12, "mode": "mit"},
    "dm4310_right": {"can_id": 0x03, "master_id": 0x13, "mode": "mit"},
    "dmh6215_left": {"can_id": 0x04, "master_id": 0x14, "mode": "velocity"},
    "dmh6215_right": {"can_id": 0x05, "master_id": 0x15, "mode": "velocity"},
}


def test_device_table_matches_hardware_configuration():
    devices_by_name = {device.name: device for device in DEVICES}

    assert set(devices_by_name) == set(EXPECTED_DEVICES)
    for name, expected in EXPECTED_DEVICES.items():
        device = devices_by_name[name]
        assert device.can_id == expected["can_id"]
        assert device.master_id == expected["master_id"]
        assert device.mode == expected["mode"]


def test_motor_table_contains_only_motors_with_expected_configuration():
    motors_by_name = {motor.name: motor for motor in MOTORS}

    assert set(motors_by_name) == set(EXPECTED_DEVICES) - {"dm_imu"}
    for name, expected in EXPECTED_DEVICES.items():
        if name == "dm_imu":
            continue
        motor = motors_by_name[name]
        assert motor.can_id == expected["can_id"]
        assert motor.master_id == expected["master_id"]
        assert motor.mode == expected["mode"]


@pytest.mark.parametrize("name", ["dm4310_left", "dm4310_right", "dmh6215_left", "dmh6215_right"])
def test_get_motor_finds_motors_by_name(name):
    motor = get_motor(name)

    assert motor.name == name
    assert motor.can_id == EXPECTED_DEVICES[name]["can_id"]
    assert motor.master_id == EXPECTED_DEVICES[name]["master_id"]
    assert motor.mode == EXPECTED_DEVICES[name]["mode"]


@pytest.mark.parametrize("name", ["dm4310_left", "dm4310_right", "dmh6215_left", "dmh6215_right"])
def test_get_motor_by_master_id_finds_motors(name):
    expected = EXPECTED_DEVICES[name]

    motor = get_motor_by_master_id(expected["master_id"])

    assert motor.name == name
    assert motor.can_id == expected["can_id"]
    assert motor.mode == expected["mode"]


def test_get_device_finds_imu_without_requiring_get_motor():
    device = get_device("dm_imu")

    assert device.name == "dm_imu"
    assert device.can_id == 0x01
    assert device.master_id == 0x11
    assert device.mode is None

    with pytest.raises(KeyError, match="unknown motor: dm_imu"):
        get_motor("dm_imu")
