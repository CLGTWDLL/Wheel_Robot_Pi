import importlib

from control.damiao_can.motors import get_motor
from control.damiao_can.protocol import disable_frame, enable_frame


def _main(argv):
    return importlib.import_module("control.can_tool").main(argv)


def _fake_frame(arbitration_id, data, timestamp):
    bus_module = importlib.import_module("control.bus")
    fake_bus_module = importlib.import_module("control.fake_bus")
    protocol_module = importlib.import_module("control.damiao_can.protocol")
    return fake_bus_module.FakeCanBus(
        [bus_module.ReceivedFrame(protocol_module.CanFrame(arbitration_id, data), timestamp=timestamp)]
    )


def test_list_ports_runs(capsys):
    assert _main(["list-ports"]) == 0
    capsys.readouterr()


def test_fake_motor_enable_sends_enable_then_disable(monkeypatch, capsys):
    bus = importlib.import_module("control.fake_bus").FakeCanBus()

    def make_fake_bus(args):
        assert args.backend == "fake"
        return bus

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)
    monkeypatch.setattr("control.can_tool.time.sleep", lambda duration: duration is None)

    assert _main(["motor", "enable", "--backend", "fake", "--motor", "dm4310_left", "--duration", "0.001"]) == 0

    motor = get_motor("dm4310_left")
    assert bus.sent_frames == [enable_frame(motor), disable_frame(motor)]
    out = capsys.readouterr().out
    assert "then disable" in out


def test_fake_motor_enable_rejects_long_duration(capsys):
    assert _main(["motor", "enable", "--backend", "fake", "--motor", "dm4310_left", "--duration", "0.501"]) == 1
    err = capsys.readouterr().err
    assert "enable duration" in err


def test_fake_group_zero_runs(capsys):
    assert _main(["group", "--backend", "fake", "zero", "--duration", "0.001", "--period", "0.001"]) == 0
    out = capsys.readouterr().out
    assert "does not set zero position" in out


def test_fake_group_conservative_test_runs(capsys):
    assert _main(["group", "--backend", "fake", "conservative-test", "--duration", "0.001", "--period", "0.001"]) == 0
    out = capsys.readouterr().out
    assert "conservative torque" in out


def test_fake_imu_read_runs(capsys):
    assert _main(["imu", "--backend", "fake", "read", "--duration", "0.001"]) == 0
    out = capsys.readouterr().out
    assert "dm_imu" in out


def test_sniff_prints_config_and_received_frame_details(monkeypatch, capsys):
    def make_fake_bus(args):
        assert args.backend == "fake"
        return _fake_frame(0x123, b"\x01\x02\x03\x04\x05\x06\x07\x08", timestamp=12.5)

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert _main(["sniff", "--backend", "fake", "--duration", "0.01"]) == 0
    out = capsys.readouterr().out
    assert "config backend=fake" in out
    assert "serial_baud=115200" in out
    assert "can_bitrate=1000000" in out
    assert "rx timestamp=12.500000 id=0x123 dlc=8 data=0102030405060708" in out


def test_imu_read_prints_received_frame_details(monkeypatch, capsys):
    def make_fake_bus(args):
        assert args.duration == 0.01
        return _fake_frame(0x011, b"\x08\x07\x06\x05\x04\x03\x02\x01", timestamp=34.25)

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert _main(["imu", "--backend", "fake", "read", "--duration", "0.01"]) == 0
    out = capsys.readouterr().out
    assert "dm_imu" in out
    assert "rx timestamp=34.250000 id=0x011 dlc=8 data=0807060504030201" in out


def test_nonexistent_slcan_port_fails_clearly(capsys):
    assert _main(["sniff", "--backend", "slcan", "--port", "/dev/nonexistent", "--duration", "0.001"]) == 1
    err = capsys.readouterr().err
    assert "failed to open SLCAN port /dev/nonexistent" in err
    assert "Traceback" not in err


def test_motor_torque_rejects_mit_motor(capsys):
    assert _main(["motor", "torque", "--backend", "fake", "--motor", "dm4310_left", "--duration", "0.001"]) == 1
    err = capsys.readouterr().err
    assert "not torque" in err
