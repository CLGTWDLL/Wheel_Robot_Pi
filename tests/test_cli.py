import argparse
import importlib

from control.damiao_can.motors import get_motor
from control.damiao_can.protocol import disable_frame, enable_frame, standard_disable_frame


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


def test_group_zero_telemetry_prints_rx_and_sends_zero_frames(monkeypatch, capsys):
    bus_module = importlib.import_module("control.bus")
    fake_bus_module = importlib.import_module("control.fake_bus")
    protocol_module = importlib.import_module("control.damiao_can.protocol")
    commands_module = importlib.import_module("control.damiao_can.commands")
    class RecordingFakeCanBus(fake_bus_module.FakeCanBus):
        def __init__(self, rx_frames):
            super().__init__(rx_frames)
            self.recv_timeouts = []

        def recv(self, timeout=None):
            self.recv_timeouts.append(timeout)
            return super().recv(timeout=timeout)

    bus = RecordingFakeCanBus(
        [bus_module.ReceivedFrame(protocol_module.CanFrame(0x12, b"\x02\x80\x00\x80\x07\xFF\x20\x21"), timestamp=56.75)]
    )

    def make_fake_bus(args):
        assert args.backend == "fake"
        return bus

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert _main(["group", "--backend", "fake", "zero", "--duration", "0.001", "--period", "0.001", "--telemetry"]) == 0

    assert bus.sent_frames[:4] == list(commands_module.group_zero_frames())
    assert len(bus.recv_timeouts) >= 2
    assert 0 < bus.recv_timeouts[0] <= 0.001
    assert bus.recv_timeouts[1] == 0
    out = capsys.readouterr().out
    assert "rx id=0x12 data=0280008007ff2021 detected=dm4310_left" in out
    assert "can_id=0x02 master_id=0x12 motor_id=0x02" in out
    assert "position=" in out
    assert "velocity=" in out
    assert "torque=" in out


def test_fake_group_conservative_test_runs(capsys):
    assert _main(["group", "--backend", "fake", "conservative-test", "--duration", "0.001", "--period", "0.001"]) == 0
    out = capsys.readouterr().out
    assert "conservative test" in out


def test_fake_pair_conservative_spin_runs(capsys):
    assert (
        _main(
            [
                "pair",
                "conservative-spin",
                "--backend",
                "fake",
                "--duration",
                "0.001",
                "--period",
                "0.001",
                "--velocity",
                "0.2",
                "--wheel-velocity",
                "0.1",
                "--torque",
                "0.2",
            ]
        )
        == 0
    )
    out = capsys.readouterr().out
    assert "targets 0x02 and 0x04 only" in out
    assert "velocity=0.2 wheel_velocity=0.1 torque=0.2 duration=0.001 period=0.001" in out
    assert "zero" in out
    assert "disables 0x02 and 0x04" in out


def test_pair_conservative_spin_telemetry_prints_rx_and_keeps_safe_finish(monkeypatch, capsys):
    bus_module = importlib.import_module("control.bus")
    fake_bus_module = importlib.import_module("control.fake_bus")
    protocol_module = importlib.import_module("control.damiao_can.protocol")
    commands_module = importlib.import_module("control.damiao_can.commands")
    bus = fake_bus_module.FakeCanBus(
        [bus_module.ReceivedFrame(protocol_module.CanFrame(0x14, b"\x04\x80\x00\x80\x07\xFF\x22\x23"), timestamp=78.5)]
    )

    def make_fake_bus(args):
        assert args.backend == "fake"
        return bus

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert (
        _main(
            [
                "pair",
                "conservative-spin",
                "--backend",
                "fake",
                "--duration",
                "0.001",
                "--period",
                "0.001",
                "--velocity",
                "0.2",
                "--wheel-velocity",
                "0.1",
                "--telemetry",
            ]
        )
        == 0
    )

    assert bus.sent_frames[:2] == list(commands_module.pair_zero_frames())
    assert bus.sent_frames[-4:-2] == list(commands_module.pair_zero_frames())
    assert bus.sent_frames[-2:] == [
        protocol_module.disable_frame(get_motor("dm4310_left")),
        protocol_module.disable_frame(get_motor("dmh6215_left")),
    ]
    out = capsys.readouterr().out
    assert "rx id=0x14 data=0480008007ff2223 detected=dmh6215_left" in out
    assert "can_id=0x04 master_id=0x14 motor_id=0x04" in out
    assert "position=" in out
    assert "velocity=" in out
    assert "torque=" in out
    assert "finish path sends pair zero, then disables 0x02 and 0x04" in out


def test_fake_imu_read_runs(capsys):
    assert _main(["imu", "--backend", "fake", "read", "--duration", "0.001"]) == 0
    out = capsys.readouterr().out
    assert "dm_imu" in out


def test_scan_motors_sends_disable_frames_and_prints_decoded_and_raw(monkeypatch, capsys):
    bus_module = importlib.import_module("control.bus")
    fake_bus_module = importlib.import_module("control.fake_bus")
    protocol_module = importlib.import_module("control.damiao_can.protocol")
    bus = fake_bus_module.FakeCanBus(
        [
            bus_module.ReceivedFrame(protocol_module.CanFrame(0x12, b"\x02\x80\x00\x80\x07\xFF\x20\x21")),
            bus_module.ReceivedFrame(protocol_module.CanFrame(0x14, b"\x04\x80\x00\x80\x07\xFF\x22\x23")),
            bus_module.ReceivedFrame(protocol_module.CanFrame(0x99, b"\x09\x01\x02\x03\x04\x05\x06\x07")),
        ]
    )

    def make_fake_bus(args):
        assert args.backend == "fake"
        return bus

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert _main(["scan", "motors", "--backend", "fake", "--start-id", "0x01", "--end-id", "0x04", "--timeout", "0.001"]) == 0

    assert bus.sent_frames == [standard_disable_frame(can_id) for can_id in range(0x01, 0x05)]
    out = capsys.readouterr().out
    assert "probe tx id=0x01 disable" in out
    assert "probe tx id=0x04 disable" in out
    assert "rx id=0x12 data=0280008007ff2021 detected=dm4310_left" in out
    assert "can_id=0x02 master_id=0x12 motor_id=0x02" in out
    assert "rx id=0x14 data=0480008007ff2223 detected=dmh6215_left" in out
    assert "can_id=0x04 master_id=0x14 motor_id=0x04" in out
    assert "rx id=0x99 data=0901020304050607 motor_id=0x09 raw" in out


def test_sniff_prints_config_and_received_frame_details(monkeypatch, capsys):
    def make_fake_bus(args):
        assert args.backend == "fake"
        return _fake_frame(0x123, b"\x01\x02\x03\x04\x05\x06\x07\x08", timestamp=12.5)

    monkeypatch.setattr("control.can_tool._make_bus", make_fake_bus)

    assert _main(["sniff", "--backend", "fake", "--duration", "0.01"]) == 0
    out = capsys.readouterr().out
    assert "config backend=fake" in out
    assert "serial_baud=921600" in out
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


def test_make_bus_uses_default_dm_usbcan_port(monkeypatch):
    can_tool = importlib.import_module("control.can_tool")
    captured = {}

    class FakeDamiaoUsbCanBus:
        def __init__(self, port, *, serial_baud, can_bitrate):
            captured["port"] = port
            captured["serial_baud"] = serial_baud
            captured["can_bitrate"] = can_bitrate

    monkeypatch.setattr(
        "control.damiao_usbcan.DamiaoUsbCanBus",
        FakeDamiaoUsbCanBus,
    )

    args = argparse.Namespace(backend="dm_usbcan", serial_baud=921600, can_bitrate=1000000)

    assert isinstance(can_tool._make_bus(args), FakeDamiaoUsbCanBus)
    assert captured == {
        "port": "/dev/cu.usbmodem00000000050C1",
        "serial_baud": 921600,
        "can_bitrate": 1000000,
    }


def test_nonexistent_dm_usbcan_port_fails_clearly(capsys):
    assert _main(["sniff", "--backend", "dm_usbcan", "--port", "/dev/nonexistent", "--duration", "0.001"]) == 1
    err = capsys.readouterr().err
    assert "failed to open Damiao USB-CAN port /dev/nonexistent" in err
    assert "Traceback" not in err


def test_motor_torque_rejects_mit_motor(capsys):
    assert _main(["motor", "torque", "--backend", "fake", "--motor", "dm4310_left", "--duration", "0.001"]) == 1
    err = capsys.readouterr().err
    assert "not torque" in err
