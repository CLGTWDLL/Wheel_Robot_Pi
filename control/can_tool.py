from __future__ import annotations

import argparse
import importlib
import math
import sys
import time

MAX_ENABLE_DURATION = 0.5
DEFAULT_ENABLE_DURATION = 0.1
DEFAULT_DM_USBCAN_PORT = "/dev/cu.usbmodem00000000050C1"


def _commands_module():
    return importlib.import_module("control.damiao_can.commands")


def _protocol_module():
    return importlib.import_module("control.damiao_can.protocol")


def _motors_module():
    return importlib.import_module("control.damiao_can.motors")


def _safety_module():
    return importlib.import_module("control.damiao_can.safety")


def _fake_bus_class():
    return importlib.import_module("control.fake_bus").FakeCanBus


def _dm_usbcan_module():
    return importlib.import_module("control.damiao_usbcan")


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    normalized_argv = _normalize_legacy_motor_args(sys.argv[1:] if argv is None else argv)
    args = parser.parse_args(normalized_argv)
    try:
        return _run(args)
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


def _normalize_legacy_motor_args(argv: list[str]) -> list[str]:
    try:
        motor_index = argv.index("motor")
    except ValueError:
        return argv
    if motor_index + 1 >= len(argv):
        return argv
    bus_arg_names = {"--backend", "--port", "--serial-baud", "--can-bitrate"}
    index = motor_index + 1
    while index < len(argv) and argv[index] in bus_arg_names:
        index += 2
    if index + 1 >= len(argv):
        return argv
    if argv[index] in {"enable", "disable", "torque"}:
        return argv
    if argv[index + 1] not in {"enable", "disable", "torque"}:
        return argv
    return [*argv[:index], argv[index + 1], "--motor", argv[index], *argv[index + 2 :]]


def _run(args: argparse.Namespace) -> int:
    if args.command == "list-ports":
        for port in _dm_usbcan_module().list_serial_ports():
            print(port)
        return 0

    if not hasattr(args, "backend"):
        args.backend = "dm_usbcan"
    if not hasattr(args, "port"):
        args.port = None
    if not hasattr(args, "serial_baud"):
        args.serial_baud = 921600
    if not hasattr(args, "can_bitrate"):
        args.can_bitrate = 1000000
    if args.backend == "dm_usbcan" and args.port is None:
        args.port = DEFAULT_DM_USBCAN_PORT

    bus = _make_bus(args)
    try:
        if args.command == "sniff":
            return _run_sniff(args, bus)
        if args.command == "imu":
            return _run_imu(args, bus)
        if args.command == "scan":
            return _run_scan(args, bus)
        if args.command == "group":
            return _run_group(args, bus)
        if args.command == "pair":
            return _run_pair(args, bus)
        if args.command == "motor":
            return _run_motor(args, bus)

        raise ValueError("unsupported command")
    finally:
        bus.close()


def _run_sniff(args: argparse.Namespace, bus) -> int:
    safety = _safety_module()
    _print_bus_config(args)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        received = bus.recv(timeout=min(safety.DEFAULT_PERIOD, max(0.0, deadline - time.monotonic())))
        if received is not None:
            print(_format_received_frame(received))
    return 0


def _run_imu(args: argparse.Namespace, bus) -> int:
    motors = _motors_module()
    safety = _safety_module()
    if args.imu_command == "read":
        imu = motors.get_device("dm_imu")
        print(f"imu {imu.name}: CAN=0x{imu.can_id:02X} MST=0x{imu.master_id:02X} mode={imu.mode}")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            received = bus.recv(timeout=min(safety.DEFAULT_PERIOD, max(0.0, deadline - time.monotonic())))
            if received is not None:
                print(_format_received_frame(received))
        return 0
    raise ValueError(f"unsupported imu command: {args.imu_command}")


def _run_scan(args: argparse.Namespace, bus) -> int:
    protocol = _protocol_module()
    if args.scan_command == "motors":
        _validate_scan_range(args.start_id, args.end_id)
        for can_id in range(args.start_id, args.end_id + 1):
            bus.send(protocol.standard_disable_frame(can_id))
            print(f"probe tx id=0x{can_id:02X} disable")
            deadline = time.monotonic() + args.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                received = bus.recv(timeout=remaining)
                if received is None:
                    break
                print(_format_scan_received_frame(received, protocol))
        return 0
    raise ValueError(f"unsupported scan command: {args.scan_command}")


def _run_group(args: argparse.Namespace, bus) -> int:
    commands = _commands_module()
    protocol = _protocol_module()
    motors = _motors_module()
    safety = _safety_module()

    print(f"group: duration={args.duration} period={args.period} max_test_torque={safety.MAX_TEST_TORQUE}")
    for frame in commands.group_zero_frames():
        print(f"target CAN=0x{frame.arbitration_id:03X}")
    for name in ("dm4310_left", "dm4310_right", "dmh6215_left", "dmh6215_right"):
        motor = motors.get_motor(name)
        print(f"motor {motor.name}: CAN=0x{motor.can_id:02X} MST=0x{motor.master_id:02X} mode={motor.mode}")
        
    if args.group_command == "zero":
        print("group zero sends zero output frames only; it does not set zero position")
        commands.run_group_zero(
            bus,
            duration=args.duration,
            period=args.period,
            telemetry=_make_telemetry_printer(protocol) if args.telemetry else None,
        )
        return 0
    if args.group_command == "conservative-test":
        print(f"conservative test max_duration={safety.MAX_NONZERO_DURATION}")
        commands.run_group_conservative_test(bus, torque=args.torque, duration=args.duration, period=args.period)
        return 0
    raise ValueError(f"unsupported group command: {args.group_command}")


def _run_pair(args: argparse.Namespace, bus) -> int:
    commands = _commands_module()
    protocol = _protocol_module()

    print("pair conservative-spin targets 0x02 and 0x04 only")
    print(
        f"velocity={args.velocity} wheel_velocity={args.wheel_velocity} torque={args.torque} "
        f"duration={args.duration} period={args.period}"
    )
    print("finish path sends pair zero, then disables 0x02 and 0x04")
    commands.run_pair_conservative_spin(
        bus,
        velocity=args.velocity,
        wheel_velocity=args.wheel_velocity,
        torque=args.torque,
        duration=args.duration,
        period=args.period,
        telemetry=_make_telemetry_printer(protocol) if args.telemetry else None,
    )
    return 0


def _run_motor(args: argparse.Namespace, bus) -> int:
    commands = _commands_module()
    motors = _motors_module()
    safety = _safety_module()

    motor = motors.get_motor(args.motor)
    print(f"motor {motor.name}: CAN=0x{motor.can_id:02X} MST=0x{motor.master_id:02X} mode={motor.mode}")
    if args.motor_command == "enable":
        duration = _validate_enable_duration(args.duration)
        print(f"enable smoke test: sending enable for {duration:.3f}s, then disable")
        bus.send(commands.single_motor_command_frame(args.motor, "enable"))
        time.sleep(duration)
        bus.send(commands.single_motor_command_frame(args.motor, "disable"))
        return 0
    if args.motor_command == "disable":
        bus.send(commands.single_motor_command_frame(args.motor, "disable"))
        return 0
    if args.motor_command == "torque":
        print(f"torque={args.torque} max_test_torque={safety.MAX_TEST_TORQUE} duration={args.duration}")
        commands.run_torque_test(bus, args.motor, torque=args.torque, duration=args.duration, period=args.period)
        return 0
    raise ValueError(f"unsupported motor command: {args.motor_command}")


def _make_bus(args: argparse.Namespace):
    backend = getattr(args, "backend", "dm_usbcan")
    if backend == "fake":
        bus_module = importlib.import_module("control.bus")
        protocol = _protocol_module()
        return _fake_bus_class()(
            [
                bus_module.ReceivedFrame(protocol.CanFrame(0x12, b"\x02\x80\x00\x80\x07\xFF\x20\x21")),
                bus_module.ReceivedFrame(protocol.CanFrame(0x14, b"\x04\x80\x00\x80\x07\xFF\x22\x23")),
            ]
        )
    if backend == "dm_usbcan":
        port = getattr(args, "port", None) or DEFAULT_DM_USBCAN_PORT
        return _dm_usbcan_module().DamiaoUsbCanBus(port, serial_baud=args.serial_baud, can_bitrate=args.can_bitrate)
    raise ValueError(f"unsupported backend: {backend}")


def _validate_enable_duration(duration: float) -> float:
    if not math.isfinite(duration):
        raise ValueError("enable duration must be finite")
    if not 0.0 <= duration <= MAX_ENABLE_DURATION:
        raise ValueError(f"enable duration must be within [0.0, {MAX_ENABLE_DURATION}]")
    return duration


def _print_bus_config(args: argparse.Namespace) -> None:
    print(
        f"config backend={args.backend} port={args.port or '-'} "
        f"serial_baud={args.serial_baud} can_bitrate={args.can_bitrate}"
    )


def _make_telemetry_printer(protocol):
    def print_telemetry_frame(received) -> None:
        print(_format_telemetry_received_frame(received, protocol))

    return print_telemetry_frame


def _format_telemetry_received_frame(received, protocol) -> str:
    try:
        protocol.decode_feedback_frame(received.frame)
    except (KeyError, ValueError):
        return _format_received_frame(received)
    return _format_scan_received_frame(received, protocol)


def _format_received_frame(received) -> str:
    frame = received.frame
    timestamp = received.timestamp if received.timestamp is not None else time.monotonic()
    return f"rx timestamp={timestamp:.6f} id=0x{frame.arbitration_id:03X} dlc={len(frame.data)} data={frame.data.hex()}"


def _format_scan_received_frame(received, protocol) -> str:
    frame = received.frame
    prefix = f"rx id=0x{frame.arbitration_id:02X} data={frame.data.hex()}"
    try:
        routed = protocol.decode_feedback_frame(frame)
    except (KeyError, ValueError):
        detected_motor_id = frame.data[0] & 0x0F if len(frame.data) == 8 else None
        if detected_motor_id is None:
            return f"{prefix} raw"
        return f"{prefix} motor_id=0x{detected_motor_id:02X} raw"
    motor = routed.motor
    feedback = routed.feedback
    return (
        f"{prefix} detected={motor.name} can_id=0x{motor.can_id:02X} "
        f"master_id=0x{motor.master_id:02X} motor_id=0x{feedback.motor_id:02X} "
        f"state={feedback.state} position={feedback.position:.6f} "
        f"velocity={feedback.velocity:.6f} torque={feedback.torque:.6f}"
    )


def _validate_scan_range(start_id: int, end_id: int) -> None:
    if not 0x000 <= start_id <= 0x7FF:
        raise ValueError("--start-id must be in standard 11-bit range 0x000..0x7FF")
    if not 0x000 <= end_id <= 0x7FF:
        raise ValueError("--end-id must be in standard 11-bit range 0x000..0x7FF")
    if start_id > end_id:
        raise ValueError("--start-id must be less than or equal to --end-id")


def _parse_int(value: str) -> int:
    return int(value, 0)


def _add_bus_args(parser: argparse.ArgumentParser, *, defaults: bool = True) -> None:
    default_backend = "dm_usbcan" if defaults else argparse.SUPPRESS
    default_port = None if defaults else argparse.SUPPRESS
    default_serial_baud = 921600 if defaults else argparse.SUPPRESS
    default_can_bitrate = 1000000 if defaults else argparse.SUPPRESS
    parser.add_argument("--backend", choices=["fake", "dm_usbcan"], default=default_backend)
    parser.add_argument("--port", default=default_port)
    parser.add_argument("--serial-baud", type=int, default=default_serial_baud)
    parser.add_argument("--can-bitrate", type=int, default=default_can_bitrate)


def _build_parser() -> argparse.ArgumentParser:
    safety = _safety_module()
    parser = argparse.ArgumentParser(prog="can_tool")
    _add_bus_args(parser)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("list-ports")

    sniff = sub.add_parser("sniff")
    _add_bus_args(sniff, defaults=False)
    sniff.add_argument("--duration", type=float, default=1.0)

    scan = sub.add_parser("scan")
    _add_bus_args(scan, defaults=False)
    scan_sub = scan.add_subparsers(dest="scan_command", required=True)
    scan_motors = scan_sub.add_parser("motors")
    _add_bus_args(scan_motors, defaults=False)
    scan_motors.add_argument("--start-id", type=_parse_int, required=True)
    scan_motors.add_argument("--end-id", type=_parse_int, required=True)
    scan_motors.add_argument("--timeout", type=float, default=0.05)

    motor = sub.add_parser("motor")
    _add_bus_args(motor, defaults=False)
    motor_sub = motor.add_subparsers(dest="motor_command", required=True)
    enable = motor_sub.add_parser("enable")
    _add_bus_args(enable, defaults=False)
    enable.add_argument("--motor", required=True)
    enable.add_argument("--duration", type=float, default=DEFAULT_ENABLE_DURATION)
    disable = motor_sub.add_parser("disable")
    _add_bus_args(disable, defaults=False)
    disable.add_argument("--motor", required=True)
    torque = motor_sub.add_parser("torque")
    _add_bus_args(torque, defaults=False)
    torque.add_argument("--motor", required=True)
    torque.add_argument("--torque", type=float, default=safety.DEFAULT_TEST_TORQUE)
    torque.add_argument("--duration", type=float, default=0.3)
    torque.add_argument("--period", type=float, default=safety.DEFAULT_PERIOD)

    group = sub.add_parser("group")
    _add_bus_args(group, defaults=False)
    group_sub = group.add_subparsers(dest="group_command", required=True)
    zero = group_sub.add_parser("zero")
    _add_bus_args(zero, defaults=False)
    zero.add_argument("--duration", type=float, default=1.0)
    zero.add_argument("--period", type=float, default=safety.DEFAULT_PERIOD)
    zero.add_argument("--telemetry", action="store_true")
    conservative = group_sub.add_parser("conservative-test")
    _add_bus_args(conservative, defaults=False)
    conservative.add_argument("--torque", type=float, default=safety.DEFAULT_TEST_TORQUE)
    conservative.add_argument("--duration", type=float, default=0.3)
    conservative.add_argument("--period", type=float, default=safety.DEFAULT_PERIOD)

    pair = sub.add_parser("pair")
    _add_bus_args(pair, defaults=False)
    pair_sub = pair.add_subparsers(dest="pair_command", required=True)
    pair_conservative = pair_sub.add_parser("conservative-spin")
    _add_bus_args(pair_conservative, defaults=False)
    pair_conservative.add_argument("--velocity", type=float, default=safety.DEFAULT_PAIR_MIT_VELOCITY)
    pair_conservative.add_argument("--wheel-velocity", type=float, default=safety.DEFAULT_PAIR_WHEEL_VELOCITY)
    pair_conservative.add_argument("--torque", type=float, default=safety.DEFAULT_PAIR_MIT_TORQUE)
    pair_conservative.add_argument("--duration", type=float, default=0.3)
    pair_conservative.add_argument("--period", type=float, default=safety.DEFAULT_PERIOD)
    pair_conservative.add_argument("--telemetry", action="store_true")

    imu = sub.add_parser("imu")
    _add_bus_args(imu, defaults=False)
    imu_sub = imu.add_subparsers(dest="imu_command", required=True)
    read = imu_sub.add_parser("read")
    _add_bus_args(read, defaults=False)
    read.add_argument("--duration", type=float, default=1.0)

    return parser


if __name__ == "__main__":
    sys.exit(main())
