from __future__ import annotations

# argparse 用来提供端口、幅值、安全阈值等命令行配置。
import argparse

# sys/pathlib 用来把 MOTORS 文件夹加入导入路径。
import sys
from pathlib import Path

# time 用来控制循环节拍、刷新 Rich 面板和运行时长。
import time

# serial 用来创建达妙 USB-CAN 串口对象。
import serial

# main.py lives in Contorl; the project root is one level above it.
CONTROL_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CONTROL_DIR.parent

# These module directories live under the project root, not under Contorl.
MOTORS_DIR = PROJECT_ROOT / "MOTORS"
STATE_DIR = PROJECT_ROOT / "Robor_State"
SAFE_DIR = PROJECT_ROOT / "Safe"

# Allow direct execution of Contorl/main.py from VS Code or PowerShell.
for import_dir in (CONTROL_DIR, PROJECT_ROOT, MOTORS_DIR, STATE_DIR, SAFE_DIR):
    if str(import_dir) not in sys.path:
        sys.path.insert(0, str(import_dir))

# 达妙电机对象、型号、控制模式和控制器来自官方驱动。
from DM_CAN import Control_Type, DM_Motor_Type, Motor, MotorControl  # noqa: E402

# RobotStateMachine 根据当前模式生成四个电机的 MIT 命令。
from State import (  # noqa: E402
    DEFAULT_STATE_CONFIG,
    RobotMode,
    RobotStateMachine,
    StateMachineConfig,
)

# BalanceSignalProcessor 把 IMU 原始姿态转换成控制器需要的 pitch/roll 信号。
from IMU.balance_signal import BalanceSignalProcessor  # noqa: E402

# DMIMUSerialReader 负责读取 COM8 上的 IMU 数据。
from IMU.imu_can import DMIMUSerialReader  # noqa: E402

# RuntimeParameterManager 负责热更新 Balance.py 里的平衡参数。
from Balance import RuntimeParameterManager  # noqa: E402

# RobotState 保存统一机器人状态；RobotDashboard 负责总 Rich 面板。
from robot_state import RobotDashboard, RobotState  # noqa: E402

# SafetyMonitor 做安全判断；MotorSafetyActuator 执行零力矩和失能。
from safe import MotorSafetyActuator, SafetyConfig, SafetyMonitor  # noqa: E402


def build_motors() -> list[Motor]:
    """创建当前机器人上的 4 个达妙电机对象。"""
    return [
        Motor(DM_Motor_Type.DM4310, 0x01, 0x11),
        Motor(DM_Motor_Type.DM4310, 0x02, 0x12),
        Motor(DM_Motor_Type.DMH6215, 0x03, 0x13),
        Motor(DM_Motor_Type.DMH6215, 0x04, 0x14),
    ]


def open_motor_control(args: argparse.Namespace, motors: list[Motor]) -> MotorControl:
    """打开电机串口、注册电机，并切换到 MIT 控制模式。"""
    serial_device = serial.Serial(args.motor_port, args.motor_baudrate, timeout=0.5)
    motor_control = MotorControl(serial_device)

    # 4310 关节电机已经在官方助手里把 PMAX 设置成 +/-0.25。
    motor_control.change_limit_param(DM_Motor_Type.DM4310, 0.25, 30, 10)

    # 所有电机进入同一个控制器对象，后续才能正常接收反馈。
    for motor in motors:
        motor_control.addMotor(motor)
        motor_control.switchControlMode(motor, Control_Type.MIT)

    return motor_control


def enable_motors(motor_control: MotorControl, motors: list[Motor]) -> None:
    """逐个使能电机。"""
    for motor in motors:
        motor_control.enable(motor)


def send_motor_commands(
    motor_control: MotorControl,
    motors: list[Motor],
    state_machine: RobotStateMachine,
    safety: SafetyMonitor,
) -> None:
    """根据状态机输出发送 MIT 电机命令。"""
    commands = state_machine.motor_commands(len(motors))
    for motor, command in zip(motors, commands):
        motor_control.controlMIT(
            motor,
            command.kp,
            command.kd,
            command.q,
            command.dq,
            safety.clamp_torque(command.tau),
        )


def disable_motors(motor_control: MotorControl, motors: list[Motor]) -> None:
    """退出程序或故障后逐个失能电机。"""
    for motor in motors:
        try:
            motor_control.disable(motor)
        except Exception:
            pass


def main() -> None:
    """运行 IMU、电机、安全管理和总 Rich 面板。"""
    args = parse_args()

    # 电机对象只描述 ID 和型号，不会自己发送命令。
    motors = build_motors()

    # 电机控制器负责真正的串口通信。
    motor_control = open_motor_control(args, motors)

    # 统一状态对象供 Rich 面板和安全模块读取。
    robot_state = RobotState()

    # 状态机负责决定当前模式下四个电机输出什么。
    state_machine = RobotStateMachine(
        StateMachineConfig(
            initial_mode=RobotMode(args.mode),
            leg_hold_position_rad=args.leg_hold_position,
            leg_hold_kp=args.leg_hold_kp,
            leg_hold_kd=args.leg_hold_kd,
            leg_hold_motor1_tau=args.leg_hold_motor1_tau,
            leg_hold_motor2_tau=args.leg_hold_motor2_tau,
        )
    )

    # 参数管理器会监听 Balance.py；保存新 Kp/Kd 后不重启也能生效。
    parameter_manager = RuntimeParameterManager()
    parameter_manager.apply_to_state_machine(state_machine, force=True)

    # IMU 处理器负责生成 pitch/roll 和角速度观测量。
    imu_processor = BalanceSignalProcessor()

    # 安全配置从命令行参数进入，单位统一换成秒。
    safety = SafetyMonitor(
        SafetyConfig(
            imu_timeout_s=args.imu_timeout_ms / 1000.0,
            motor_timeout_s=args.motor_timeout_ms / 1000.0,
            max_abs_pitch_deg=args.max_abs_pitch_deg,
            max_abs_roll_deg=args.max_abs_roll_deg,
            max_abs_torque_command=args.max_abs_torque,
        )
    )

    # 安全执行器只做两件事：零力矩和失能。
    motor_safety = MotorSafetyActuator(motor_control, motors)

    try:
        # IMU 走 COM8 USB 虚拟串口。
        with DMIMUSerialReader(
            port=args.imu_port,
            baudrate=args.imu_baudrate,
            timeout=0.02,
            validate_crc=args.validate_crc,
        ) as imu_reader:
            # 可选：配置 IMU 只输出角速度和欧拉角。
            if args.configure_imu:
                imu_reader.configure_usb_balance_stream(save=args.save_imu_config)

            # 串口和模式准备好后再使能电机。
            enable_motors(motor_control, motors)

            # loop_count 用于显示循环次数。
            loop_count = 0

            # imu_samples 用于显示有效 IMU 观测样本数。
            imu_samples = 0

            # next_refresh 控制 Rich 面板刷新频率。
            next_refresh = 0.0

            # start_time 用于支持 --duration 限时运行。
            start_time = time.time()
            # 总 Rich 面板同时显示电机、IMU 和安全状态。
            with RobotDashboard(
                robot_state, refresh_per_second=args.refresh_hz
            ) as dashboard:
                while args.duration <= 0 or time.time() - start_time < args.duration:
                    # 数字键切换模式：1=IDLE，2=LEG_HOLD，3=FAULT，4=BALANCE。
                    selected_mode = state_machine.poll_keyboard_mode_switch()

                    # 进入 FAULT 后立即执行故障停止动作。
                    if selected_mode == RobotMode.FAULT:
                        motor_safety.fault_stop()
                        break

                    # 记录主循环次数。
                    loop_count += 1

                    # 每隔很短时间检查一次 Balance.py，参数变了就立即写入状态机。
                    parameter_manager.apply_to_state_machine(state_machine)

                    # Use the latest wheel feedback to add velocity damping torque.
                    if len(motors) >= 4:
                        state_machine.update_wheel_velocity_feedback(
                            motor3_velocity_rad_s=motors[2].getVelocity(),
                            motor4_velocity_rad_s=motors[3].getVelocity(),
                        )

                    # 电机命令集中在一个短函数里，便于后续替换控制律。
                    send_motor_commands(motor_control, motors, state_machine, safety)

                    # 将电机反馈同步到统一机器人状态。
                    robot_state.update_motors_from_dm(motors)

                    # 用真实电机反馈时间戳更新电机安全计时器。
                    safety.mark_motor_feedback_from_motors(motors)

                    # 读取本轮 IMU 串口缓存，并更新统一机器人状态。
                    for imu_state in imu_reader.read_states():
                        observation = imu_processor.update(imu_state)
                        if observation is not None:
                            robot_state.update_imu_from_observation(observation)
                            state_machine.update_balance_observation(
                                pitch_deg=observation.pitch_deg,
                                roll_deg=observation.roll_deg,
                                pitch_rate_rad_s=observation.pitch_rate_rad_s,
                            )
                            safety.mark_imu_update()
                            imu_samples += 1

                    # 统一执行安全检查。
                    safety_status = safety.check(robot_state)

                    # 一旦进入 FAULT，先零力矩再失能电机。
                    if safety_status.is_fault:
                        state_machine.enter_fault()
                        motor_safety.fault_stop()

                    # Rich 面板按 refresh_hz 刷新，不跟随 1ms 控制循环硬刷。
                    now = time.time()
                    if now >= next_refresh:
                        dashboard.update(
                            loop_count=loop_count,
                            mode=state_machine.mode.value,
                            target_position=state_machine.target_q,
                            imu_samples=imu_samples,
                            safety_status=safety_status,
                        )
                        next_refresh = now + 1.0 / args.refresh_hz

                    # 故障动作和面板刷新完成后退出主循环。
                    if safety_status.is_fault:
                        break

                    # 主循环目标节拍约 1ms。
                    time.sleep(0.001)
    finally:
        # 无论正常退出还是异常退出，都尝试失能电机并关闭串口。
        disable_motors(motor_control, motors)
        motor_control.serial_.close()


def parse_args() -> argparse.Namespace:
    """解析主程序命令行参数。"""
    # 状态机默认参数是电机控制参数的唯一默认来源。
    state_defaults = DEFAULT_STATE_CONFIG

    parser = argparse.ArgumentParser(
        description="Run robot monitor and motor control.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # 电机 USB-CAN 串口配置。
    parser.add_argument("--motor-port", default="COM3")
    parser.add_argument("--motor-baudrate", type=int, default=921600)

    # IMU USB 虚拟串口配置。
    parser.add_argument("--imu-port", default="COM8")
    parser.add_argument("--imu-baudrate", type=int, default=921600)

    # Rich 面板刷新频率。
    parser.add_argument("--refresh-hz", type=float, default=20.0)

    # duration <= 0 表示一直运行。
    parser.add_argument("--duration", type=float, default=0.0)

    # 主状态机模式。
    parser.add_argument(
        "--mode",
        choices=[mode.value for mode in RobotMode],
        default=state_defaults.initial_mode.value,
    )

    # LEG_HOLD 模式下的关节保持目标角。
    parser.add_argument(
        "--leg-hold-position",
        type=float,
        default=state_defaults.leg_hold_position_rad,
    )

    # LEG_HOLD 模式下的关节 MIT 参数。
    parser.add_argument("--leg-hold-kp", type=float, default=state_defaults.leg_hold_kp)
    parser.add_argument("--leg-hold-kd", type=float, default=state_defaults.leg_hold_kd)
    parser.add_argument(
        "--leg-hold-motor1-tau",
        type=float,
        default=state_defaults.leg_hold_motor1_tau,
    )
    parser.add_argument(
        "--leg-hold-motor2-tau",
        type=float,
        default=state_defaults.leg_hold_motor2_tau,
    )

    # 安全超时阈值，默认 200ms 适合当前 Python/Windows 调试链路。
    parser.add_argument("--imu-timeout-ms", type=float, default=200.0)
    parser.add_argument("--motor-timeout-ms", type=float, default=200.0)

    # 姿态保护阈值。
    parser.add_argument("--max-abs-pitch-deg", type=float, default=25.0)
    parser.add_argument("--max-abs-roll-deg", type=float, default=25.0)

    # MIT 前馈力矩限幅。
    parser.add_argument("--max-abs-torque", type=float, default=0.5)

    # IMU CRC 校验默认关闭，避免因手册 CRC 字节序差异影响调试。
    parser.add_argument("--validate-crc", action="store_true")

    # 可选配置 IMU 输出内容，默认不主动改 IMU 参数。
    parser.add_argument("--configure-imu", action="store_true")
    parser.add_argument("--save-imu-config", action="store_true")

    return parser.parse_args()


if __name__ == "__main__":
    main()
