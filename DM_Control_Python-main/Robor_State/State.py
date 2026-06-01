from __future__ import annotations

# dataclass 用来定义轻量配置和 MIT 输出命令。
from dataclasses import dataclass, field

# Enum 用来约束机器人只能处在有限几个模式里。
from enum import Enum

# degrees 用来把 IMU 的 rad/s 角速度换成 BalanceController 使用的 deg/s。
from math import degrees

# BalanceController 负责根据 pitch/roll 生成轮电机平衡力矩。
from Balance import BalanceConfig, BalanceController, BalanceOutput


class RobotMode(str, Enum):
    """机器人主状态机模式。"""

    # IDLE 只显示数据和记录日志，四个电机都发零输出。
    IDLE = "IDLE"

    # LEG_HOLD 让关节电机保持固定角度，轮子电机零输出。
    LEG_HOLD = "LEG_HOLD"

    # BALANCE 让关节保持目标角，同时用 0x03/0x04 的相反力矩调 pitch。
    BALANCE = "BALANCE"

    # FAULT 是故障保护态，四个电机零输出，主程序可进一步失能。
    FAULT = "FAULT"


@dataclass
class MITCommand:
    """单个电机的一帧 MIT 控制命令。"""

    # 位置刚度。
    kp: float = 0.0

    # 速度阻尼。
    kd: float = 0.0

    # 目标位置，单位 rad。
    q: float = 0.0

    # 目标速度，单位 rad/s。
    dq: float = 0.0

    # 前馈力矩。
    tau: float = 0.0


@dataclass
class StateMachineConfig:
    """状态机配置参数。"""

    # 默认启动模式；当前先保守停在 IDLE。
    initial_mode: RobotMode = RobotMode.IDLE

    # LEG_HOLD 时 4310 关节电机保持的目标角度。
    leg_hold_position_rad: float = 0.1

    # LEG_HOLD 时关节电机 MIT 位置刚度。
    leg_hold_kp: float = 6.0

    # LEG_HOLD 时关节电机 MIT 阻尼。
    leg_hold_kd: float = 0.5

    # LEG_HOLD 时 Motor1 的关节前馈力矩。
    leg_hold_motor1_tau: float = 0.0

    # LEG_HOLD 时 Motor2 的关节前馈力矩。
    leg_hold_motor2_tau: float = 0.0

    # BALANCE 模式下的 pitch/roll 目标和轮电机力矩参数。
    balance_config: BalanceConfig = field(default_factory=BalanceConfig)

    # BALANCE 模式下 0x03 轮毂电机的 MIT 速度阻尼。
    balance_motor3_kd: float = 0.05

    # BALANCE 模式下 0x04 轮毂电机的 MIT 速度阻尼。
    balance_motor4_kd: float = 0.1

    # 轮子和 IDLE 模式使用的零输出命令。
    zero_command: MITCommand = field(default_factory=MITCommand)


# main.py 和状态机共用这一套默认电机参数。
DEFAULT_STATE_CONFIG = StateMachineConfig()


class RobotStateMachine:
    """机器人运行状态机。"""

    def __init__(self, config: StateMachineConfig | None = None):
        """初始化状态机和模式配置。"""
        self.config = config or StateMachineConfig()
        self.mode = self.config.initial_mode
        self.balance_controller = BalanceController(self.config.balance_config)
        self._last_balance_pitch_deg = self.config.balance_config.target_pitch_deg
        self._last_balance_roll_deg = self.config.balance_config.target_roll_deg
        self._last_balance_pitch_rate_rad_s = 0.0
        self.balance_output = self.balance_controller.update(
            pitch_deg=self.config.balance_config.target_pitch_deg,
            roll_deg=self.config.balance_config.target_roll_deg,
        )

    @property
    def target_q(self) -> float | None:
        """返回当前模式下用于显示的目标关节角。"""
        if self.mode == RobotMode.LEG_HOLD:
            return self.config.leg_hold_position_rad
        if self.mode == RobotMode.BALANCE:
            return self.balance_output.target_q_rad
        return None

    def set_mode(self, mode: RobotMode) -> None:
        """切换状态机模式。"""
        self.mode = mode

    def handle_key(self, key: str) -> RobotMode | None:
        """根据按键切换模式：1=IDLE，2=LEG_HOLD，3=FAULT，4=BALANCE。"""
        key_to_mode = {
            "1": RobotMode.IDLE,
            "2": RobotMode.LEG_HOLD,
            "3": RobotMode.FAULT,
            "4": RobotMode.BALANCE,
            "q": RobotMode.FAULT,
            "Q": RobotMode.FAULT,
            "\x1b": RobotMode.FAULT,
            " ": RobotMode.FAULT,
        }
        mode = key_to_mode.get(key)
        if mode is None:
            return None
        self.set_mode(mode)
        return mode

    def poll_keyboard_mode_switch(self) -> RobotMode | None:
        """读取一次键盘输入，并执行状态切换。"""
        try:
            import msvcrt
        except ImportError:
            return None

        if not msvcrt.kbhit():
            return None

        return self.handle_key(msvcrt.getwch())

    def enter_fault(self) -> None:
        """进入故障保护态。"""
        self.mode = RobotMode.FAULT

    def set_balance_gains(
        self,
        pitch_kp: float,
        pitch_kd: float,
        motor3_kd: float | None = None,
        motor4_kd: float | None = None,
    ) -> None:
        """热更新 BALANCE 模式下 pitch 轴和轮毂电机的 Kd。"""
        self.config.balance_config.pitch_torque_kp = float(pitch_kp)
        self.config.balance_config.pitch_rate_torque_kd = float(pitch_kd)
        if motor3_kd is not None:
            self.config.balance_motor3_kd = float(motor3_kd)
        if motor4_kd is not None:
            self.config.balance_motor4_kd = float(motor4_kd)
        self.update_balance_observation(
            pitch_deg=self._last_balance_pitch_deg,
            roll_deg=self._last_balance_roll_deg,
            pitch_rate_rad_s=self._last_balance_pitch_rate_rad_s,
        )

    def update_balance_observation(
        self,
        pitch_deg: float | None,
        roll_deg: float | None,
        pitch_rate_rad_s: float | None = None,
    ) -> BalanceOutput | None:
        """用最新 IMU 姿态刷新 BALANCE 模式的控制输出。"""
        if pitch_deg is None or roll_deg is None:
            return None

        self._last_balance_pitch_deg = pitch_deg
        self._last_balance_roll_deg = roll_deg
        if pitch_rate_rad_s is None:
            self._last_balance_pitch_rate_rad_s = 0.0
        else:
            self._last_balance_pitch_rate_rad_s = pitch_rate_rad_s

        pitch_rate_deg_s = 0.0
        if pitch_rate_rad_s is not None:
            pitch_rate_deg_s = degrees(pitch_rate_rad_s)

        self.balance_output = self.balance_controller.update(
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            pitch_rate_deg_s=pitch_rate_deg_s,
        )
        return self.balance_output

    def motor_commands(self, motor_count: int = 4) -> list[MITCommand]:
        """根据当前模式生成一组电机 MIT 命令。"""
        if self.mode == RobotMode.LEG_HOLD:
            return self._leg_hold_commands(motor_count)
        if self.mode == RobotMode.BALANCE:
            return self._balance_commands(motor_count)
        return self._zero_commands(motor_count)

    def _leg_hold_commands(self, motor_count: int) -> list[MITCommand]:
        """生成 LEG_HOLD 模式下的命令。"""
        commands = self._zero_commands(motor_count)
        self._apply_joint_hold_commands(
            commands,
            target_q=self.config.leg_hold_position_rad,
        )
        return commands

    def _apply_joint_hold_commands(
        self,
        commands: list[MITCommand],
        target_q: float,
    ) -> None:
        """用同一套关节参数填充 0x01/0x02 的 MIT 保持命令。"""
        motor_count = len(commands)
        if motor_count >= 1:
            commands[0] = MITCommand(
                kp=self.config.leg_hold_kp,
                kd=self.config.leg_hold_kd,
                q=target_q,
                dq=0.0,
                tau=self.config.leg_hold_motor1_tau,
            )
        if motor_count >= 2:
            commands[1] = MITCommand(
                kp=self.config.leg_hold_kp,
                kd=self.config.leg_hold_kd,
                q=-target_q,
                dq=0.0,
                tau=self.config.leg_hold_motor2_tau,
            )

    def _balance_commands(self, motor_count: int) -> list[MITCommand]:
        """生成 BALANCE 模式下的关节保持和轮电机平衡命令。"""
        commands = self._zero_commands(motor_count)
        target_q = self.balance_output.target_q_rad
        self._apply_joint_hold_commands(commands, target_q=target_q)

        if motor_count >= 3:
            commands[2] = MITCommand(
                kp=0.0,
                kd=self.config.balance_motor3_kd,
                q=0.0,
                dq=0.0,
                tau=self.balance_output.motor3_tau,
            )
        if motor_count >= 4:
            commands[3] = MITCommand(
                kp=0.0,
                kd=self.config.balance_motor4_kd,
                q=0.0,
                dq=0.0,
                tau=self.balance_output.motor4_tau,
            )
        return commands

    def _zero_commands(self, motor_count: int) -> list[MITCommand]:
        """生成指定数量的零输出命令。"""
        return [MITCommand() for _ in range(motor_count)]
