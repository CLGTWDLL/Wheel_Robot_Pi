from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable


class SafetyMode(str, Enum):
    """安全状态机的两种模式。"""

    # NORMAL 表示允许继续控制。
    NORMAL = "NORMAL"

    # FAULT 表示必须停止输出并进入故障态。
    FAULT = "FAULT"


@dataclass
class SafetyConfig:
    """安全模块的所有阈值配置。"""

    # IMU 数据允许的最大年龄。
    imu_timeout_s: float = 0.20

    # 电机反馈允许的最大年龄。
    motor_timeout_s: float = 0.20

    # 启动阶段宽限时间，避免刚打开串口时误触发。
    startup_grace_s: float = 1.0

    # pitch 绝对值超过该角度即进入 FAULT。
    max_abs_pitch_deg: float = 25.0

    # roll 绝对值超过该角度即进入 FAULT。
    max_abs_roll_deg: float = 25.0

    # 前馈力矩命令的绝对值限幅。
    max_abs_torque_command: float = 0.5

    # 当前 DM_CAN 没有温度反馈，先保留接口。
    max_motor_temperature_c: float | None = None


@dataclass
class SafetyStatus:
    """一次安全检查的结果，用于主循环和 Rich 面板。"""

    # 当前安全模式。
    mode: SafetyMode

    # 进入 FAULT 的原因。
    reason: str = ""

    # 距离上次有效 IMU 数据的时间。
    imu_age_ms: float | None = None

    # 距离上次有效电机反馈的时间。
    motor_age_ms: float | None = None

    # 保留 pitch 快照字段，后续需要时可直接显示。
    pitch_deg: float | None = None

    # 保留 roll 快照字段，后续需要时可直接显示。
    roll_deg: float | None = None

    @property
    def is_fault(self) -> bool:
        """判断当前是否已经进入故障态。"""
        return self.mode == SafetyMode.FAULT


class SafetyMonitor:
    """安全监视器：负责判断是否应该进入 FAULT。"""

    def __init__(self, config: SafetyConfig | None = None):
        """初始化安全阈值和各类数据时间戳。"""
        self.config = config or SafetyConfig()
        self.start_time = time.monotonic()
        self.last_imu_time: float | None = None
        self.last_motor_time: float | None = None
        self.missing_motor_names: list[str] = []
        self.mode = SafetyMode.NORMAL
        self.reason = ""

    def mark_imu_update(self, now: float | None = None) -> None:
        """记录一次有效 IMU 数据到达时间。"""
        self.last_imu_time = time.monotonic() if now is None else now

    def mark_motor_update(self, now: float | None = None) -> None:
        """手动记录一次电机反馈时间。"""
        self.last_motor_time = time.monotonic() if now is None else now

    def mark_motor_feedback_from_motors(self, motors: Iterable) -> None:
        """从电机对象的真实反馈时间戳中提取最慢反馈时间。"""
        motors = list(motors)
        self.missing_motor_names = []
        feedback_times = [
            motor.last_feedback_time
            for motor in motors
            if getattr(motor, "last_feedback_time", None) is not None
        ]
        for index, motor in enumerate(motors, start=1):
            if getattr(motor, "last_feedback_time", None) is None:
                self.missing_motor_names.append(f"Motor{index}")
        if feedback_times:
            self.last_motor_time = min(feedback_times)

    def clamp_torque(self, torque: float) -> float:
        """将力矩命令裁剪到安全限幅内。"""
        limit = abs(self.config.max_abs_torque_command)
        return max(-limit, min(limit, torque))

    def request_fault(self, reason: str) -> SafetyStatus:
        """主动进入故障态，并记录故障原因。"""
        self.mode = SafetyMode.FAULT
        self.reason = reason
        return self.status()

    def check(self, robot_state, now: float | None = None) -> SafetyStatus:
        """检查超时、姿态角和温度等安全条件。"""
        now = time.monotonic() if now is None else now
        if self.mode == SafetyMode.FAULT:
            return self.status(now)

        imu = robot_state.imu
        # pitch 保护当前调试阶段先关闭；需要恢复时取消下面三行注释。
        # if imu.pitch_deg is not None and abs(imu.pitch_deg) > self.config.max_abs_pitch_deg:
        #     return self.request_fault(
        #         f"pitch too large: {imu.pitch_deg:+.2f} deg"
        #     )
        if imu.roll_deg is not None and abs(imu.roll_deg) > self.config.max_abs_roll_deg:
            return self.request_fault(f"roll too large: {imu.roll_deg:+.2f} deg")

        temperature_fault = self._check_motor_temperatures(robot_state)
        if temperature_fault is not None:
            return self.request_fault(temperature_fault)

        if now - self.start_time >= self.config.startup_grace_s:
            if self.last_imu_time is None:
                return self.request_fault("IMU data missing")
            if self.config.motor_timeout_s > 0:
                if self.missing_motor_names:
                    return self.request_fault(
                        "motor feedback missing: "
                        + ", ".join(self.missing_motor_names)
                    )
                if self.last_motor_time is None:
                    return self.request_fault("motor feedback missing")

            imu_age_s = now - self.last_imu_time
            if imu_age_s > self.config.imu_timeout_s:
                return self.request_fault(
                    f"IMU timeout: {imu_age_s * 1000.0:.0f} ms"
                )
            if self.config.motor_timeout_s > 0:
                motor_age_s = now - self.last_motor_time
                if motor_age_s > self.config.motor_timeout_s:
                    return self.request_fault(
                        f"motor timeout: {motor_age_s * 1000.0:.0f} ms"
                    )

        return self.status(now)

    def poll_keyboard_emergency_stop(self) -> SafetyStatus | None:
        """检查键盘急停键：q、Esc 或空格。"""
        try:
            import msvcrt
        except ImportError:
            return None

        if not msvcrt.kbhit():
            return None

        key = msvcrt.getwch()
        if key in ("q", "Q", "\x1b", " "):
            return self.request_fault("emergency stop key pressed")
        return None

    def status(self, now: float | None = None) -> SafetyStatus:
        """生成当前安全状态快照。"""
        now = time.monotonic() if now is None else now
        return SafetyStatus(
            mode=self.mode,
            reason=self.reason,
            imu_age_ms=self._age_ms(self.last_imu_time, now),
            motor_age_ms=self._age_ms(self.last_motor_time, now),
        )

    def _check_motor_temperatures(self, robot_state) -> str | None:
        """检查电机温度；当前项目暂未解析温度，通常返回 None。"""
        if self.config.max_motor_temperature_c is None:
            return None

        for motor in robot_state.motors.values():
            temperature = getattr(motor, "temperature_c", None)
            if (
                temperature is not None
                and temperature > self.config.max_motor_temperature_c
            ):
                return (
                    f"{motor.name} temperature too high: "
                    f"{temperature:.1f} C"
                )
        return None

    @staticmethod
    def _age_ms(last_time: float | None, now: float) -> float | None:
        """把最近更新时间转换为毫秒年龄。"""
        if last_time is None:
            return None
        return (now - last_time) * 1000.0


class MotorSafetyActuator:
    """故障发生时实际操作电机的执行器。"""

    def __init__(self, motor_control, motors: Iterable):
        """保存电机控制器和电机列表。"""
        self.motor_control = motor_control
        self.motors = list(motors)

    def zero_torque(self, cycles: int = 3) -> None:
        """发送几轮零刚度、零速度、零力矩命令。"""
        for _ in range(cycles):
            for motor in self.motors:
                try:
                    self.motor_control.controlMIT(
                        motor,
                        0,
                        0,
                        motor.getPosition(),
                        0,
                        0,
                    )
                except Exception:
                    pass

    def disable(self) -> None:
        """逐个失能电机。"""
        for motor in self.motors:
            try:
                self.motor_control.disable(motor)
            except Exception:
                    pass

    def fault_stop(self) -> None:
        """故障停止动作：先零力矩，再失能。"""
        self.zero_torque()
        self.disable()
