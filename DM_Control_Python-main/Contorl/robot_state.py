from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text


@dataclass
class MotorTelemetry:
    """单个电机的统一状态快照。"""

    # 电机显示名称，例如 Motor1。
    name: str

    # 电机型号字符串，例如 DM4310。
    motor_type: str

    # 电机 CAN ID。
    slave_id: int

    # 电机当前位置，单位 rad。
    position_rad: float | None = None

    # 电机当前速度，单位 rad/s。
    velocity_rad_s: float | None = None

    # 电机当前力矩或电流反馈。
    effort: float | None = None

    # effort 字段的单位说明。
    effort_unit: str = "Nm/A"


@dataclass
class IMUTelemetry:
    """IMU 平衡控制所需的姿态和角速度。"""

    # 俯仰角，单位 deg。
    pitch_deg: float | None = None

    # 横滚角，单位 deg。
    roll_deg: float | None = None

    # 俯仰角，单位 rad。
    pitch_rad: float | None = None

    # 横滚角，单位 rad。
    roll_rad: float | None = None

    # 俯仰角速度，单位 rad/s。
    pitch_rate_rad_s: float | None = None

    # 横滚角速度，单位 rad/s。
    roll_rate_rad_s: float | None = None


@dataclass
class RobotState:
    """机器人统一状态对象。"""

    # 用电机名索引的电机状态。
    motors: dict[str, MotorTelemetry] = field(default_factory=dict)

    # 当前 IMU 状态。
    imu: IMUTelemetry = field(default_factory=IMUTelemetry)

    def update_motor(
        self,
        name: str,
        motor_type: str,
        slave_id: int,
        position_rad: float,
        velocity_rad_s: float,
        effort: float,
        effort_unit: str = "Nm/A",
    ) -> None:
        """用显式数值更新某个电机状态。"""
        self.motors[name] = MotorTelemetry(
            name=name,
            motor_type=motor_type,
            slave_id=slave_id,
            position_rad=position_rad,
            velocity_rad_s=velocity_rad_s,
            effort=effort,
            effort_unit=effort_unit,
        )

    def update_motor_from_dm(self, name: str, motor, effort_unit: str = "Nm") -> None:
        """从 DM_CAN Motor 对象中读取位置、速度和力矩。"""
        self.update_motor(
            name=name,
            motor_type=motor.MotorType.name,
            slave_id=motor.SlaveID,
            position_rad=motor.getPosition(),
            velocity_rad_s=motor.getVelocity(),
            effort=motor.getTorque(),
            effort_unit=effort_unit,
        )

    def update_motors_from_dm(
        self, motors: Iterable, prefix: str = "Motor", effort_unit: str = "Nm"
    ) -> None:
        """批量从 DM_CAN Motor 对象列表更新电机状态。"""
        for index, motor in enumerate(motors, start=1):
            self.update_motor_from_dm(f"{prefix}{index}", motor, effort_unit)

    def update_imu(
        self,
        pitch_deg: float,
        roll_deg: float,
        pitch_rate_rad_s: float,
        roll_rate_rad_s: float,
        pitch_rad: float | None = None,
        roll_rad: float | None = None,
    ) -> None:
        """更新 IMU 姿态和角速度。"""
        self.imu = IMUTelemetry(
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            pitch_rad=math.radians(pitch_deg) if pitch_rad is None else pitch_rad,
            roll_rad=math.radians(roll_deg) if roll_rad is None else roll_rad,
            pitch_rate_rad_s=pitch_rate_rad_s,
            roll_rate_rad_s=roll_rate_rad_s,
        )

    def update_imu_from_observation(self, observation) -> None:
        """从 BalanceObservation 更新 IMU 状态。"""
        if observation is None:
            return
        self.update_imu(
            pitch_deg=observation.pitch_deg,
            roll_deg=observation.roll_deg,
            pitch_rad=observation.pitch_rad,
            roll_rad=observation.roll_rad,
            pitch_rate_rad_s=observation.pitch_rate_rad_s,
            roll_rate_rad_s=observation.roll_rate_rad_s,
        )

    def controller_observation_vector(self) -> tuple[float, float, float, float] | None:
        """返回控制器常用向量：pitch、roll、pitch_rate、roll_rate。"""
        imu = self.imu
        if (
            imu.pitch_rad is None
            or imu.roll_rad is None
            or imu.pitch_rate_rad_s is None
            or imu.roll_rate_rad_s is None
        ):
            return None
        return (
            imu.pitch_rad,
            imu.roll_rad,
            imu.pitch_rate_rad_s,
            imu.roll_rate_rad_s,
        )


class RobotDashboard:
    """统一 Rich 面板，同时显示电机、IMU 和安全状态。"""

    def __init__(self, state: RobotState, refresh_per_second: float = 20.0):
        """创建一个绑定到 RobotState 的 Live 面板。"""
        self.state = state
        self.live = Live(
            self._render(loop_count=0),
            refresh_per_second=refresh_per_second,
            screen=False,
        )

    def __enter__(self) -> "RobotDashboard":
        """进入 Rich Live 上下文。"""
        self.live.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出 Rich Live 上下文。"""
        return self.live.__exit__(exc_type, exc_val, exc_tb)

    def update(
        self,
        loop_count: int = 0,
        mode: str | None = None,
        target_position: float | None = None,
        imu_samples: int | None = None,
        csv_path: str | Path | None = None,
        safety_status=None,
    ) -> None:
        """刷新面板内容。"""
        self.live.update(
            self._render(
                loop_count=loop_count,
                mode=mode,
                target_position=target_position,
                imu_samples=imu_samples,
                csv_path=csv_path,
                safety_status=safety_status,
            )
        )

    def _render(
        self,
        loop_count: int,
        mode: str | None = None,
        target_position: float | None = None,
        imu_samples: int | None = None,
        csv_path: str | Path | None = None,
        safety_status=None,
    ) -> Panel:
        """渲染整个机器人状态面板。"""
        status = Text()
        status.append("Loop: ", style="bold")
        status.append(str(loop_count))
        if mode is not None:
            status.append("    Mode: ", style="bold")
            status.append(mode)
        if target_position is not None:
            status.append("    Target q: ", style="bold")
            status.append(f"{target_position:+.4f} rad")
        if imu_samples is not None:
            status.append("    IMU samples: ", style="bold")
            status.append(str(imu_samples))
        if csv_path is not None:
            status.append("    CSV: ", style="bold")
            status.append(str(csv_path))
        if safety_status is not None:
            status.append("    Safety: ", style="bold")
            if safety_status.is_fault:
                status.append(safety_status.mode.value, style="bold red")
                status.append(f" ({safety_status.reason})", style="red")
            else:
                status.append(safety_status.mode.value, style="bold green")
            if safety_status.imu_age_ms is not None:
                status.append("    IMU age: ", style="bold")
                status.append(f"{safety_status.imu_age_ms:.0f} ms")
            if safety_status.motor_age_ms is not None:
                status.append("    Motor age: ", style="bold")
                status.append(f"{safety_status.motor_age_ms:.0f} ms")

        return Panel(
            Group(status, self._motor_table(), self._imu_table()),
            title="Robot State Dashboard",
            border_style="magenta",
        )

    def _motor_table(self) -> Table:
        """渲染电机状态表。"""
        table = Table(title="Motors")
        table.add_column("Motor", style="cyan")
        table.add_column("Type", justify="center")
        table.add_column("Slave ID", justify="center")
        table.add_column("Position(rad)", justify="right")
        table.add_column("Velocity(rad/s)", justify="right")
        table.add_column("Effort", justify="right")

        if not self.state.motors:
            table.add_row("waiting", "-", "-", "-", "-", "-")
            return table

        for motor in self.state.motors.values():
            table.add_row(
                motor.name,
                motor.motor_type,
                f"0x{motor.slave_id:02X}",
                _fmt(motor.position_rad),
                _fmt(motor.velocity_rad_s),
                f"{_fmt(motor.effort)} {motor.effort_unit}",
            )
        return table

    def _imu_table(self) -> Table:
        """渲染 IMU 平衡信号表。"""
        imu = self.state.imu
        table = Table(title="IMU Balance Signals")
        table.add_column("Signal", style="cyan")
        table.add_column("Value", justify="right")
        table.add_column("Unit")
        table.add_row("pitch", _fmt(imu.pitch_deg), "deg")
        table.add_row("roll", _fmt(imu.roll_deg), "deg")
        table.add_row("pitch_rate", _fmt(imu.pitch_rate_rad_s, precision=6), "rad/s")
        table.add_row("roll_rate", _fmt(imu.roll_rate_rad_s, precision=6), "rad/s")
        return table


def _fmt(value: float | None, precision: int = 4) -> str:
    """统一格式化浮点数和空值。"""
    if value is None:
        return "waiting"
    return f"{value:+.{precision}f}"
