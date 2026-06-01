# Legacy independent motor Rich dashboard.
# New robot-level programs should use RobotDashboard from ../robot_state.py.

# Group 用来把状态文本和表格组合成一个面板。
from rich.console import Group

# Live 用来让终端 Rich 面板原地刷新。
from rich.live import Live

# Panel 用来给电机表加边框。
from rich.panel import Panel

# Table 用来显示电机状态行。
from rich.table import Table

# Text 用来显示循环次数和目标位置。
from rich.text import Text


class MotorDashboard:
    """旧版独立电机 Rich 面板。"""

    def __init__(self, motors, refresh_per_second=10):
        """绑定电机列表并创建 Live 面板。"""
        self.motors = motors
        self.refresh_per_second = refresh_per_second
        self.live = Live(
            self._render(iteration=0, target_position=0.0),
            refresh_per_second=refresh_per_second,
            screen=False,
        )

    def __enter__(self):
        """进入 Live 面板上下文。"""
        self.live.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """退出 Live 面板上下文。"""
        return self.live.__exit__(exc_type, exc_val, exc_tb)

    def update(self, iteration, target_position):
        """刷新电机面板。"""
        self.live.update(
            self._render(iteration=iteration, target_position=target_position)
        )

    def _render(self, iteration, target_position):
        """渲染当前电机状态。"""
        status = Text()
        status.append("Loop: ", style="bold")
        status.append(str(iteration))
        status.append("    Target q: ", style="bold")
        status.append(f"{target_position:+.4f} rad")

        table = Table(title="DaMiao Motor Realtime Status")
        table.add_column("Motor", justify="center", style="cyan")
        table.add_column("Type", justify="center")
        table.add_column("Slave ID", justify="center")
        table.add_column("Position(rad)", justify="right")
        table.add_column("Velocity(rad/s)", justify="right")
        table.add_column("Torque(Nm)", justify="right")

        for idx, motor in enumerate(self.motors, start=1):
            table.add_row(
                f"Motor{idx}",
                motor.MotorType.name,
                f"0x{motor.SlaveID:02X}",
                f"{motor.getPosition():+.4f}",
                f"{motor.getVelocity():+.4f}",
                f"{motor.getTorque():+.4f}",
            )

        return Panel(
            Group(status, table),
            title="Motor Dashboard",
            border_style="green",
        )
