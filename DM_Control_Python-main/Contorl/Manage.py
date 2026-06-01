from __future__ import annotations

# runpy 用来安全读取本文件里最新保存的参数，读取失败时不影响正在运行的程序。
import runpy

# dataclass 用来保存一次热更新后的参数快照。
from dataclasses import dataclass

# Path 用来定位当前 Manage.py 文件。
from pathlib import Path

# time 用来控制检查文件变化的频率，避免每个 1ms 循环都读磁盘。
import time


# =========================
# 运行时可调参数区
# 运行 main.py 时，直接修改下面这些值并保存文件即可生效。
# =========================

# BALANCE 模式下 pitch 轴的比例增益，单位 Nm/deg。
BALANCE_PITCH_KP = 0.16

# BALANCE 模式下 pitch 轴角速度阻尼，单位 Nm/(deg/s)。
BALANCE_PITCH_KD = 0.012

# BALANCE 模式下 0x03 轮毂电机 MIT 速度阻尼。
BALANCE_MOTOR3_KD = 0.02

# BALANCE 模式下 0x04 轮毂电机 MIT 速度阻尼。
BALANCE_MOTOR4_KD = 0.08


@dataclass(frozen=True)
class RuntimeBalanceGains:
    """一次热更新得到的 balance 参数。"""

    # pitch 轴比例增益。
    pitch_kp: float

    # pitch 轴速度阻尼。
    pitch_kd: float

    # 0x03 轮毂电机 MIT 速度阻尼。
    motor3_kd: float

    # 0x04 轮毂电机 MIT 速度阻尼。
    motor4_kd: float


class RuntimeParameterManager:
    """监听 Manage.py 的保存动作，并把新参数写入运行中的状态机。"""

    def __init__(
        self,
        path: str | Path | None = None,
        reload_interval_s: float = 0.20,
    ):
        """初始化参数管理器。"""
        self.path = Path(__file__) if path is None else Path(path)
        self.reload_interval_s = reload_interval_s
        self.last_error: str | None = None
        self._last_check_time = 0.0
        self._last_mtime_ns = 0
        self.balance_gains = RuntimeBalanceGains(
            pitch_kp=float(BALANCE_PITCH_KP),
            pitch_kd=float(BALANCE_PITCH_KD),
            motor3_kd=float(BALANCE_MOTOR3_KD),
            motor4_kd=float(BALANCE_MOTOR4_KD),
        )
        self.refresh(force=True)

    def apply_to_state_machine(self, state_machine, force: bool = False) -> bool:
        """如果参数变化，就写入状态机；返回 True 表示本轮发生了更新。"""
        gains = self.refresh(force=force)
        if gains is None and not force:
            return False

        active_gains = self.balance_gains if gains is None else gains
        state_machine.set_balance_gains(
            pitch_kp=active_gains.pitch_kp,
            pitch_kd=active_gains.pitch_kd,
            motor3_kd=active_gains.motor3_kd,
            motor4_kd=active_gains.motor4_kd,
        )
        return gains is not None or force

    def refresh(self, force: bool = False) -> RuntimeBalanceGains | None:
        """检查 Manage.py 是否被保存过，并在变化时读取新参数。"""
        now = time.monotonic()
        if not force and now - self._last_check_time < self.reload_interval_s:
            return None
        self._last_check_time = now

        try:
            mtime_ns = self.path.stat().st_mtime_ns
        except OSError as exc:
            self.last_error = f"cannot stat {self.path}: {exc}"
            return None

        if not force and mtime_ns == self._last_mtime_ns:
            return None

        try:
            gains = self._load_balance_gains()
        except Exception as exc:
            self.last_error = f"cannot load runtime params: {exc}"
            return None

        self._last_mtime_ns = mtime_ns
        self.last_error = None
        if not force and gains == self.balance_gains:
            return None

        self.balance_gains = gains
        return gains

    def _load_balance_gains(self) -> RuntimeBalanceGains:
        """从磁盘上的 Manage.py 读取最新 Kp/Kd。"""
        namespace = runpy.run_path(str(self.path))
        return RuntimeBalanceGains(
            pitch_kp=float(namespace.get("BALANCE_PITCH_KP", BALANCE_PITCH_KP)),
            pitch_kd=float(namespace.get("BALANCE_PITCH_KD", BALANCE_PITCH_KD)),
            motor3_kd=float(namespace.get("BALANCE_MOTOR3_KD", BALANCE_MOTOR3_KD)),
            motor4_kd=float(namespace.get("BALANCE_MOTOR4_KD", BALANCE_MOTOR4_KD)),
        )
