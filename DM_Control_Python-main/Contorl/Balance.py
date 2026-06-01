from __future__ import annotations

# dataclass 用来把平衡控制参数和输出结果组织成明确的数据对象。
from dataclasses import dataclass


@dataclass
class BalanceConfig:
    """轮腿平衡控制的基础参数。"""

    # 目标 pitch，单位 deg；当前要求机身前后俯仰回到 6。
    target_pitch_deg: float = 2.0

    # 目标 roll，单位 deg；当前要求机身左右横滚回到 0。
    target_roll_deg: float = -1.0

    # 关节目标角，单位 rad；后续给 4310 关节电机做腿部保持用。
    target_q_rad: float = -0.1

    # pitch 到轮电机力矩的比例系数，单位 Nm/deg。
    pitch_torque_kp: float = 2

    # pitch 角速度阻尼系数，单位 Nsm/(deg/s)。
    pitch_rate_torque_kd: float = 0.0

    # 轮电机力矩限幅，防止第一次调试时输出过大。
    max_wheel_torque: float = 0.3


@dataclass
class BalanceOutput:
    """平衡控制器给状态机或 main.py 使用的输出。"""

    # pitch 误差，单位 deg，定义为 pitch - target_pitch。
    pitch_error_deg: float

    # roll 误差，单位 deg，定义为 roll - target_roll。
    roll_error_deg: float

    # 4310 关节电机的目标角，单位 rad。
    target_q_rad: float

    # 0x03 轮电机力矩；pitch 增加时该值为负。
    motor3_tau: float

    # 0x04 轮电机力矩；始终与 0x03 相反。
    motor4_tau: float


class BalanceController:
    """根据 IMU 姿态生成轮电机平衡力矩。"""

    def __init__(self, config: BalanceConfig | None = None):
        """保存平衡控制参数。"""
        self.config = config or BalanceConfig()

    def update(
        self,
        pitch_deg: float,
        roll_deg: float,
        pitch_rate_deg_s: float = 0.0,
    ) -> BalanceOutput:
        """输入当前姿态，输出目标关节角和 0x03/0x04 轮电机力矩。"""
        pitch_error = pitch_deg - self.config.target_pitch_deg
        roll_error = roll_deg - self.config.target_roll_deg

        # 符号约定：pitch 增加时 0x03 必须输出负力矩，所以这里取负号。
        motor3_tau = -(
            self.config.pitch_torque_kp * pitch_error
            + self.config.pitch_rate_torque_kd * pitch_rate_deg_s
        )
        motor3_tau = self._clamp(motor3_tau, self.config.max_wheel_torque)

        return BalanceOutput(
            pitch_error_deg=pitch_error,
            roll_error_deg=roll_error,
            target_q_rad=self.config.target_q_rad,
            motor3_tau=motor3_tau,
            motor4_tau=-motor3_tau,
        )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """把数值限制到 +/-limit 范围内。"""
        abs_limit = abs(limit)
        return max(-abs_limit, min(abs_limit, value))


# 默认平衡控制器，方便 main.py 或状态机直接复用同一套默认参数。
DEFAULT_BALANCE_CONFIG = BalanceConfig()
