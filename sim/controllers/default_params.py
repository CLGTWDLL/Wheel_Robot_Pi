"""控制器参数预设与序列化工具。

所有控制器参数的唯一真值源。launch_mujoco.py、测试文件和 optimize.py
均从此模块引用参数，禁止在其他位置硬编码。
"""
from __future__ import annotations

import json
from typing import Any

import numpy as np

from sim.controllers.combined import CombinedParams
from sim.controllers.phase import JumpPhaseParams
from sim.controllers.vmc import VmcParams


# --- 参数预设 ---

STAND_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.142,  # mid of LUT range [0.134, 0.149]
        kp_motor=30.0,
        kd_motor=1.0,
        crouch_offset=-0.02,
        crouch_target_height=0.09,
        crouch_duration=0.20,
        extend_offset=0.02,
        landing_damping=1.0,
        land_kp_scale=0.5,
        land_kd_scale=2.5,
        gravity_compensation_scale=1.0,
        max_height_rate=0.05,
        roll_level_kp_height=0.0,
        roll_level_kd_height=0.002,
        roll_level_offset_limit=0.010,
    ),
    # 2026-05-16 retune after mass rebalance (10.24 kg → 2.20 kg).
    # B matrix scaled ~7× because B ∝ 1/(M·L) and CoM dropped 36 mm.
    # 2026-05-17 重构到 5D LQR:state 为 [pitch, pitch_rate, roll, roll_rate, wheel_vel],
    # 不再包含 wheel_pos。原 Q[4]=50 (wheel_pos) 已删除,原 Q[5]=500 (wheel_vel) 平移到 Q[4]。
    # 2026-05-18 删除 pitch_trim (改用 equilibrium_pitch_from_geometry 自动计算),
    # 加入位置外环参数 position_kp / position_velocity_limit。
    # 2026-05-21 相位独占重构: LQR 在跳跃全程不输出 (CROUCH/EXTEND/FLIGHT/LAND),
    # VMC 独占腿;轮力矩 = 0。删除 flight_wheel_q、jump_extend_feedforward (轨迹动态 FF 取代)。
    q_diag=np.array([1000.0, 200.0, 1000.0, 200.0, 500.0]),
    r_diag=np.array([200.0, 400.0]),
    velocity_ki=0.3,
    pitch_lean_gain=0.15,
    position_kp=3.0,
    position_kd=1.5,
    position_velocity_limit=0.6,
    yaw_damping=0.5,
    yaw_ki=0.0,
    fixed_height=False,
    lqr_height_bin_size=0.02,
    ff_gain=4.0,
)

# 2026-05-16: same gains as STAND_PARAMS; drive validated up to 0.5 m/s in
# headless. Old Optuna-tuned STAND_THEN_DRIVE_PARAMS (10.24 kg model) replaced.
STAND_THEN_DRIVE_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.142,
        kp_motor=30.0,
        kd_motor=1.0,
        crouch_offset=0.02,
        crouch_target_height=0.09,
        crouch_duration=0.20,
        extend_offset=0.03,
        landing_damping=0.628,
        land_kp_scale=0.5,
        land_kd_scale=2.5,
        roll_level_kp_height=0.0,
        roll_level_kd_height=0.002,
        roll_level_offset_limit=0.010,
    ),
    q_diag=np.array([1000.0, 200.0, 1000.0, 200.0, 500.0]),
    r_diag=np.array([200.0, 400.0]),
    pitch_lean_gain=0.15,
    velocity_ki=0.3,
    position_kp=3.0,
    position_kd=1.5,
    position_velocity_limit=0.6,
    yaw_damping=0.5,
    yaw_ki=0.0,
)

# 跳跃相位机默认参数
DEFAULT_PHASE_PARAMS = JumpPhaseParams()


# --- 序列化工具 ---

def params_to_dict(params: CombinedParams) -> dict[str, Any]:
    """将 CombinedParams 序列化为可 JSON 存储的字典。"""
    return {
        "vmc": {
            "nominal_height": params.vmc.nominal_height,
            "kp_motor": params.vmc.kp_motor,
            "kd_motor": params.vmc.kd_motor,
            "crouch_offset": params.vmc.crouch_offset,
            "crouch_target_height": params.vmc.crouch_target_height,
            "crouch_duration": params.vmc.crouch_duration,
            "extend_offset": params.vmc.extend_offset,
            "landing_damping": params.vmc.landing_damping,
            "land_kp_scale": params.vmc.land_kp_scale,
            "land_kd_scale": params.vmc.land_kd_scale,
            "gravity_compensation_scale": params.vmc.gravity_compensation_scale,
            "max_height_rate": params.vmc.max_height_rate,
            "roll_level_kp_height": params.vmc.roll_level_kp_height,
            "roll_level_kd_height": params.vmc.roll_level_kd_height,
            "roll_level_offset_limit": params.vmc.roll_level_offset_limit,
        },
        "q_diag": params.q_diag.tolist(),
        "r_diag": params.r_diag.tolist(),
        "target_velocity": params.target_velocity,
        "pitch_lean_gain": params.pitch_lean_gain,
        "velocity_ki": params.velocity_ki,
        "position_kp": params.position_kp,
        "position_kd": params.position_kd,
        "position_velocity_limit": params.position_velocity_limit,
        "yaw_damping": params.yaw_damping,
        "yaw_ki": params.yaw_ki,
        "target_yaw_rate": params.target_yaw_rate,
        "fixed_height": params.fixed_height,
        "lqr_height_bin_size": params.lqr_height_bin_size,
        "ff_gain": params.ff_gain,
    }


def params_from_dict(d: dict[str, Any]) -> CombinedParams:
    """从字典反序列化 CombinedParams。"""
    vmc_data = d["vmc"]
    return CombinedParams(
        vmc=VmcParams(
            nominal_height=vmc_data["nominal_height"],
            kp_motor=vmc_data["kp_motor"],
            kd_motor=vmc_data["kd_motor"],
            crouch_offset=vmc_data.get("crouch_offset", -0.03),
            crouch_target_height=vmc_data.get("crouch_target_height", 0.09),
            crouch_duration=vmc_data.get("crouch_duration", 0.20),
            extend_offset=vmc_data.get("extend_offset", 0.05),
            landing_damping=vmc_data.get("landing_damping", 1.0),
            land_kp_scale=vmc_data.get("land_kp_scale", 0.5),
            land_kd_scale=vmc_data.get("land_kd_scale", 2.5),
            gravity_compensation_scale=vmc_data.get("gravity_compensation_scale", 1.0),
            max_height_rate=vmc_data.get("max_height_rate", 0.1),
            roll_level_kp_height=vmc_data.get("roll_level_kp_height", 0.0),
            roll_level_kd_height=vmc_data.get("roll_level_kd_height", 0.0),
            roll_level_offset_limit=vmc_data.get("roll_level_offset_limit", 0.0),
        ),
        q_diag=np.array(d["q_diag"]),
        r_diag=np.array(d["r_diag"]),
        target_velocity=d.get("target_velocity", 0.0),
        pitch_lean_gain=d.get("pitch_lean_gain", 0.02),
        velocity_ki=d.get("velocity_ki", 0.1),
        position_kp=d.get("position_kp", 1.5),
        position_kd=d.get("position_kd", 1.5),
        position_velocity_limit=d.get("position_velocity_limit", 0.3),
        yaw_damping=d.get("yaw_damping", 0.5),
        yaw_ki=d.get("yaw_ki", 0.0),
        target_yaw_rate=d.get("target_yaw_rate", 0.0),
        fixed_height=d.get("fixed_height", True),
        lqr_height_bin_size=d.get("lqr_height_bin_size", 0.02),
        ff_gain=d.get("ff_gain", 1.0),
    )


def params_to_json(params: CombinedParams) -> str:
    """将 CombinedParams 序列化为 JSON 字符串。"""
    return json.dumps(params_to_dict(params), indent=2)


def params_from_json(json_str: str) -> CombinedParams:
    """从 JSON 字符串反序列化 CombinedParams。"""
    return params_from_dict(json.loads(json_str))
