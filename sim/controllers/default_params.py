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
        nominal_height=-0.026,
        kp_height=50.0,
        kd_height=5.0,
        crouch_offset=0.02,
        extend_offset=0.03,
        landing_damping=1.0,
    ),
    q_diag=np.array([10000.0, 1000.0, 1.0, 0.01]),
    r_diag=np.array([2.0, 2.0]),
    velocity_kp=0.5,
    velocity_ki=0.5,
    pitch_trim=0.0,
    pitch_lean_gain=0.05,
    yaw_damping=0.5,
    yaw_ki=0.0,
)

# 300 轮 Optuna 优化 (stand_then_drive 30s, 纯 PD VMC + LQR, score=-200.96)
STAND_THEN_DRIVE_PARAMS = CombinedParams(
    vmc=VmcParams(
        nominal_height=0.24,
        kp_height=20.0,
        kd_height=2.0,
        crouch_offset=0.02,
        extend_offset=0.03,
        landing_damping=0.628,
    ),
    q_diag=np.array([1000.0, 100.0, 10.0, 1.0]),
    r_diag=np.array([1.0, 1.0]),
    pitch_trim=0.0,
    pitch_lean_gain=0.05,
    velocity_kp=0.5,
    velocity_ki=0.5,
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
            "kp_height": params.vmc.kp_height,
            "kd_height": params.vmc.kd_height,
            "crouch_offset": params.vmc.crouch_offset,
            "extend_offset": params.vmc.extend_offset,
            "landing_damping": params.vmc.landing_damping,
        },
        "q_diag": params.q_diag.tolist(),
        "r_diag": params.r_diag.tolist(),
        "target_velocity": params.target_velocity,
        "pitch_lean_gain": params.pitch_lean_gain,
        "velocity_kp": params.velocity_kp,
        "velocity_ki": params.velocity_ki,
        "yaw_damping": params.yaw_damping,
    }


def params_from_dict(d: dict[str, Any]) -> CombinedParams:
    """从字典反序列化 CombinedParams。"""
    vmc_data = d["vmc"]
    return CombinedParams(
        vmc=VmcParams(
            nominal_height=vmc_data["nominal_height"],
            kp_height=vmc_data["kp_height"],
            kd_height=vmc_data["kd_height"],
            crouch_offset=vmc_data.get("crouch_offset", -0.03),
            extend_offset=vmc_data.get("extend_offset", 0.05),
            landing_damping=vmc_data.get("landing_damping", 1.0),
        ),
        q_diag=np.array(d["q_diag"]),
        r_diag=np.array(d["r_diag"]),
        target_velocity=d.get("target_velocity", 0.0),
        pitch_lean_gain=d.get("pitch_lean_gain", 0.02),
        velocity_kp=d.get("velocity_kp", 0.5),
        velocity_ki=d.get("velocity_ki", 0.1),
        yaw_damping=d.get("yaw_damping", 0.5),
    )


def params_to_json(params: CombinedParams) -> str:
    """将 CombinedParams 序列化为 JSON 字符串。"""
    return json.dumps(params_to_dict(params), indent=2)


def params_from_json(json_str: str) -> CombinedParams:
    """从 JSON 字符串反序列化 CombinedParams。"""
    return params_from_dict(json.loads(json_str))
