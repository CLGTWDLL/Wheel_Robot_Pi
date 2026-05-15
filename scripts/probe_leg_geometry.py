#!/usr/bin/env python3
"""Generate sim/controllers/leg_height_lut.json by sweeping motor angles.

The 5-bar mechanism's closed-loop equality constraint makes the linkage
oscillate when pushed by the actuator clip (+/-5 N*m) past its "easy" range.
This probe samples θ in fine steps and averages θ_actual / h over the tail
of each settle window so the LUT captures the mean kinematic h(θ) curve
despite the residual oscillation.

Runtime VMC uses the LUT to map ``cmd_height → target motor angle``, which
avoids the singularity-driven explosion seen with task-space height feedback.

When to rerun (the LUT becomes stale if any of these change):
- ``sim/robot/robot.urdf`` — joint geometry, masses, mesh references
- ``sim/model_xml.py`` — closed-loop equality anchors, keyframe, actuators
- ``sim/mujoco_mesh_preprocess.py`` — collision/visual mesh transforms
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.controllers.vmc import LEG_CLOSED_LOOP
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import body_id, model_addresses


THETA_MIN = 0.325
THETA_MAX = 0.65
THETA_STEP = 0.005

KP_PROBE = 80.0
KD_PROBE = 10.0
MOTOR_CTRL_CLIP = 5.0
SETTLE_STEPS = 3000
AVERAGE_WINDOW = 600  # average over last N steps to filter constraint oscillation


def _drive_to_target(
    model,
    data,
    theta_target,
    *,
    motor_acts,
    motor_qpos,
    motor_qvel,
    base_id,
    wheel_ids,
    root_qpos,
    root_qvel,
    base_qpos_pinned,
):
    theta_L_samples: list[float] = []
    theta_R_samples: list[float] = []
    h_samples: list[float] = []
    dy_wheel_samples: list[float] = []
    pitch_eq_samples: list[float] = []
    for step in range(SETTLE_STEPS):
        for side in LEG_CLOSED_LOOP:
            theta_cur = float(data.qpos[motor_qpos[side]])
            theta_rate = float(data.qvel[motor_qvel[side]])
            torque = KP_PROBE * (theta_target - theta_cur) - KD_PROBE * theta_rate
            data.ctrl[motor_acts[side]] = float(
                np.clip(torque, -MOTOR_CTRL_CLIP, MOTOR_CTRL_CLIP)
            )
        mujoco.mj_step(model, data)
        data.qpos[root_qpos : root_qpos + 7] = base_qpos_pinned
        data.qvel[root_qvel : root_qvel + 6] = 0.0
        mujoco.mj_forward(model, data)

        if step >= SETTLE_STEPS - AVERAGE_WINDOW:
            theta_L_samples.append(float(data.qpos[motor_qpos["left"]]))
            theta_R_samples.append(float(data.qpos[motor_qpos["right"]]))
            base_z = float(data.xipos[base_id, 2])
            wheel_z_l = float(data.xipos[wheel_ids["left"], 2])
            wheel_z_r = float(data.xipos[wheel_ids["right"], 2])
            h_samples.append(base_z - 0.5 * (wheel_z_l + wheel_z_r))
            dy_wheel, pitch_eq = _base_frame_geometry(model, data, base_id, wheel_ids)
            dy_wheel_samples.append(dy_wheel)
            pitch_eq_samples.append(pitch_eq)

    theta_L_mean = float(np.mean(theta_L_samples))
    theta_R_mean = float(np.mean(theta_R_samples))
    h_mean = float(np.mean(h_samples))
    dy_wheel_mean = float(np.mean(dy_wheel_samples))
    pitch_eq_mean = float(np.mean(pitch_eq_samples))
    theta_std = float(np.std(0.5 * (np.array(theta_L_samples) + np.array(theta_R_samples))))
    h_std = float(np.std(h_samples))
    qvel_max = max(
        float(abs(data.qvel[motor_qvel[side]])) for side in LEG_CLOSED_LOOP
    )
    return theta_L_mean, theta_R_mean, h_mean, dy_wheel_mean, pitch_eq_mean, theta_std, h_std, qvel_max


def _base_frame_geometry(model, data, base_id: int, wheel_ids: dict[str, int]) -> tuple[float, float]:
    base_p = np.asarray(data.xpos[base_id], dtype=float)
    base_R = np.asarray(data.xmat[base_id], dtype=float).reshape(3, 3)
    wheel_mid_w = 0.5 * (
        np.asarray(data.xipos[wheel_ids["left"]], dtype=float)
        + np.asarray(data.xipos[wheel_ids["right"]], dtype=float)
    )
    masses = np.asarray(model.body_mass, dtype=float)
    com_w = (masses[:, None] * np.asarray(data.xipos, dtype=float)).sum(axis=0) / float(np.sum(masses))

    wheel_in_base = base_R.T @ (wheel_mid_w - base_p)
    com_in_base = base_R.T @ (com_w - base_p)
    com_from_wheel = com_in_base - wheel_in_base
    pitch_eq = float(np.atan2(-com_from_wheel[1], abs(com_from_wheel[2])))
    return float(wheel_in_base[1]), pitch_eq


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    urdf_path = repo_root / "sim" / "robot" / "robot.urdf"
    cache_dir = repo_root / "tmp" / "probe_leg_geometry"
    cache_dir.mkdir(parents=True, exist_ok=True)

    model_path = prepare_controlled_mujoco_xml(urdf_path, cache_dir=cache_dir)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    model.opt.gravity[:] = 0.0

    addresses = model_addresses(model)
    motor_acts = {
        side: addresses.actuators[geometry.motor_joint]
        for side, geometry in LEG_CLOSED_LOOP.items()
    }
    motor_qpos = {
        side: addresses.joint_qpos[geometry.motor_joint]
        for side, geometry in LEG_CLOSED_LOOP.items()
    }
    motor_qvel = {
        side: addresses.joint_qvel[geometry.motor_joint]
        for side, geometry in LEG_CLOSED_LOOP.items()
    }
    base_id = body_id(model, "base_link")
    wheel_ids = {
        side: body_id(model, geometry.wheel_body)
        for side, geometry in LEG_CLOSED_LOOP.items()
    }

    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    if stand_id < 0:
        raise RuntimeError("missing 'stand' keyframe")
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)

    root_qpos = addresses.root_qpos
    root_qvel = addresses.root_qvel
    base_qpos_pinned = data.qpos[root_qpos : root_qpos + 7].copy()

    theta_grid = np.arange(THETA_MAX, THETA_MIN - 1e-9, -THETA_STEP)
    print(
        f"sweep θ from {theta_grid[0]:.4f} down to {theta_grid[-1]:.4f} rad,"
        f" step {THETA_STEP}, average over last {AVERAGE_WINDOW} steps"
    )
    print(f"{'θ_target':>9} {'θ_L_mean':>9} {'θ_R_mean':>9} {'h_mean':>8} {'θ_std':>8} {'h_std':>8} {'qv_end':>8}")
    print("-" * 75)

    results = []
    for theta_target in theta_grid:
        (
            theta_L_mean,
            theta_R_mean,
            h_mean,
            dy_wheel_mean,
            pitch_eq_mean,
            theta_std,
            h_std,
            qvel_max,
        ) = _drive_to_target(
            model, data, float(theta_target),
            motor_acts=motor_acts, motor_qpos=motor_qpos, motor_qvel=motor_qvel,
            base_id=base_id, wheel_ids=wheel_ids,
            root_qpos=root_qpos, root_qvel=root_qvel, base_qpos_pinned=base_qpos_pinned,
        )
        results.append(
            {
                "theta_target": float(theta_target),
                "theta_actual_left": theta_L_mean,
                "theta_actual_right": theta_R_mean,
                "h": h_mean,
                "dy_wheel_in_base": dy_wheel_mean,
                "pitch_eq": pitch_eq_mean,
                "theta_std": theta_std,
                "h_std": h_std,
                "final_motor_qvel_max": qvel_max,
            }
        )
        print(
            f"{theta_target:>9.4f} {theta_L_mean:>9.4f} {theta_R_mean:>9.4f}"
            f" {h_mean:>8.4f} {theta_std:>8.2e} {h_std:>8.2e} {qvel_max:>8.2e}"
        )

    theta_arr = np.array(
        [0.5 * (r["theta_actual_left"] + r["theta_actual_right"]) for r in results]
    )
    h_arr = np.array([r["h"] for r in results])
    dy_arr = np.array([r["dy_wheel_in_base"] for r in results])
    pitch_eq_arr = np.array([r["pitch_eq"] for r in results])
    order = np.argsort(theta_arr)
    theta_sorted = theta_arr[order]
    h_sorted = h_arr[order]
    dy_sorted = dy_arr[order]
    pitch_eq_sorted = pitch_eq_arr[order]

    keep_idx = [0]
    for i in range(1, len(theta_sorted)):
        if theta_sorted[i] - theta_sorted[keep_idx[-1]] > 5e-4:
            keep_idx.append(i)
    theta_sorted = theta_sorted[keep_idx]
    h_sorted = h_sorted[keep_idx]
    dy_sorted = dy_sorted[keep_idx]
    pitch_eq_sorted = pitch_eq_sorted[keep_idx]

    diffs = np.diff(h_sorted)
    strictly_monotonic = bool(np.all(diffs > 0) or np.all(diffs < 0))
    sign_change_indices: list[int] = []
    if not strictly_monotonic:
        sign_change_indices = (np.where(np.diff(np.sign(diffs)) != 0)[0] + 1).tolist()

    output = {
        "theta_grid": theta_sorted.tolist(),
        "h_grid": h_sorted.tolist(),
        "dy_grid": dy_sorted.tolist(),
        "pitch_eq_grid": pitch_eq_sorted.tolist(),
        "h_min": float(h_sorted.min()),
        "h_max": float(h_sorted.max()),
        "theta_min": float(theta_sorted.min()),
        "theta_max": float(theta_sorted.max()),
        "monotonic": strictly_monotonic,
        "sign_change_indices": sign_change_indices,
        "raw": results,
    }

    lut_path = repo_root / "sim" / "controllers" / "leg_height_lut.json"
    with open(lut_path, "w") as f:
        json.dump(output, f, indent=2)

    print()
    print(f"LUT saved to {lut_path}")
    print(f"  unique samples: {len(theta_sorted)}")
    print(f"  θ range: [{output['theta_min']:.4f}, {output['theta_max']:.4f}] rad")
    print(f"  h range: [{output['h_min']:.4f}, {output['h_max']:.4f}] m")
    print(f"  h span:  {output['h_max'] - output['h_min']:.4f} m")
    print(f"  h(θ) strictly monotonic: {strictly_monotonic}")
    dy_dh = np.gradient(dy_sorted, h_sorted)
    sample_indices = np.linspace(0, len(dy_dh) - 1, 3, dtype=int)
    print("  dy_wheel_dh samples:")
    for idx in sample_indices:
        print(f"    h={h_sorted[idx]:.4f} dy/dh={dy_dh[idx]:.4f}")
    print(f"  pitch_eq(h=0.142): {float(np.interp(0.142, h_sorted, pitch_eq_sorted)):.4f} rad")
    if not strictly_monotonic:
        print(f"  ⚠ sign change indices: {sign_change_indices}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
