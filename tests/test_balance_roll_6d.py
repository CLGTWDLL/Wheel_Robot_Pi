from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.balance_lqr import (
    LEG_ROLL_DIFF_SIGNS,
    _balance_state_selection,
    _base_roll_inertia,
    _com_height_above_wheels,
    _lqr_control_selection,
    _reduced_balance_system,
    _track_width,
    compute_balance_lqr_gain,
)
from sim.controllers.balance_state import balance_tangent_state_6d
from sim.controllers.combined import CombinedController
from sim.controllers.default_params import STAND_PARAMS
from sim.model_semantics import MODEL_SEMANTICS, WHEEL_FORWARD_SIGNS, WHEEL_RADIUS
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import extract_sim_state, model_addresses


def _stand_model(tmp_path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    assert stand_id >= 0
    mujoco.mj_resetDataKeyframe(model, data, stand_id)
    mujoco.mj_forward(model, data)
    return model, data


def test_balance_tangent_state_6d_matches_sim_state_units(tmp_path: Path) -> None:
    model, data = _stand_model(tmp_path)
    state = extract_sim_state(model, data)

    tangent = balance_tangent_state_6d(model, data, state)

    left_wheel, right_wheel = MODEL_SEMANTICS.wheel_joints
    expected_wheel_pos = 0.5 * (
        WHEEL_FORWARD_SIGNS[left_wheel] * state.wheel_positions["left"]
        + WHEEL_FORWARD_SIGNS[right_wheel] * state.wheel_positions["right"]
    ) * WHEEL_RADIUS
    expected_wheel_vel = 0.5 * (
        WHEEL_FORWARD_SIGNS[left_wheel] * state.wheel_velocities["left"]
        + WHEEL_FORWARD_SIGNS[right_wheel] * state.wheel_velocities["right"]
    ) * WHEEL_RADIUS

    assert tangent.shape == (6,)
    assert np.allclose(
        tangent,
        np.array([state.pitch, state.pitch_rate, state.roll, state.roll_rate, expected_wheel_pos, expected_wheel_vel]),
    )


def test_balance_state_selection_is_6d_and_matches_small_angle_rows(tmp_path: Path) -> None:
    model, _data = _stand_model(tmp_path)
    addresses = model_addresses(model)

    p = _balance_state_selection(model)

    assert p.shape == (6, 2 * model.nv)
    assert p[0, addresses.root_qvel + 3] == -1.0
    assert p[1, model.nv + addresses.root_qvel + 3] == -1.0
    assert p[2, addresses.root_qvel + 4] == 1.0
    assert p[3, model.nv + addresses.root_qvel + 4] == 1.0
    for joint_name in MODEL_SEMANTICS.wheel_joints:
        dof = addresses.joint_qvel[joint_name]
        assert p[4, dof] == WHEEL_FORWARD_SIGNS[joint_name] * WHEEL_RADIUS / 2.0
        assert p[5, model.nv + dof] == WHEEL_FORWARD_SIGNS[joint_name] * WHEEL_RADIUS / 2.0


def test_lqr_control_selection_maps_forward_and_roll_diff(tmp_path: Path) -> None:
    model, _data = _stand_model(tmp_path)
    addresses = model_addresses(model)

    s = _lqr_control_selection(model)

    assert s.shape == (model.nu, 2)
    for joint_name in MODEL_SEMANTICS.wheel_joints:
        assert s[addresses.actuators[joint_name], 0] == WHEEL_FORWARD_SIGNS[joint_name]
        assert s[addresses.actuators[joint_name], 1] == 0.0
    for joint_name, sign in LEG_ROLL_DIFF_SIGNS.items():
        assert s[addresses.actuators[joint_name], 0] == 0.0
        assert s[addresses.actuators[joint_name], 1] == sign


def test_reduced_balance_system_is_6d_2input_and_stabilizable(tmp_path: Path) -> None:
    model, data = _stand_model(tmp_path)

    a, b = _reduced_balance_system(model, data)
    gain = compute_balance_lqr_gain(model, data, STAND_PARAMS.q_diag, STAND_PARAMS.r_diag)
    eigenvalues = np.linalg.eigvals(a - b @ gain)

    assert a.shape == (6, 6)
    assert b.shape == (6, 2)
    assert gain.shape == (2, 6)
    assert np.all(np.isfinite(a))
    assert np.all(np.isfinite(b))
    assert np.max(np.abs(eigenvalues)) < 1.0
    assert np.allclose(a[0, 1], 0.002)
    assert np.allclose(a[2, 3], 0.002)
    assert np.allclose(a[4, 5], 0.002)
    assert b[1, 0] < 0.0
    assert b[5, 0] > 0.0
    assert b[3, 1] != 0.0
    assert np.allclose(b[[0, 2, 4], :], 0.0)


def test_reduced_balance_system_matches_roll_formula(tmp_path: Path) -> None:
    model, data = _stand_model(tmp_path)
    a, b = _reduced_balance_system(model, data)

    dt = 0.002
    l_roll = _com_height_above_wheels(model, data)
    i_roll = _base_roll_inertia(model, data)
    track_width = _track_width(model, data)
    expected_b_roll = (track_width / 2.0) / (i_roll * l_roll) * dt

    assert np.isclose(a[3, 2], (9.81 / l_roll) * dt)
    assert np.isclose(b[3, 1], expected_b_roll)


def test_vmc_mean_and_lqr_roll_diff_are_added_orthogonally(tmp_path: Path) -> None:
    model, _data = _stand_model(tmp_path)
    addresses = model_addresses(model)
    controller = CombinedController(STAND_PARAMS)
    vmc_control = np.zeros(model.nu)
    left_leg, right_leg = MODEL_SEMANTICS.leg_motor_joints
    vmc_control[addresses.actuators[left_leg]] = 3.0
    vmc_control[addresses.actuators[right_leg]] = 3.0

    control = controller._assemble_and_clip_control(model, 0.0, 0.0, 0.5, vmc_control, addresses)

    assert np.isclose(control[addresses.actuators[left_leg]], 2.5)
    assert np.isclose(control[addresses.actuators[right_leg]], 3.5)


def test_positive_roll_virtual_torque_matches_measured_short_step_direction(tmp_path: Path) -> None:
    model, data = _stand_model(tmp_path)
    addresses = model_addresses(model)
    controller = CombinedController(STAND_PARAMS)
    stand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "stand")
    before = extract_sim_state(model, data)

    control = controller._allocate_lqr_control(model, 0.0, 1.0, addresses)
    for _ in range(5):
        data.ctrl[:] = 0.0
        for joint_name in MODEL_SEMANTICS.leg_motor_joints:
            act_idx = addresses.actuators[joint_name]
            data.ctrl[act_idx] = control[act_idx]
        mujoco.mj_step(model, data)

    after = extract_sim_state(model, data)
    assert stand_id >= 0
    assert after.roll - before.roll > 0.0
