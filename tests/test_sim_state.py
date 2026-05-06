from pathlib import Path

import numpy as np
import mujoco

from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import extract_sim_state, model_addresses


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _model_and_data(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(controlled_xml))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


def test_model_addresses_resolve_semantic_joints_and_actuators(tmp_path):
    model, _ = _model_and_data(tmp_path)

    addresses = model_addresses(model)

    assert addresses.root_qpos == 0
    assert addresses.root_qvel == 0
    assert set(addresses.joint_qpos) >= {"robot_link2_left_r3", "robot_link2_right_r7"}
    assert set(addresses.joint_qvel) >= {"robot_link2_left_r3", "robot_link2_right_r7"}
    assert set(addresses.actuators) >= {"robot_link2_left_r3", "robot_link2_right_r7"}
    assert addresses.joint_qpos["robot_link2_left_r3"] >= 7
    assert addresses.joint_qvel["robot_link2_left_r3"] >= 6


def test_extract_sim_state_returns_finite_controller_state(tmp_path):
    model, data = _model_and_data(tmp_path)

    state = extract_sim_state(model, data)

    assert state.base_position.shape == (3,)
    assert state.base_quaternion.shape == (4,)
    assert state.base_linear_velocity.shape == (3,)
    assert state.base_angular_velocity.shape == (3,)
    assert isinstance(state.pitch, float)
    assert isinstance(state.pitch_rate, float)
    assert set(state.wheel_positions) == {"left", "right"}
    assert set(state.wheel_velocities) == {"left", "right"}
    assert set(state.leg_joint_positions) == {
        "robot_base_link_r1",
        "robot_base_link_r9",
    }
    assert state.contact_count >= 0

    arrays = [
        state.base_position,
        state.base_quaternion,
        state.base_linear_velocity,
        state.base_angular_velocity,
        np.array([state.pitch, state.pitch_rate]),
        np.array(list(state.wheel_positions.values())),
        np.array(list(state.wheel_velocities.values())),
        np.array(list(state.leg_joint_positions.values())),
        np.array(list(state.leg_joint_velocities.values())),
    ]
    assert all(np.all(np.isfinite(array)) for array in arrays)


def test_model_addresses_fail_clearly_for_missing_actuator(tmp_path):
    model, _ = _model_and_data(tmp_path)
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_robot_link2_left_r3")
    original_trnid = model.actuator_trnid[actuator_id, 0]
    model.actuator_trnid[actuator_id, 0] = -1

    try:
        try:
            model_addresses(model)
        except ValueError as exc:
            assert "robot_link2_left_r3" in str(exc)
        else:
            raise AssertionError("expected missing actuator mapping to fail")
    finally:
        model.actuator_trnid[actuator_id, 0] = original_trnid
