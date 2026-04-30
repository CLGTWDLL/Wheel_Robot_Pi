from pathlib import Path

import numpy as np
import mujoco

from sim.linearize import LinearizationConfig, linearize_transition, one_step_transition
from sim.model_xml import prepare_controlled_mujoco_xml


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _model_and_state(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(controlled_xml))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    state = np.concatenate([data.qpos.copy(), data.qvel.copy()])
    control = np.zeros(model.nu)
    return model, state, control


def test_one_step_transition_returns_next_full_state(tmp_path):
    model, state, control = _model_and_state(tmp_path)

    next_state = one_step_transition(model, state, control)

    assert next_state.shape == (model.nq + model.nv,)
    assert np.all(np.isfinite(next_state))


def test_linearize_transition_returns_finite_tangent_matrices(tmp_path):
    model, state, control = _model_and_state(tmp_path)

    result = linearize_transition(model, state, control, LinearizationConfig(epsilon=1e-6))

    assert result.valid
    assert result.a.shape == (2 * model.nv, 2 * model.nv)
    assert result.b.shape == (2 * model.nv, model.nu)
    assert np.all(np.isfinite(result.a))
    assert np.all(np.isfinite(result.b))


def test_linearized_prediction_matches_small_control_perturbation(tmp_path):
    model, state, control = _model_and_state(tmp_path)
    result = linearize_transition(model, state, control, LinearizationConfig(epsilon=1e-6))
    control_delta = np.zeros(model.nu)
    control_delta[0] = 1e-4

    actual_error = result.transition_error(one_step_transition(model, state, control + control_delta))
    predicted_error = result.b @ control_delta

    assert np.linalg.norm(actual_error - predicted_error) < 1e-5


def test_linearize_transition_rejects_invalid_epsilon(tmp_path):
    model, state, control = _model_and_state(tmp_path)

    result = linearize_transition(model, state, control, LinearizationConfig(epsilon=0.0))

    assert not result.valid
    assert result.reason == "invalid epsilon"


def test_linearize_transition_rejects_nonfinite_state(tmp_path):
    model, state, control = _model_and_state(tmp_path)
    state[0] = np.nan

    result = linearize_transition(model, state, control)

    assert not result.valid
    assert result.reason == "nonfinite input"
