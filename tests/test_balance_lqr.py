from pathlib import Path

import numpy as np
import pytest
import mujoco

from sim.controllers.balance_lqr import _balance_state_selection, _reduced_balance_system, compute_balance_lqr_gain
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.state import model_addresses


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _model_and_data(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(controlled_xml))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


def test_compute_balance_lqr_gain_returns_finite_wheel_gain(tmp_path):
    model, data = _model_and_data(tmp_path)

    gain = compute_balance_lqr_gain(model, data, np.array([10.0, 1.0, 1.0, 0.1]), np.array([0.1, 0.1]))

    assert gain.shape == (2, 4)
    assert np.all(np.isfinite(gain))


def test_compute_balance_lqr_gain_changes_with_q_and_r(tmp_path):
    model, data = _model_and_data(tmp_path)

    first = compute_balance_lqr_gain(model, data, np.array([10.0, 1.0, 1.0, 0.1]), np.array([0.1, 0.1]))
    second = compute_balance_lqr_gain(model, data, np.array([1.0, 10.0, 0.1, 1.0]), np.array([1.0, 0.5]))

    assert not np.allclose(first, second)


def test_balance_state_selection_matches_sim_state_pitch_sign(tmp_path):
    model, _data = _model_and_data(tmp_path)
    addresses = model_addresses(model)
    selection = _balance_state_selection(model)

    assert selection[0, addresses.root_qvel + 4] == -1.0
    assert selection[1, model.nv + addresses.root_qvel + 4] == -1.0

    # 轮子行使用 WHEEL_FORWARD_SIGNS: 左轮 +0.5, 右轮 -0.5
    from sim.model_semantics import WHEEL_FORWARD_SIGNS, MODEL_SEMANTICS
    for name in MODEL_SEMANTICS.wheel_joints:
        dof = addresses.joint_qvel[name]
        expected_sign = WHEEL_FORWARD_SIGNS[name] / 2.0
        assert selection[2, dof] == expected_sign, f"P[2, {name}] = {selection[2, dof]}, expected {expected_sign}"
        assert selection[3, model.nv + dof] == expected_sign, f"P[3, {name}] = {selection[3, model.nv + dof]}, expected {expected_sign}"


@pytest.mark.parametrize(
    ("q_diag", "r_diag"),
    [
        (np.array([np.nan, 1.0, 1.0, 1.0]), np.array([1.0, 1.0])),
        (np.array([1.0, 1.0, 1.0, np.inf]), np.array([1.0, 1.0])),
        (np.array([1.0, 1.0, 0.0, 1.0]), np.array([1.0, 1.0])),
        (np.array([1.0, 1.0, 1.0]), np.array([1.0, 1.0])),
        (np.array([1.0, 1.0, 1.0, 1.0]), np.array([np.nan, 1.0])),
        (np.array([1.0, 1.0, 1.0, 1.0]), np.array([1.0, -1.0])),
        (np.array([1.0, 1.0, 1.0, 1.0]), np.array([1.0])),
    ],
)
def test_compute_balance_lqr_gain_rejects_nonfinite_or_nonpositive_weights(tmp_path, q_diag, r_diag):
    model, data = _model_and_data(tmp_path)

    with pytest.raises(ValueError):
        compute_balance_lqr_gain(model, data, q_diag, r_diag)


def test_reduced_balance_closed_loop_is_stable(tmp_path):
    model, data = _model_and_data(tmp_path)
    q_diag = np.array([10.0, 1.0, 1.0, 0.1])
    r_diag = np.array([0.1, 0.1])

    a, b = _reduced_balance_system(model, data)
    gain = compute_balance_lqr_gain(model, data, q_diag, r_diag)

    assert np.max(np.abs(np.linalg.eigvals(a - b @ gain))) < 1.0
