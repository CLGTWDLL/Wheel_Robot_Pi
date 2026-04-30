from pathlib import Path

import numpy as np
import mujoco

from sim.controllers.lqr import LqrController, solve_discrete_lqr
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.rollout import RolloutConfig, run_rollout
from sim.state import extract_sim_state


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _model_and_data(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(controlled_xml))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    return model, data


def test_solve_discrete_lqr_stabilizes_scalar_system():
    a = np.array([[1.1]])
    b = np.array([[1.0]])
    q = np.array([[1.0]])
    r = np.array([[1.0]])

    gain = solve_discrete_lqr(a, b, q, r)

    assert gain.shape == (1, 1)
    assert abs((a - b @ gain)[0, 0]) < 1.0


def test_lqr_controller_returns_clipped_finite_control(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = LqrController.zero_gain(model)

    control = controller(model, data, extract_sim_state(model, data))

    assert control.shape == (model.nu,)
    assert np.all(np.isfinite(control))
    assert np.all(control <= model.actuator_ctrlrange[:, 1])
    assert np.all(control >= model.actuator_ctrlrange[:, 0])


def test_lqr_controller_requires_state_provider_for_nonzero_gain(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = LqrController(
        gain=np.ones((model.nu, 2 * model.nv)),
        target=np.zeros(2 * model.nv),
        feedforward=np.zeros(model.nu),
    )

    try:
        controller(model, data, extract_sim_state(model, data))
    except ValueError as exc:
        assert "tangent_state_provider" in str(exc)
    else:
        raise AssertionError("expected nonzero gain without provider to fail")


def test_lqr_controller_clips_large_feedback_to_actuator_range(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = LqrController(
        gain=np.full((model.nu, 2 * model.nv), 1_000.0),
        target=np.zeros(2 * model.nv),
        feedforward=np.zeros(model.nu),
        tangent_state_provider=lambda model, _data, _state: np.ones(2 * model.nv),
    )

    control = controller(model, data, extract_sim_state(model, data))

    assert np.all(control == model.actuator_ctrlrange[:, 0])


def test_lqr_controller_rejects_nonfinite_control(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = LqrController(
        gain=np.zeros((model.nu, 2 * model.nv)),
        target=np.zeros(2 * model.nv),
        feedforward=np.full(model.nu, np.nan),
    )

    try:
        controller(model, data, extract_sim_state(model, data))
    except ValueError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("expected nonfinite LQR control to fail")


def test_lqr_controller_runs_short_headless_rollout(tmp_path):
    def make_controller(model, data, state):
        del data, state
        return LqrController.zero_gain(model)

    controller_holder = {"controller": None}

    def controller(model, data, state):
        if controller_holder["controller"] is None:
            controller_holder["controller"] = make_controller(model, data, state)
        return controller_holder["controller"](model, data, state)

    result = run_rollout(XML_PATH, RolloutConfig(duration=0.05, cache_dir=tmp_path), controller=controller)

    assert result.metrics.finite
    assert result.last_control.shape == (result.model.nu,)
