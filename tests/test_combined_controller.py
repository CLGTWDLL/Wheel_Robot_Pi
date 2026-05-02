from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.combined import CombinedController, CombinedParams
from sim.controllers.vmc import VmcParams
from sim.model_semantics import MODEL_SEMANTICS
from sim.model_xml import prepare_controlled_mujoco_xml
from sim.rollout import RolloutConfig, run_rollout
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


def _params(nominal_height=0.31, target_velocity=0.0):
    return CombinedParams(
        vmc=VmcParams(nominal_height=nominal_height, kp_height=2.0, kd_height=0.5),
        q_diag=np.array([10.0, 1.0, 1.0, 0.1]),
        r_diag=np.array([0.1, 0.1]),
        target_velocity=target_velocity,
    )


def test_combined_controller_returns_finite_clipped_control(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = CombinedController(_params())

    control = controller(model, data, extract_sim_state(model, data))

    assert control.shape == (model.nu,)
    assert np.all(np.isfinite(control))
    assert np.all(control <= model.actuator_ctrlrange[:, 1])
    assert np.all(control >= model.actuator_ctrlrange[:, 0])


def test_combined_controller_wheel_channels_respond_to_pitch_disturbance(tmp_path):
    model, data = _model_and_data(tmp_path)
    state = extract_sim_state(model, data)
    disturbed_state = replace(state, pitch=state.pitch + 0.05)
    controller = CombinedController(_params())
    controller(model, data, state)

    control = controller(model, data, disturbed_state)

    addresses = model_addresses(model)
    wheel_actuators = [addresses.actuators[name] for name in MODEL_SEMANTICS.wheel_joints]
    assert np.any(np.abs(control[wheel_actuators]) > 1e-9)


def test_combined_controller_leg_channels_respond_to_higher_height_target(tmp_path):
    model, data = _model_and_data(tmp_path)
    state = extract_sim_state(model, data)
    controller = CombinedController(_params(nominal_height=float(state.base_position[2]) + 0.1))

    control = controller(model, data, state)

    addresses = model_addresses(model)
    leg_actuators = [addresses.actuators[name] for name in MODEL_SEMANTICS.leg_motor_joints]
    assert np.any(np.abs(control[leg_actuators]) > 1e-9)


def test_combined_controller_short_rollout_has_no_nan(tmp_path):
    controller = CombinedController(_params(nominal_height=0.06078))

    result = run_rollout(XML_PATH, RolloutConfig(duration=0.02, cache_dir=tmp_path), controller=controller)

    assert result.metrics.finite
    assert result.failure_reason is None
    assert result.steps >= 10
    assert result.last_control.shape == (result.model.nu,)
    assert np.all(np.isfinite(result.last_control))


def test_combined_controller_zero_target_velocity_sets_lqr_target_wheel_velocity_to_zero(tmp_path):
    model, data = _model_and_data(tmp_path)
    controller = CombinedController(_params(target_velocity=0.0))

    controller(model, data, extract_sim_state(model, data))

    assert controller.lqr_controller is not None
    assert controller.lqr_controller.target[3] == 0.0
