from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.phase import JumpPhase
from sim.launch_mujoco import (
    HOME_KEYFRAME,
    build_controlled_model,
    create_controlled_controller,
    parse_args,
    step_controlled_model,
)
from sim.rollout import zero_controller


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def test_parse_args_defaults_preserve_viewer_only_behavior():
    args = parse_args([])

    assert args.mode == "viewer"
    assert args.controller == "zero"
    assert args.scenario == "stand"
    assert args.xml == XML_PATH
    assert args.cache_dir is None


def test_controlled_model_builder_prepares_actuated_model_with_finite_home(tmp_path):
    model, data = build_controlled_model(XML_PATH, cache_dir=tmp_path)

    assert model.nu > 0
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, HOME_KEYFRAME)
    assert key_id != -1
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_step_controlled_model_clips_ctrl_steps_and_returns_finite(tmp_path):
    model, data = build_controlled_model(XML_PATH, cache_dir=tmp_path)
    requested = np.full(model.nu, 10.0)

    def controller(model, data, state):
        assert state.base_position.shape == (3,)
        return requested

    finite = step_controlled_model(model, data, controller)

    assert finite
    assert np.all(data.ctrl <= model.actuator_ctrlrange[:, 1])
    assert np.all(data.ctrl >= model.actuator_ctrlrange[:, 0])
    assert np.allclose(data.ctrl, model.actuator_ctrlrange[:, 1])
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_create_controlled_controller_supports_zero_and_jump_vmc():
    assert create_controlled_controller(controller_name="zero", scenario="stand") is zero_controller

    controller = create_controlled_controller(controller_name="vmc", scenario="jump")

    assert controller.phase_machine is not None
    assert controller.phase_machine.phase == JumpPhase.CROUCH
