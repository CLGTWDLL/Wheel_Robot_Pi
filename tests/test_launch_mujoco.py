from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.phase import JumpPhase
from sim.launch_mujoco import (
    HOME_KEYFRAME,
    build_controlled_model,
    create_controlled_controller,
    ensure_dependencies,
    parse_args,
    run_controlled_viewer_loop,
    step_controlled_model,
)
from sim.rollout import zero_controller


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def test_parse_args_defaults_use_controlled_zero_controller():
    args = parse_args([])

    assert args.mode == "controlled"
    assert args.controller == "zero"
    assert args.scenario == "stand"
    assert args.xml == XML_PATH
    assert args.cache_dir is None


def test_ensure_dependencies_only_requires_mujoco_by_default(monkeypatch):
    real_import = __import__

    def fake_import(name, *args, **kwargs):
        if name in {"trimesh", "fast_simplification"}:
            raise ModuleNotFoundError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", fake_import)

    ensure_dependencies()


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
        del model, data
        assert state.base_position.shape == (3,)
        return requested

    finite = step_controlled_model(model, data, controller)

    assert finite
    assert np.all(data.ctrl <= model.actuator_ctrlrange[:, 1])
    assert np.all(data.ctrl >= model.actuator_ctrlrange[:, 0])
    assert np.allclose(data.ctrl, model.actuator_ctrlrange[:, 1])
    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))


def test_controlled_viewer_loop_steps_while_holding_viewer_lock(monkeypatch):
    events: list[str] = []

    class FakeLock:
        def __enter__(self):
            events.append("lock_enter")

        def __exit__(self, exc_type, exc, traceback):
            del exc_type, exc, traceback
            events.append("lock_exit")

    class FakeViewer:
        def __init__(self):
            self.running_checks = 0

        def is_running(self):
            self.running_checks += 1
            return self.running_checks == 1

        def lock(self):
            return FakeLock()

        def sync(self):
            events.append("sync")

    def fake_step_controlled_model(model, data, controller):
        del model, data, controller
        assert events == ["lock_enter"]
        events.append("step")
        return True

    monkeypatch.setattr(
        "sim.launch_mujoco.step_controlled_model", fake_step_controlled_model
    )

    def unused_controller(model, data, state):
        del model, data, state
        return None

    run_controlled_viewer_loop(object(), object(), unused_controller, FakeViewer())

    assert events == ["lock_enter", "step", "lock_exit", "sync"]


def test_create_controlled_controller_supports_zero_and_jump_vmc():
    assert create_controlled_controller(controller_name="zero", scenario="stand") is zero_controller

    controller = create_controlled_controller(controller_name="vmc", scenario="jump")

    assert controller.phase_machine is not None
    assert controller.phase_machine.phase == JumpPhase.CROUCH
