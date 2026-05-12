from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from sim.controllers.vmc import LEG_HEIGHT_BODIES, VmcController, VmcParams, _leg_height
from sim.model_xml import prepare_controlled_mujoco_xml


def test_leg_height_uses_body_frame_z_not_inertial_frame(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    heights = {
        side: _leg_height(model, data, wheel_body)
        for side, (_joint_name, wheel_body) in LEG_HEIGHT_BODIES.items()
    }

    assert heights["left"] > 0.0
    assert heights["right"] > 0.0
    assert np.isclose(heights["left"], heights["right"], atol=0.01)


def test_vmc_leg_motor_jacobians_are_symmetric_for_current_model(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    controller = VmcController(VmcParams(nominal_height=0.0, kp_height=0.0, kd_height=0.0))
    jacobian = controller.leg_height_jacobian(model, data)

    assert jacobian["left"] < 0.0
    assert jacobian["right"] < 0.0
    assert np.isclose(jacobian["left"], jacobian["right"], rtol=0.1, atol=5e-4)
