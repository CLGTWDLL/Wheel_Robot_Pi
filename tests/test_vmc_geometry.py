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


def test_right_link2_frame_keeps_drive_passive_inertia_coupling_symmetric(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    mass_matrix = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass_matrix, data.qM)

    def coupling(joint_a: str, joint_b: str) -> float:
        id_a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_a)
        id_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_b)
        return float(mass_matrix[model.jnt_dofadr[id_a], model.jnt_dofadr[id_b]])

    left = coupling("link1_left_旋转-6", "link2_left_旋转-13")
    right = coupling("link1_right_旋转-5", "link2_right_旋转-12")

    assert left > 0.0
    assert right > 0.0
    assert np.isclose(left, right, rtol=0.01, atol=1e-6)


def test_link23_connect_constraints_start_closed(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def body_point(body_name: str, local_point: np.ndarray) -> np.ndarray:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return data.xpos[body_id] + data.xmat[body_id].reshape(3, 3) @ local_point

    for side, body1, body2 in (
        ("left", "link2_left", "link3_left"),
        ("right", "link2_right", "link3_right"),
    ):
        equality_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, f"link23_{side}_connect")
        point1 = body_point(body1, model.eq_data[equality_id, 0:3])
        point2 = body_point(body2, model.eq_data[equality_id, 3:6])

        assert np.linalg.norm(point1 - point2) < 1e-6
