from pathlib import Path

import mujoco

from sim.model_semantics import MODEL_SEMANTICS
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"


def _load_model():
    return mujoco.MjModel.from_xml_path(str(prepare_mujoco_xml(XML_PATH)))


def test_model_semantics_references_existing_bodies_and_joints():
    model = _load_model()

    for body_name in MODEL_SEMANTICS.body_roles:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name) != -1

    for joint_name in MODEL_SEMANTICS.joint_roles:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name) != -1


def test_model_semantics_declares_expected_control_roles():
    assert MODEL_SEMANTICS.body_roles["base_link"] == "base"
    assert set(MODEL_SEMANTICS.wheel_joints) == {"robot_link2_left_r3", "robot_link2_right_r7"}
    assert set(MODEL_SEMANTICS.leg_motor_joints) == {
        "robot_base_link_r1",
        "robot_base_link_r2",
        "robot_base_link_r4",
        "robot_base_link_r9",
    }
    assert set(MODEL_SEMANTICS.passive_joints) == {"robot_link1_left_r8", "robot_link1_right_r6"}


def test_model_semantics_references_existing_connect_sites():
    model = _load_model()

    for site_name in MODEL_SEMANTICS.connect_sites:
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name) != -1
