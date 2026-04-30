from pathlib import Path
from xml.etree import ElementTree

import numpy as np
import mujoco

from sim.model_semantics import MODEL_SEMANTICS
from sim.model_xml import prepare_controlled_mujoco_xml


XML_PATH = Path(__file__).resolve().parents[1] / "sim" / "轮腿v1" / "轮腿v1.xml"
MAX_TRACKED_MESH_BYTES = 10 * 1024 * 1024


def _controlled_model(tmp_path):
    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)
    return controlled_xml, mujoco.MjModel.from_xml_path(str(controlled_xml))


def test_mujoco_xml_references_packaged_mesh_assets():
    xml_root = ElementTree.parse(XML_PATH).getroot()

    for mesh in xml_root.findall("./asset/mesh"):
        mesh_file = XML_PATH.parent / mesh.attrib["file"]
        assert mesh_file.is_file(), f"missing mesh asset: {mesh_file}"
        assert mesh_file.stat().st_size < MAX_TRACKED_MESH_BYTES, (
            f"mesh asset exceeds packaging size limit: {mesh_file}"
        )


def test_prepare_controlled_mujoco_xml_does_not_overwrite_source(tmp_path):
    source_before = XML_PATH.read_text()

    controlled_xml = prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)

    assert controlled_xml != XML_PATH
    assert controlled_xml.is_file()
    assert XML_PATH.read_text() == source_before


def test_prepare_controlled_mujoco_xml_isolates_mesh_preprocessing(tmp_path, monkeypatch):
    import sim.model_xml as model_xml

    seen_cache_dirs = []

    def fake_prepare_mujoco_xml(xml_path, cache_dir=None):
        seen_cache_dirs.append(cache_dir)
        return xml_path

    monkeypatch.setattr(model_xml, "prepare_mujoco_xml", fake_prepare_mujoco_xml)

    prepare_controlled_mujoco_xml(XML_PATH, cache_dir=tmp_path)

    assert seen_cache_dirs == [tmp_path / "mesh_preprocess"]


def test_controlled_model_has_root_freejoint_and_actuators(tmp_path):
    _, model = _controlled_model(tmp_path)

    root_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")

    assert root_joint_id != -1
    assert model.jnt_type[root_joint_id] == mujoco.mjtJoint.mjJNT_FREE
    assert model.nq == 15
    assert model.nv == 14
    assert model.nu == len(MODEL_SEMANTICS.wheel_joints) + len(MODEL_SEMANTICS.leg_motor_joints)
    assert np.all(np.abs(model.actuator_ctrlrange) <= 0.5)


def test_controlled_model_keyframe_reset_is_finite(tmp_path):
    _, model = _controlled_model(tmp_path)
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")

    assert key_id != -1

    mujoco.mj_resetDataKeyframe(model, data, key_id)

    assert np.all(np.isfinite(data.qpos))
    assert np.all(np.isfinite(data.qvel))
    assert data.qpos.shape == (model.nq,)
    assert data.qvel.shape == (model.nv,)


def test_controlled_model_marks_stl_mesh_geoms_visual_only(tmp_path):
    _, model = _controlled_model(tmp_path)

    for geom_id in range(model.ngeom):
        if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
            assert model.geom_contype[geom_id] == 0
            assert model.geom_conaffinity[geom_id] == 0


def test_controlled_model_adds_wheel_collision_proxies(tmp_path):
    _, model = _controlled_model(tmp_path)

    proxy_names = {"robot_wheel_left_collision", "robot_wheel_right_collision"}
    for proxy_name in proxy_names:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, proxy_name)
        assert geom_id != -1
        assert model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_SPHERE
        assert model.geom_contype[geom_id] != 0
        assert model.geom_conaffinity[geom_id] != 0
        body_name = proxy_name.removesuffix("_collision")
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert np.allclose(model.geom_pos[geom_id], model.body_ipos[body_id], atol=1e-9)


def test_controlled_home_keyframe_places_wheel_proxies_on_ground(tmp_path):
    _, model = _controlled_model(tmp_path)
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    wheel_bottoms = []
    for proxy_name in ("robot_wheel_left_collision", "robot_wheel_right_collision"):
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, proxy_name)
        wheel_bottoms.append(data.geom_xpos[geom_id, 2] - model.geom_size[geom_id, 0])

    assert min(wheel_bottoms) == np.asarray(wheel_bottoms).min()
    assert abs(min(wheel_bottoms)) < 1e-3
