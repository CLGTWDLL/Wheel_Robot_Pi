from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from sim.launch_mujoco import build_controlled_model
from sim.model_xml import prepare_controlled_mujoco_xml


def test_single_wheel_trapezoid_terrain_is_injected_after_preprocessing(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain="single_wheel_trapezoid",
    )
    root = ET.parse(model_path).getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None

    geoms = {
        geom.get("name"): geom
        for geom in worldbody.findall("geom")
    }

    assert "floor" in geoms
    for name in (
        "single_wheel_trapezoid_ramp_up",
        "single_wheel_trapezoid_platform",
        "single_wheel_trapezoid_ramp_down",
    ):
        geom = geoms[name]
        assert geom.get("type") == "box"
        assert geom.get("contype") == "1"
        assert geom.get("conaffinity") == "1"


def test_single_wheel_trapezoid_keeps_closed_loop_equality_constraints(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain="single_wheel_trapezoid",
    )
    model = mujoco.MjModel.from_xml_path(str(model_path))

    for equality_name in ("link23_left_connect", "link23_right_connect"):
        equality_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name)
        assert equality_id >= 0


def test_launch_controlled_model_uses_single_wheel_trapezoid_by_default(tmp_path: Path) -> None:
    model, _data = build_controlled_model(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)

    terrain_geom = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "single_wheel_trapezoid_platform",
    )

    assert terrain_geom >= 0


def test_default_trapezoid_starts_ahead_of_initial_wheels(tmp_path: Path) -> None:
    model, data = build_controlled_model(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)

    ramp_geom = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "single_wheel_trapezoid_ramp_up",
    )
    left_wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_left")
    right_wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_right")

    ramp_min_x = float(data.geom_xpos[ramp_geom, 0] - model.geom_size[ramp_geom, 0])
    initial_wheel_max_x = float(max(data.xipos[left_wheel, 0], data.xipos[right_wheel, 0]))
    assert ramp_min_x > initial_wheel_max_x + 0.02


def test_launch_controlled_model_can_disable_default_trapezoid(tmp_path: Path) -> None:
    model, _data = build_controlled_model(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain=None,
    )

    terrain_geom = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "single_wheel_trapezoid_platform",
    )

    assert terrain_geom == -1
