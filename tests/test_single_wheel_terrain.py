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


def test_single_wheel_trapezoid_width_is_halved_without_changing_side_profile(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain="single_wheel_trapezoid",
    )
    root = ET.parse(model_path).getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None

    ramp = worldbody.find("geom[@name='single_wheel_trapezoid_ramp_up']")
    platform = worldbody.find("geom[@name='single_wheel_trapezoid_platform']")
    assert ramp is not None
    assert platform is not None

    ramp_size = _float_triplet(ramp.get("size"))
    platform_size = _float_triplet(platform.get("size"))
    assert ramp_size[1] == 0.2
    assert platform_size[1] == 0.2
    assert ramp_size[0] == 0.101519703
    assert ramp_size[2] == 0.01
    assert platform_size[0] == 0.125
    assert platform_size[2] == 0.0175


def test_rock_road_terrain_coexists_with_trapezoid(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(
        Path("sim/robot/robot.urdf"),
        cache_dir=tmp_path,
        terrain="single_wheel_trapezoid",
    )
    root = ET.parse(model_path).getroot()
    asset = root.find("asset")
    worldbody = root.find("worldbody")
    assert asset is not None
    assert worldbody is not None

    hfield = asset.find("hfield[@name='rock_road_hfield']")
    geom = worldbody.find("geom[@name='rock_road']")
    assert hfield is not None
    assert geom is not None
    assert geom.get("type") == "hfield"
    assert geom.get("hfield") == "rock_road_hfield"
    assert geom.get("contype") == "1"
    assert geom.get("conaffinity") == "1"
    assert _float_quad(hfield.get("size")) == (0.6, 0.3, 0.018, 0.02)
    assert (tmp_path / "rock_road_hfield.png").is_file()

    model = mujoco.MjModel.from_xml_path(str(model_path))
    hfield_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "rock_road_hfield")
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rock_road")
    assert hfield_id >= 0
    assert geom_id >= 0
    assert model.hfield_nrow[hfield_id] == 25
    assert model.hfield_ncol[hfield_id] == 49
    assert model.hfield_size[hfield_id].tolist() == [0.6, 0.3, 0.018, 0.02]


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


def test_default_rock_road_starts_after_spawn_and_trapezoid(tmp_path: Path) -> None:
    model, data = build_controlled_model(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)

    road_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rock_road")
    road_hfield = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_HFIELD, "rock_road_hfield")
    ramp_down = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "single_wheel_trapezoid_ramp_down",
    )
    left_wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_left")
    right_wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wheel_right")

    road_min_x = float(data.geom_xpos[road_geom, 0] - model.hfield_size[road_hfield, 0])
    ramp_max_x = float(data.geom_xpos[ramp_down, 0] + model.geom_size[ramp_down, 0])
    initial_wheel_max_x = float(max(data.xipos[left_wheel, 0], data.xipos[right_wheel, 0]))
    assert road_min_x > initial_wheel_max_x + 0.5
    assert road_min_x > ramp_max_x + 0.05


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
    rock_road_geom = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "rock_road",
    )

    assert terrain_geom == -1
    assert rock_road_geom == -1


def _float_triplet(value: str | None) -> tuple[float, float, float]:
    assert value is not None
    parts = tuple(float(part) for part in value.split())
    assert len(parts) == 3
    return parts


def _float_quad(value: str | None) -> tuple[float, float, float, float]:
    assert value is not None
    parts = tuple(float(part) for part in value.split())
    assert len(parts) == 4
    return parts
