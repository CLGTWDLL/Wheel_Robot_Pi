from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from sim.model_semantics import MODEL_SEMANTICS
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml


ROOT_QPOS = "0 0 0.31 0 1 0 0"
CONTROL_RANGE = "-0.5 0.5"


def prepare_controlled_mujoco_xml(xml_path: Path, cache_dir: Path | None = None) -> Path:
    """Create a MuJoCo XML copy suitable for controlled simulation."""
    xml_path = xml_path.expanduser().resolve()
    output_root = (
        Path(cache_dir).expanduser().resolve()
        if cache_dir is not None
        else Path(tempfile.mkdtemp(prefix="mujoco_controlled_"))
    )
    output_root.mkdir(parents=True, exist_ok=True)

    mesh_cache_dir = output_root / "mesh_preprocess"
    prepared_xml = prepare_mujoco_xml(xml_path, cache_dir=mesh_cache_dir)
    prepared_xml = prepared_xml.expanduser().resolve()

    tree = ET.parse(prepared_xml)
    root = tree.getroot()

    _make_mesh_paths_absolute(root, prepared_xml.parent)
    _ensure_root_freejoint(root)
    _mark_mesh_geoms_visual_only(root)
    _replace_actuators(root)
    _expand_keyframes(root)

    output_path = output_root / f"{xml_path.stem}_controlled.xml"
    tree.write(output_path, encoding="utf-8", xml_declaration=False)
    return output_path


def _make_mesh_paths_absolute(root: ET.Element, xml_dir: Path) -> None:
    asset = root.find("asset")
    if asset is None:
        return

    for mesh in asset.findall("mesh"):
        file_attr = mesh.get("file")
        if not file_attr:
            continue
        mesh_path = Path(file_attr)
        if not mesh_path.is_absolute():
            mesh.set("file", str((xml_dir / mesh_path).resolve()))


def _ensure_root_freejoint(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MuJoCo XML is missing <worldbody>")

    base_link = worldbody.find("body[@name='base_link']")
    if base_link is None:
        raise ValueError("MuJoCo XML is missing base_link body")

    for child in base_link:
        if child.tag in {"freejoint", "joint"} and child.get("name") == "root":
            return

    base_link.insert(0, ET.Element("freejoint", {"name": "root"}))


def _mark_mesh_geoms_visual_only(root: ET.Element) -> None:
    for geom in root.iter("geom"):
        if geom.get("type") == "mesh":
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            geom.set("group", "1")


def _replace_actuators(root: ET.Element) -> None:
    for actuator in root.findall("actuator"):
        root.remove(actuator)

    actuator = ET.Element("actuator")
    for joint_name in MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints:
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"act_{joint_name}",
                "joint": joint_name,
                "ctrllimited": "true",
                "ctrlrange": CONTROL_RANGE,
                "gear": "1",
            },
        )
    root.append(actuator)


def _expand_keyframes(root: ET.Element) -> None:
    keyframe = root.find("keyframe")
    if keyframe is None:
        return

    for key in keyframe.findall("key"):
        qpos = key.get("qpos")
        if qpos:
            values = qpos.split()
            if len(values) == 8:
                key.set("qpos", f"{ROOT_QPOS} {qpos}")
        key.set("qvel", " ".join("0" for _ in range(14)))
