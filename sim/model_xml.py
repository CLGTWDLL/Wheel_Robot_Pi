from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

from sim.model_semantics import MODEL_SEMANTICS, WHEEL_RADIUS
from sim.mujoco_mesh_preprocess import prepare_mujoco_xml


CMD_SLIDER_NAMES = ("cmd_linear_x", "cmd_angular_z")


def _urdf_to_mjcf(urdf_path: Path, output_dir: Path) -> Path:
    """Convert a URDF file to MJCF using MuJoCo's built-in converter."""
    urdf_tree = ET.parse(urdf_path)
    urdf_root = urdf_tree.getroot()

    # Absolutize mesh paths so the temp URDF can find them
    for mesh_el in urdf_root.iter("mesh"):
        filename = mesh_el.get("filename")
        if not filename:
            continue
        mesh_path = (urdf_path.parent / filename).resolve()
        if mesh_path.is_file():
            mesh_el.set("filename", str(mesh_path))

    tmp_urdf = output_dir / urdf_path.name
    urdf_tree.write(tmp_urdf, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(tmp_urdf))
    mjcf_path = output_dir / f"{urdf_path.stem}_converted.xml"
    mujoco.mj_saveLastXML(str(mjcf_path), model)
    return mjcf_path


def prepare_controlled_mujoco_xml(urdf_path: Path, cache_dir: Path | None = None) -> Path:
    """Create a MuJoCo XML copy suitable for controlled simulation.

    Accepts a URDF file, converts it to MJCF via MuJoCo's built-in
    converter, then applies post-processing (freejoint, actuators,
    equality constraints, etc.).
    """
    urdf_path = urdf_path.expanduser().resolve()
    output_root = (
        Path(cache_dir).expanduser().resolve()
        if cache_dir is not None
        else Path(tempfile.mkdtemp(prefix="mujoco_controlled_"))
    )
    output_root.mkdir(parents=True, exist_ok=True)

    # Step 1: URDF -> MJCF via MuJoCo
    converted_xml = _urdf_to_mjcf(urdf_path, output_root)

    # Step 2: Mesh preprocessing on the converted MJCF
    mesh_cache_dir = output_root / "mesh_preprocess"
    prepared_xml = prepare_mujoco_xml(converted_xml, cache_dir=mesh_cache_dir)
    prepared_xml = prepared_xml.expanduser().resolve()

    tree = ET.parse(prepared_xml)
    root = tree.getroot()

    _make_mesh_paths_absolute(root, prepared_xml.parent)
    _ensure_world_environment(root)
    _ensure_root_freejoint(root)
    _ensure_command_slider_joints(root)
    _configure_geom_collisions(root)
    _add_wheel_collision_proxies(root)
    _replace_actuators(root)
    _ensure_equality_constraints(root)
    _add_connection_markers(root)

    output_path = output_root / f"{urdf_path.stem}_controlled.xml"
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


def _ensure_world_environment(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        return

    # Add light if not present
    if worldbody.find("light") is None:
        ET.SubElement(worldbody, "light", {"pos": "0 0 1.5", "dir": "0 0 -1", "directional": "true"})

    # Add floor if not present
    if not any(g.get("name") == "floor" for g in worldbody.findall("geom")):
        ET.SubElement(
            worldbody,
            "geom",
            {"name": "floor", "type": "plane", "size": "0 0 1", "pos": "0 0 0", "rgba": "0.8 0.9 0.8 1"},
        )

    # Disable gravity
    option = root.find("option")
    if option is None:
        option = ET.Element("option", {"gravity": "0 0 0"})
        root.insert(0, option)
    else:
        option.set("gravity", "0 0 0")


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


def _ensure_command_slider_joints(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MuJoCo XML is missing <worldbody>")
    if worldbody.find("body[@name='command_slider_body']") is not None:
        return

    body = ET.SubElement(worldbody, "body", {"name": "command_slider_body", "pos": "0 0 -10"})
    ET.SubElement(
        body,
        "inertial",
        {
            "mass": "1e-6",
            "pos": "0 0 0",
            "diaginertia": "1e-9 1e-9 1e-9",
        },
    )
    ET.SubElement(body, "joint", {"name": CMD_SLIDER_NAMES[0], "type": "hinge", "axis": "1 0 0", "damping": "0"})
    ET.SubElement(body, "joint", {"name": CMD_SLIDER_NAMES[1], "type": "hinge", "axis": "0 1 0", "damping": "0"})


def _configure_geom_collisions(root: ET.Element) -> None:
    # Set ground to contype=1, conaffinity=1 (default, but ensure it)
    # The ground is usually directly in worldbody
    worldbody = root.find("worldbody")
    if worldbody is not None:
        for geom in worldbody.findall("geom"):
            geom.set("contype", "1")
            geom.set("conaffinity", "1")

    # Set all robot geoms to contype=0, conaffinity=1 so they only collide with ground
    base_link = root.find(".//body[@name='base_link']")
    if base_link is not None:
        for geom in base_link.iter("geom"):
            geom_name = geom.get("name", "")
            is_wheel_visual = geom_name in ("wheel_left_geom", "wheel_right_geom")
            if geom.get("type") == "mesh" or is_wheel_visual:
                geom.set("contype", "0")
                geom.set("conaffinity", "0")
                geom.set("group", "1")
            else:
                geom.set("contype", "1")
                geom.set("conaffinity", "1")
                geom.set("group", "3")

def _add_wheel_collision_proxies(root: ET.Element) -> None:
    for body_name in ("wheel_left", "wheel_right"):
        body = root.find(f".//body[@name='{body_name}']")
        if body is None:
            raise ValueError(f"MuJoCo XML is missing wheel body: {body_name}")
        proxy_name = f"{body_name}_collision_proxy"
        if body.find(f"geom[@name='{proxy_name}']") is not None:
            continue
        inertial = body.find("inertial")
        inertial_pos = inertial.get("pos") if inertial is not None else None
        proxy_pos = inertial_pos if inertial_pos is not None else "0 0 0"
        ET.SubElement(
            body,
            "geom",
            {
                "name": proxy_name,
                "type": "sphere",
                "size": str(WHEEL_RADIUS),
                "pos": proxy_pos,
                "group": "3",
                "contype": "0",
                "conaffinity": "1",
                "friction": "1.0 0.02 0.001",
            },
        )


def _replace_actuators(root: ET.Element) -> None:
    for actuator in root.findall("actuator"):
        root.remove(actuator)

    actuator = ET.Element("actuator")
    for joint_name in MODEL_SEMANTICS.wheel_joints + MODEL_SEMANTICS.leg_motor_joints:
        ctrlrange = "-1.0 1.0" if joint_name in MODEL_SEMANTICS.wheel_joints else "-3.5 3.5"
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"act_{joint_name}",
                "joint": joint_name,
                "ctrllimited": "true",
                "ctrlrange": ctrlrange,
                "gear": "1",
            },
        )
    for name, ctrlrange in zip(CMD_SLIDER_NAMES, ("-1.0 1.0", "-3.0 3.0")):
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": name,
                "joint": name,
                "ctrllimited": "true",
                "ctrlrange": ctrlrange,
                "gear": "0",
            },
        )
    root.append(actuator)


def _ensure_equality_constraints(root: ET.Element) -> None:
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")

    # Only add if not already present
    if equality.find("connect[@name='link23_left_connect']") is not None:
        return

    # Computed local coordinates for the connection holes (link2 <-> link3)
    # These anchors are in the local coordinate frame of link2_left / link2_right
    ET.SubElement(
        equality,
        "connect",
        {
            "name": "link23_left_connect",
            "body1": "link2_left",
            "body2": "link3_left",
            "anchor": "-0.02865 -0.00846 0.02459",
            "solref": "0.005 1",
            "solimp": "0.95 0.99 0.001 0.5 2",
        },
    )

    ET.SubElement(
        equality,
        "connect",
        {
            "name": "link23_right_connect",
            "body1": "link2_right",
            "body2": "link3_right",
            "anchor": "-0.02869 0.00853 0.02452",
            "solref": "0.005 1",
            "solimp": "0.95 0.99 0.001 0.5 2",
        },
    )


def _add_connection_markers(root: ET.Element) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        return

    # Find the bodies and add a red sphere site to them
    for body in worldbody.iter("body"):
        name = body.get("name")
        if name == "link2_left":
            # Avoid adding multiple times if re-run
            if not any(s.get("name") == "marker_link23_left" for s in body.findall("site")):
                ET.SubElement(
                    body,
                    "site",
                    {
                        "name": "marker_link23_left",
                        "pos": "-0.02865 -0.00846 0.02459",
                        "size": "0.005",
                        "type": "sphere",
                        "rgba": "1 0 0 1",
                    },
                )
        elif name == "link2_right":
            if not any(s.get("name") == "marker_link23_right" for s in body.findall("site")):
                ET.SubElement(
                    body,
                    "site",
                    {
                        "name": "marker_link23_right",
                        "pos": "-0.02869 0.00853 0.02452",
                        "size": "0.005",
                        "type": "sphere",
                        "rgba": "1 0 0 1",
                    },
                )
