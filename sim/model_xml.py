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
    _add_collision_proxies(root)
    _replace_actuators(root)
    _ensure_equality_constraints(root)
    _add_connection_markers(root)
    _ensure_standing_keyframe(root)

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

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")

    if asset.find("texture[@name='skybox']") is None:
        ET.SubElement(asset, "texture", {
            "name": "skybox", "type": "skybox", "builtin": "gradient",
            "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "512",
        })
    if asset.find("texture[@name='checker']") is None:
        ET.SubElement(asset, "texture", {
            "name": "checker", "type": "2d", "builtin": "checker",
            "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "width": "512", "height": "512",
            "mark": "cross", "markrgb": ".8 .8 .8",
        })
    if asset.find("material[@name='grid']") is None:
        ET.SubElement(asset, "material", {
            "name": "grid", "texture": "checker", "texrepeat": "1 1",
            "texuniform": "true", "reflectance": "0.2",
        })

    if worldbody.find("light") is None:
        ET.SubElement(worldbody, "light", {
            "pos": "0 0 3", "dir": "0 0 -1", "directional": "true", "castshadow": "true",
        })

    existing_floor = next((g for g in worldbody.findall("geom") if g.get("name") == "floor"), None)
    if existing_floor is None:
        ET.SubElement(worldbody, "geom", {
            "name": "floor", "type": "plane", "size": "0 0 1", "pos": "0 0 0", "material": "grid",
        })
    else:
        existing_floor.set("material", "grid")
        if "rgba" in existing_floor.attrib:
            del existing_floor.attrib["rgba"]

    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    if visual.find("headlight") is None:
        ET.SubElement(visual, "headlight", {"ambient": ".1 .1 .1", "diffuse": ".6 .6 .6", "specular": ".3 .3 .3"})
    if visual.find("rgba") is None:
        ET.SubElement(visual, "rgba", {"haze": ".15 .25 .35 1"})
    if visual.find("global") is None:
        ET.SubElement(visual, "global", {"azimuth": "120", "elevation": "-20"})

    # Use MuJoCo default gravity (0 0 -9.81). Only insert an <option> if absent.
    if root.find("option") is None:
        root.insert(0, ET.Element("option", {"gravity": "0 0 -9.81"}))


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

def _add_collision_proxies(root: ET.Element) -> None:
    """Add simple collision primitives to all robot bodies.

    Mesh geoms are visual-only (contype=0, conaffinity=0).  These proxies
    provide lightweight collision with the ground (conaffinity=1) without
    internal self-collision (contype=0).
    """
    # --- Collision proxy definitions ---
    # Each entry: body_name -> (type, size, pos, quat_or_None, friction)
    # Capsules: size="radius half-length", oriented along local X by default.
    # For MuJoCo capsules the cylinder axis is along Z in the geom frame,
    # so we use quat to rotate as needed.

    _PROXY_DEFS: list[dict[str, str]] = [
        # base_link: box approximation
        {
            "body": "base_link",
            "type": "box",
            "size": "0.10 0.05 0.03",
            "pos": "0 0.03 0.025",
        },
        # link1 left/right: short capsule along link (~67mm span)
        {
            "body": "link1_left",
            "type": "capsule",
            "size": "0.008",
            "fromto": "0 0 0 -0.067 0.015 -0.026",
        },
        {
            "body": "link1_right",
            "type": "capsule",
            "size": "0.008",
            "fromto": "0 0 0 -0.067 -0.015 -0.026",
        },
        # link2 left/right: longer capsule (~100mm span to wheel joint)
        {
            "body": "link2_left",
            "type": "capsule",
            "size": "0.008",
            "fromto": "0 0 0 0.076 0 -0.064",
        },
        {
            "body": "link2_right",
            "type": "capsule",
            "size": "0.008",
            "fromto": "0 0 0 0.076 0 -0.064",
        },
        # link3 left/right: short passive capsule (~40mm)
        {
            "body": "link3_left",
            "type": "capsule",
            "size": "0.006",
            "fromto": "0 0 0 -0.04 0 -0.02",
        },
        {
            "body": "link3_right",
            "type": "capsule",
            "size": "0.006",
            "fromto": "0 0 0 -0.04 0 0.02",
        },
        # wheels: sphere proxy (same as before)
        {
            "body": "wheel_left",
            "type": "sphere",
            "size": str(WHEEL_RADIUS),
            "friction": "1.0 0.02 0.001",
        },
        {
            "body": "wheel_right",
            "type": "sphere",
            "size": str(WHEEL_RADIUS),
            "friction": "1.0 0.02 0.001",
        },
    ]

    for defn in _PROXY_DEFS:
        body_name = defn["body"]
        body = root.find(f".//body[@name='{body_name}']")
        if body is None:
            continue
        proxy_name = f"{body_name}_collision_proxy"
        if body.find(f"geom[@name='{proxy_name}']") is not None:
            continue

        attrs: dict[str, str] = {
            "name": proxy_name,
            "type": defn["type"],
            "size": defn["size"],
            "group": "3",
            "contype": "0",
            "conaffinity": "1",
        }
        if "pos" in defn:
            attrs["pos"] = defn["pos"]
        if "fromto" in defn:
            attrs["fromto"] = defn["fromto"]
        if "friction" in defn:
            attrs["friction"] = defn["friction"]

        # For wheel proxies, place at inertial pos if no explicit pos given
        if "pos" not in defn and "fromto" not in defn:
            inertial = body.find("inertial")
            if inertial is not None and inertial.get("pos"):
                attrs["pos"] = inertial.get("pos")
            else:
                attrs["pos"] = "0 0 0"

        ET.SubElement(body, "geom", attrs)


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


# Constraint-consistent symmetric standing pose. Both legs in the "low elbow"
# branch of the 5-bar mechanism (link1 passive ≈ 0.98, base passive ≈ 0.72).
# Pre-computed via equality-constraint settling with gravity disabled and
# symmetric joint initialization. Base z lifts the wheel sphere proxies onto
# the floor (z=0). Wheels and command sliders at 0.
# CoM is ~2 cm ahead of wheels at this pose (manageable by LQR pitch_trim).
# Indices match the order in sim/state.py:model_addresses().
_STANDING_KEYFRAME_QPOS = (
    "0 0 0.175 "                          # base x y z
    "1 0 0 0 "                            # base quat (identity)
    "0.752 0.980 0 "                      # right: motor, link1 passive, wheel
    "0.752 0.980 0 "                      # left: motor, link1 passive, wheel
    "0.720 0.720 "                        # base passive joints (link3 right/left)
    "0 0"                                 # cmd_linear_x, cmd_angular_z
)


def _ensure_standing_keyframe(root: ET.Element) -> None:
    """Add a standing keyframe satisfying the closed-loop equality constraints."""
    keyframe = root.find("keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    if keyframe.find("key[@name='stand']") is not None:
        return
    ET.SubElement(keyframe, "key", {"name": "stand", "qpos": _STANDING_KEYFRAME_QPOS})
