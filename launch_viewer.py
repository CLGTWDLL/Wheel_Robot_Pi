#!/usr/bin/env python3
"""
Launch the robot XML as-is (base fixed, no freejoint).
Adds position actuators on leg motors for manual slider control.
Use to verify XML model integrity (linkage, meshes, constraints).
"""

import argparse
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import mujoco
import mujoco.viewer

from sim.mujoco_mesh_preprocess import prepare_mujoco_xml

LEG_MOTOR_JOINTS = ("robot_base_link_r1", "robot_base_link_r9")


def main():
    parser = argparse.ArgumentParser(description="Launch MuJoCo viewer for XML testing.")
    parser.add_argument("--xml", type=Path, default=Path("sim/轮腿v1/轮腿v1.xml"))
    args = parser.parse_args()

    xml_path = args.xml.resolve()
    if not xml_path.exists():
        print(f"Error: XML file {xml_path} does not exist.")
        sys.exit(1)

    # Mesh preprocessing only — no structural changes
    prepared_xml = prepare_mujoco_xml(xml_path)
    prepared_dir = Path(prepared_xml).parent

    tree = ET.parse(prepared_xml)
    root = tree.getroot()

    # Make mesh paths absolute so temp output dir can still find them
    asset = root.find("asset")
    if asset is not None:
        for mesh in asset.findall("mesh"):
            f = mesh.get("file")
            if f and not Path(f).is_absolute():
                mesh.set("file", str((prepared_dir / f).resolve()))

    from sim.controllers.vmc import LEG_EFFORT_SIGNS

    # Append position actuators for the two leg motors — nothing else changes
    actuator = ET.SubElement(root, "actuator")
    for jname in LEG_MOTOR_JOINTS:
        gear = LEG_EFFORT_SIGNS.get(jname, 1.0)
        ET.SubElement(actuator, "position", {
            "name": f"act_{jname}",
            "joint": jname,
            "kp": "5",
            "ctrllimited": "true",
            "ctrlrange": "-1.5 1.5",
            "gear": str(gear),
        })

    out_xml = Path(tempfile.mkdtemp()) / "check_model.xml"
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    data = mujoco.MjData(model)

    # Apply home keyframe — this is what connects the five-bar linkage
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "link23_connected_home")
    if key_id != -1:
        mujoco.mj_resetDataKeyframe(model, data, key_id)

    mujoco.mj_forward(model, data)

    # Sync ctrl sliders to current joint positions so legs don't jump
    for act_id in range(model.nu):
        if model.actuator_trntype[act_id] == mujoco.mjtTrn.mjTRN_JOINT:
            j_id = model.actuator_trnid[act_id, 0]
            # When gear is -1, ctrl * (-1) = qpos, so ctrl = qpos / (-1)
            gear = model.actuator_gear[act_id, 0]
            data.ctrl[act_id] = data.qpos[model.jnt_qposadr[j_id]] / gear

    print("=" * 60)
    print("Base is FIXED. Drag sliders in Control panel to move legs.")
    print("  act_robot_base_link_r1  ->  left leg")
    print("  act_robot_base_link_r9  ->  right leg")
    print("=" * 60)

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
