#!/usr/bin/env python3
"""
Launch the robot URDF via MuJoCo's built-in converter (base fixed, no freejoint).
Adds position actuators on leg motors for manual slider control.
Use to verify model integrity (linkage, meshes, constraints).
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

LEG_MOTOR_JOINTS = ("base_link_旋转-2", "base_link_旋转-1")


def main():
    parser = argparse.ArgumentParser(description="Launch MuJoCo viewer for URDF testing.")
    parser.add_argument("--urdf", type=Path, default=Path("sim/robot/robot.urdf"))
    args = parser.parse_args()

    urdf_path = args.urdf.resolve()
    if not urdf_path.exists():
        print(f"Error: URDF file {urdf_path} does not exist.")
        sys.exit(1)

    # Load URDF via MuJoCo (handles oversized mesh splitting) and preprocess
    from sim.model_xml import _urdf_to_mjcf
    tmp_dir = Path(tempfile.mkdtemp())
    converted_xml = _urdf_to_mjcf(urdf_path, tmp_dir)

    prepared_xml = prepare_mujoco_xml(converted_xml)
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

    # Hardcode effort signs or default to 1.0 since VMC removed them
    actuator = ET.SubElement(root, "actuator")
    for jname in LEG_MOTOR_JOINTS:
        gear = -1.0 if "旋转-1" in jname else 1.0
        ET.SubElement(actuator, "position", {
            "name": f"act_{jname}",
            "joint": jname,
            "kp": "5",
            "ctrllimited": "true",
            "ctrlrange": "-1.5 1.5",
            "gear": str(gear),
        })

    # Disable gravity so the unconstrained passive links don't fall down
    option = root.find("option")
    if option is None:
        option = ET.Element("option", {"gravity": "0 0 0"})
        root.insert(0, option)
    else:
        option.set("gravity", "0 0 0")

    out_xml = Path(tempfile.mkdtemp()) / "check_model.xml"
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    data = mujoco.MjData(model)

    mujoco.mj_forward(model, data)

    # Sync ctrl sliders to current joint positions so legs don't jump
    for act_id in range(model.nu):
        if model.actuator_trntype[act_id] == mujoco.mjtTrn.mjTRN_JOINT:
            j_id = model.actuator_trnid[act_id, 0]
            # When gear is -1, ctrl * (-1) = qpos, so ctrl = qpos / (-1)
            gear = model.actuator_gear[act_id, 0]
            if gear != 0:
                data.ctrl[act_id] = data.qpos[model.jnt_qposadr[j_id]] / gear

    import time
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("=" * 60)
        print("Base is FIXED. Drag sliders in Control panel to move legs.")
        print("  act_base_link_旋转-2  ->  left leg")
        print("  act_base_link_旋转-1  ->  right leg")
        print("=" * 60)
        
        while viewer.is_running():
            # If you want to enable physics stepping from the UI, you can check if it's paused.
            # However, for just inspecting the model or dragging sliders, we just sync the viewer.
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(float(model.opt.timestep))


if __name__ == "__main__":
    main()
