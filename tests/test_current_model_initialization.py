from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from sim.model_xml import prepare_controlled_mujoco_xml


def test_controlled_model_uses_identity_root_pose_without_keyframes(tmp_path: Path) -> None:
    model_path = prepare_controlled_mujoco_xml(Path("sim/robot/robot.urdf"), cache_dir=tmp_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    mujoco.mj_forward(model, data)

    assert model.nkey == 0
    assert np.allclose(data.qpos[:7], np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]))
