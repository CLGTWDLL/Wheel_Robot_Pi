import mujoco
model = mujoco.MjModel.from_xml_path("sim/robot/robot.urdf")
for i in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
    print(name, model.geom_pos[i], model.geom_quat[i])
