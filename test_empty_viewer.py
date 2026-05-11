import mujoco
import mujoco.viewer
import sys

try:
    print("Creating model...")
    model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom type='plane' size='1 1 0.1'/></worldbody></mujoco>")
    data = mujoco.MjData(model)
    print("Launching viewer...")
    mujoco.viewer.launch(model, data)
except Exception as e:
    print("Exception:", type(e), e)
