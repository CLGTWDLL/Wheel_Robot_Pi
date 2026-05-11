import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><geom type='plane' size='1 1 0.1'/></worldbody></mujoco>")
data = mujoco.MjData(model)

print("Attempting to launch viewer...")
try:
    mujoco.viewer.launch(model, data)
    print("Viewer closed normally.")
except Exception as e:
    print("Failed to launch viewer:", repr(e))
