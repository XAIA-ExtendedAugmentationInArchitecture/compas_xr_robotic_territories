import mujoco
import mujoco.viewer
import time

# Load the model
model = mujoco.MjModel.from_xml_path(
    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\mujoco_test\output\ur20_mujoco.xml"
)

# Create a data instance
data = mujoco.MjData(model)

# Viewer (for manual inspection)
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer started. Close the window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
