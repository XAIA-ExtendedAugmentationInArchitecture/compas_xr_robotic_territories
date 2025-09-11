import pybullet as p
import time

cid1 = p.connect(p.GUI)  # or p.GUI
cid2 = p.connect(p.DIRECT)


urdf_1 = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\urdf\ur_description\urdf\ur20.urdf"
urdf_2 = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\planning\ur_20_attached_ee_attached_base\urdf\ur20_ee_ab.urdf"

# Reset and load stuff in environment 1
p.resetSimulation(physicsClientId=cid1)
p.loadURDF(urdf_1, physicsClientId=cid1)

# Reset and load stuff in environment 2
p.resetSimulation(physicsClientId=cid2)
p.loadURDF(urdf_2, physicsClientId=cid2)

# Step both individually
p.stepSimulation(physicsClientId=cid1)
p.stepSimulation(physicsClientId=cid2)

time.sleep(2)