# from compas_fab.backends import PyBulletClient


# # with PyBulletClient(connection_type='direct') as client:
# #     print('Connected:', client.is_connected)

# import time
# from compas.datastructures import Mesh

# import compas_fab
# from compas_fab.backends import PyBulletClient
# from compas_fab.robots import CollisionMesh

# with PyBulletClient() as client:
#     urdf_path = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\02_Production\01_SetUp\20250105_setup_working\ur20.urdf"
#     # urdf_filepath = compas_fab.get(urdf_path)
#     robot = client.load_robot(urdf_path)
#     print(robot)

from compas_fab.backends import PyBulletClient
import pybullet

urdf_filepath = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\02_Production\01_SetUp\20250105_setup_working\ur20.urdf"

# Verify PyBullet can read the file
physicsClient = pybullet.connect(pybullet.DIRECT)
robot_id = pybullet.loadURDF(urdf_filepath)

if robot_id < 0:
    print("ERROR: PyBullet could not load the URDF. Check file path and format.")
else:
    print("SUCCESS: URDF loaded with ID", robot_id)

# Now, try with compas_fab
with PyBulletClient() as client:
    robot = client.load_robot(urdf_filepath)
    print(robot)