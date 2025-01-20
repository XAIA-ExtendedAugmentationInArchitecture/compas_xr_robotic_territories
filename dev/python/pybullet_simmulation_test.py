import time
import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

with PyBulletClient() as client:
    urdf_path = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\02_Production\01_SetUp\20250105_setup_working\ur_description\urdf\ur20.urdf"
    urdf_filepath = compas_fab.get(urdf_path)
    robot = client.load_robot(urdf_filepath)

    time.sleep(15)