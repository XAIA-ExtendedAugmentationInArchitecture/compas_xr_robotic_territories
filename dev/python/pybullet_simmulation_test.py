import os
import time

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

# Construct file path to urdf file
script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_relative_path = os.path.join(script_dir, "../scripts/urdf/ur_description/urdf/ur20.urdf")
urdf_file_path = os.path.normpath(urdf_relative_path)

# Load robot in PyBullet
with PyBulletClient() as client:
    urdf_file = compas_fab.get(urdf_file_path)
    robot = client.load_robot(urdf_file)
    time.sleep(15)