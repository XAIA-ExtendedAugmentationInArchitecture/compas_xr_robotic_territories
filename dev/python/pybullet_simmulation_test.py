import os
import time

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

from compas.data import json_load
from compas.data import json_dump
from datetime import datetime

script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct relative paths to urdf
urdf_relative_path = os.path.join(script_dir, "../scripts/urdf/ur_description/urdf/ur20.urdf")
urdf_file_path = os.path.normpath(urdf_relative_path)

# Load the frames from the json file
frames_relative_path = os.path.join(script_dir, "frames.json")
frames_file_path = os.path.normpath(frames_relative_path)
frames = json_load(frames_file_path)

# Output ik configurations to a json file
output_relative_path = os.path.join(script_dir, "ik_configurations_pybullet.json")
output_file_path = os.path.normpath(output_relative_path)

with PyBulletClient() as client:
    urdf_file = compas_fab.get(urdf_file_path)
    robot = client.load_robot(urdf_file)
    time.sleep(1)

    ik_information = {}
    ik_configurations = []
    
    for i, frame in enumerate(frames):
        if i == 0:
            start_config = robot.zero_configuration()
        else:
            start_config = ik_configurations[-1]
    
        ik_configuration = robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config)
        ik_configurations.append(ik_configuration)

        timestamp = datetime.now().isoformat()
        ik_information[i] = {
        "target_configuration": ik_configuration,
        "frame": frame,
        "start_configuration": start_config,
        "time": timestamp
        }

    print(ik_configurations)
    json_dump(ik_information, output_file_path, pretty=True)