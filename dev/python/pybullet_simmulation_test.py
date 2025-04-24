import os
import time

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

from compas.data import json_load
from compas.data import json_dump
from datetime import datetime

#TODO: CHECK THIS LATER.... TRY THE STEP SIMULATION IN THE BACKEND.....

script_dir = os.path.dirname(os.path.abspath(__file__))

# Construct relative paths to urdf
urdf_relative_path = os.path.join(script_dir, "../scripts/urdf/ur_description/urdf/ur20.urdf")
urdf_file_path = os.path.normpath(urdf_relative_path)

# Load the frames from the json file
frames_relative_path = os.path.join(script_dir, "frames_same.json")
frames_file_path = os.path.normpath(frames_relative_path)
frames = json_load(frames_file_path)

# Output ik configurations to a json file
output_relative_path = os.path.join(script_dir, "ik_configurations_pybullet.json")
output_file_path = os.path.normpath(output_relative_path)

with PyBulletClient() as client:
    urdf_file = compas_fab.get(urdf_file_path)
    robot = client.load_robot(urdf_file)
    # print(dir(robot))
    # time.sleep(1)

    ik_information = {}
    ik_configurations = []
    random_config_frames = []
    configuration_data = []

    for i, frame in enumerate(frames):
        if i == 0:
            start_config = robot.zero_configuration()
        else:
            start_config = ik_configurations[-1]
        # state = robot.__getstate__()
        # print ("STATE Pre:", state["_current_ik"])
        ik_configuration = robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config)
        client.step_simulation()
        time.sleep(0.5)
        configuration_data.append(ik_configuration.__data__)
        # test = robot.get_group_configuration(robot.main_group_name, ik_configuration)
        # print(test)
        # state2 = robot.__getstate__()
        # print ("STATE post:", state["_current_ik"])
        time.sleep(5)
        frame = robot.forward_kinematics(ik_configuration)
        random_config_frames.append(frame)
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
    json_dump(random_config_frames, os.path.join(script_dir, "random_config_frames.json"), pretty=True)
    json_dump(configuration_data, os.path.join(script_dir, "configuration_data_testing.json"), pretty=True)