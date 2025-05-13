import os
from compas_fab.backends import RosClient
from compas.data import json_load
from datetime import datetime
from compas.data import json_dump

ip = '127.0.0.1'
port = 9090
script_dir = os.path.dirname(os.path.abspath(__file__))

# Load the frames from the json file
frames_relative_path = os.path.join(script_dir, "frames.json")
frames_file_path = os.path.normpath(frames_relative_path)
frames = json_load(frames_file_path)

# Output ik configurations to a json file
output_relative_path = os.path.join(script_dir, "ik_configurations_ros.json")
output_file_path = os.path.normpath(output_relative_path)

# Connect to ROS
ros_client = RosClient(ip, port)
ros_client.run(5)
is_connected = ros_client.is_connected if ros_client else False

if is_connected:
    robot = ros_client.load_robot(load_geometry=True, precision=12)
    fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\random\robot_vs_geo.json"
    json_dump(data=robot, fp=fp, pretty=True)
    if robot:
        robot.client = ros_client
        
        ik_information = {}
        ik_configurations = []

        for i, frame in enumerate(frames):
            if i == 0:
                start_config = robot.zero_configuration()
            else:
                start_config = ik_configurations[-1]

            ik_configuration = robot.inverse_kinematics(frame, start_configuration=start_config)
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