from compas_fab.backends import RosClient
from compas.data import json_load
from datetime import datetime
from compas.data import json_dump

ip = '127.0.0.1'
port = 9090

frames = json_load(r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\py_scripts\frames.json")

ros_client = RosClient(ip, port)
ros_client.run(5)
is_connected = ros_client.is_connected if ros_client else False


if is_connected:
    robot = ros_client.load_robot(load_geometry=False, precision=12)
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
    json_dump(ik_information, r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\py_scripts\ik_configurations.json", pretty=True)