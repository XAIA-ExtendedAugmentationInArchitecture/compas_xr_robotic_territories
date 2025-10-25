import sys
import os
from control import fabrication as rtde
import time

# from ..rhino.fabrication.control import fabrication as rtde
from compas_robots import Configuration
from compas.data import json_load, json_dump

def trajectory_points_to_configs(trajectory_points):
    configs = [] 
    for trajectory_point in trajectory_points:
        config = Configuration(trajectory_point.joint_values, trajectory_point.joint_types, trajectory_point.joint_names)
        configs.append(config)
    return configs


def send_simple_pick_and_place_with_vac_rt(trajectories_list, speed=0.06, acceleration=0.10, radius=0.006, vac_io =1, ip="192.168.1.10"):
    pick_trajectory = trajectories_list[0]
    place_trajectory = trajectories_list[1]

    # Send the pick trajectory
    pick_trajectory_configs = trajectory_points_to_configs(pick_trajectory.points)
    rtde.send_to_single_trajectory(pick_trajectory_configs, speed, acceleration, radius, nowait=True, ip=ip)

    # Activate the gripper
    rtde.set_tool_digital_io(vac_io, 1, ip=ip)
    time.sleep(2)  # Wait for the pick to complete

    place_trajectory_configs = trajectory_points_to_configs(place_trajectory.points)
    rtde.send_to_single_trajectory(place_trajectory_configs, speed, acceleration, radius, nowait=True, ip=ip)
    # Activate the gripper
    rtde.set_tool_digital_io(vac_io, 0, ip=ip)
    time.sleep(2)  # Wait for the pick to complete

# def SEND_TO_STATIC_CONFIG_FOR_INFERENCE(speed, accel, ur_c):
#     joe_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
#     joe_joint_types = [0, 0, 0, 0, 0, 0]
#     # joe_start_config_values = [-0.10790457, -0.29844413,  0.06922359, -1.36258329, -1.5687577 , -1.64426868]
#     joe_start_config_values = [-0.10790457000000009, -1.26844413, 1.0792235899999998, -2.8625832899999999, -1.5687576999999999, -1.6442686799999999]
#     joe_start_configuration = Configuration(joint_values=joe_start_config_values, joint_names=joe_joint_names, joint_types=joe_joint_types)
#     move_to_joints_urc(joe_start_configuration, speed, accel, True, ur_c)
#     print("SENT TO STATIC CONFIG")
#     config = Configuration(config_start, joint_types, joint_names)
#     rtde.move_to_joints(config=config, speed=SPEED, accel=ACCELERATION, nowait=True, ip=IP)

SPEED = 0.06
ACCELERATION = 0.10
RADIUS = 0.006
IP = "192.168.1.10"

# Use the data to execute the printpath
if __name__ == "__main__":

    # Testing digital IO
    # rtde.set_tool_digital_io(1, 1, ip="192.168.1.10")
    # time.sleep(2)
    # rtde.set_tool_digital_io(1, 0, ip="192.168.1.10")


    # # Send to start configuration
    joe_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joe_joint_types = [0, 0, 0, 0, 0, 0]
    # joe_start_config_values = [-0.10790457, -0.29844413,  0.06922359, -1.36258329, -1.5687577 , -1.64426868]
    joe_start_config_values = [-0.10790457000000009, -1.26844413, 1.0792235899999998, -2.8625832899999999, -1.5687576999999999, -1.6442686799999999]
    joe_start_configuration = Configuration(joint_values=joe_start_config_values, joint_names=joe_joint_names, joint_types=joe_joint_types)
    json_dump(data=joe_start_configuration.__data__, fp=os.path.join(os.path.dirname(__file__), "joe_start_configuration.json"), pretty=True)
    # rtde.move_to_joints(config=joe_start_configuration, speed=SPEED, accel=ACCELERATION, nowait=True, ip=IP)

    # # Send to a trajectory from the trajectories folder
    # # SET THE FILE NAME OF THE TRAJECTORY YOU WANT TO SEND
    # trajectories_file_name = "scripted_policy_test.json"
    # trajectories_file_name ="20250815_scripted_policy_test_from_simple_target.json"
    # trajectories_file_name = "20250819_joe_real_test.json"

    # # Get the path to the trajectory you want to send
    # script_dir = os.path.dirname(os.path.abspath(__file__))
    # trajectories_path = os.path.join(script_dir, "trajectories", trajectories_file_name)

    # # Load the trajectory from the json file
    # trajectory = json_load(trajectories_path)

    # # Convert the trajectory points to configurations
    # configs = trajectory_points_to_configs(trajectory.points)
    # rtde.send_to_single_trajectory(configs, SPEED, ACCELERATION, RADIUS, nowait=True, ip=IP)

    # Send to a trajectory from the trajectories folder
    # SET THE FILE NAME OF THE TRAJECTORY YOU WANT TO SEND
    # trajectories_file_name = "20250819_double_trajectory.json"

    # Get the path to the trajectory you want to send
    # script_dir = os.path.dirname(os.path.abspath(__file__))
    # trajectories_path = os.path.join(script_dir, "trajectories", trajectories_file_name)

    # Load the trajectory from the json file
    # trajectories_list = json_load(trajectories_path)

    # Convert the trajectory points to configurations
    # print(len(trajectories_list))
    # send_simple_pick_and_place_with_vac_rt(trajectories_list, speed=SPEED, acceleration=ACCELERATION, radius=RADIUS, vac_io=1, ip=IP)