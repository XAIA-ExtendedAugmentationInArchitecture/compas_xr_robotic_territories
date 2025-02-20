import sys
import os
from control import fabrication as rtde

# from ..rhino.fabrication.control import fabrication as rtde
from compas_robots import Configuration
from compas.data import json_load

def trajectory_points_to_configs(trajectory_points):
    configs = [] 
    for trajectory_point in trajectory_points:
        config = Configuration(trajectory_point.joint_values, trajectory_point.joint_types, trajectory_point.joint_names)
        configs.append(config)
    return configs


SPEED = 0.06
ACCELERATION = 0.10
RADIUS = 0.006
IP = "192.168.1.10"

# Use the data to execute the printpath
if __name__ == "__main__":

    # SET THE FILE NAME OF THE TRAJECTORY YOU WANT TO SEND
    trajectories_file_name = "trajectories_testing_2.json"

    # Get the path to the trajectory you want to send
    script_dir = os.path.dirname(os.path.abspath(__file__))
    trajectories_path = os.path.join(script_dir, "trajectories", trajectories_file_name)

    # Load the trajectory from the json file
    trajectories = json_load(trajectories_path)

    # Combine the trajectory points into a list of configurations
    combined_trajectory_points = []
    for trajectory in trajectories:
        combined_trajectory_points.extend(trajectory.points)
    configs = trajectory_points_to_configs(combined_trajectory_points)
    
    #Initialize the RTDE control
    rtde.send_to_single_trajectory(configs, SPEED, ACCELERATION, RADIUS, nowait=True, ip=IP)