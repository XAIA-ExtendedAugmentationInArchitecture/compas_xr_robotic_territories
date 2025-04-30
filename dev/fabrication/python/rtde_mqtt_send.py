from control import fabrication as rtde
import time
from compas_eve import Subscriber
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage
from compas.geometry import Frame, Transformation 
from compas_robots import Configuration
from compas.data import json_load, json_dump

def trajectory_points_to_configs(trajectory_points):
    configs = [] 
    for trajectory_point in trajectory_points:
        config = Configuration(trajectory_point.joint_values, trajectory_point.joint_types, trajectory_point.joint_names)
        configs.append(config)
    return configs

def handle_mimic_request(msg: RealtimeMimicRequestMessage):
    print(f"[Realtime Mimic Request] Frame: {msg.requested_robot_frame} Robot Name: {msg.robot_name}, Message: {msg.message}, Header: {msg.header}, InitialRequest: {msg.initial_request}")
    
    transform_frames_from_incomming_message(msg.requested_robot_frame, msg.message, msg.robot_name, msg.header.device_id, msg.initial_request)

def load_transformations_from_file(file_path):
    # Load the transformations from the JSON file
    transform_dict = json_load(file_path)
    inverse_transform = transform_dict["inverse"]
    transform = transform_dict["transform"]
    return inverse_transform, transform

def transform_frames_from_incomming_message(frame, message, robot_name, device_id, initial_reaquest):

    global transformation_testing_dict

    TX_FILEPATH = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\mqtt_transformations.json"
    inverse_transform, transform = load_transformations_from_file(TX_FILEPATH)

    tx_frame = frame.transformed(transform)
    inverse_frame = frame.transformed(inverse_transform)

    if initial_reaquest:
        transformation_testing_dict = {}

    transformation_testing_dict[message] = {
        "inv_frame": inverse_frame,
        "frame": frame,
        "tx_frame": tx_frame
    }

    dump_file = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\received_frames.json"
    json_dump(data=transformation_testing_dict, fp=dump_file, pretty=True)

    rtde.move_to_target(inverse_frame, speed=SPEED, accel=ACCELERATION, nowait=False, ip=IP)

    print(f"Received Message: Sending Frame Number {message} to {robot_name} as requested by device ID {device_id}")


#Constants RTDE
SPEED = 0.06
ACCELERATION = 0.10
IP = "192.168.1.10"

#CONSTANTS MQTT
TOPIC_BASE = "robotic_territories/real_time_mimic_request/"
PROJECT_NAME = "robotic_territories_testing_base_frame"
BROKER = "broker.hivemq.com"
# BROKER = "localhost"
PORT = 1883


transformation_testing_dict = {}

# Use the data to execute the printpath
if __name__ == "__main__":

    print("RTDE Send Script")
    topic_str = f"{TOPIC_BASE}{PROJECT_NAME}"

    topic = Topic(topic_str, RealtimeMimicRequestMessage)
    server = MqttTransport(BROKER, PORT)

    subcriber = Subscriber(topic, callback=handle_mimic_request, transport=server)
    subcriber.subscribe()

    print("Waiting for messages, press CTRL+C to cancel")

    while True:
        time.sleep(1)
    
    # SET THE FILE NAME OF THE TRAJECTORY YOU WANT TO SEND
    # trajectories_file_name = "trajectories_testing_raj_fixed.json"
    # # trajectories_file_name = "trajectories_testing_2.json"

    # # Get the path to the trajectory you want to send
    # script_dir = os.path.dirname(os.path.abspath(__file__))
    # trajectories_path = os.path.join(script_dir, "trajectories", trajectories_file_name)

    # # Load the trajectory from the json file
    # trajectories = json_load(trajectories_path)

    # # Combine the trajectory points into a list of configurations
    # combined_trajectory_points = []
    # for trajectory in trajectories:
    #     combined_trajectory_points.extend(trajectory.points)
    # configs = trajectory_points_to_configs(combined_trajectory_points)
    
    # #Initialize the RTDE control
    # rtde.send_to_single_trajectory(configs, SPEED, ACCELERATION, RADIUS, nowait=True, ip=IP)

    # #Loading only the configurations for testing
    # # trajectories_file_name = "just_joint_values.json"

    # # # # Get the path to the trajectory you want to send
    # # script_dir = os.path.dirname(os.path.abspath(__file__))
    # # trajectories_path = os.path.join(script_dir, "trajectories", trajectories_file_name)
    # # configs = json_load(trajectories_path)

    # rtde.send_to_single_trajectory_only_joint_values(configs, SPEED, ACCELERATION, RADIUS, nowait=True, ip=IP)