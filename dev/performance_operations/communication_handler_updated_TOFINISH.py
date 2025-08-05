from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage, MimicTrajectoryRequestMessage, MimicTrajectoryResultMessage

# from robots.com_handlers.realtime_mimic_roshandler import URRealtimeMimicHandler
from robots.com_handlers.realtime_mimic_pbhandler import URRealtimeMimicHandlerPyB
from compas.data import json_load, json_dump
import os

#TODO: Tranfromation is still comming from GH explort load... it should be from streaming the .py data
#TODO: Update to work for planning other requests for Mimic and Relatime Mimic

#TODO: FIX ME JOSEPH.

class CommunicationManager:

    def __init__(self, project_name, robot_name, broker='localhost', mqtt_port=1883):
        self.mqtt = MqttTransport(broker, mqtt_port)
        self.project_name = project_name
        self.robot_name = robot_name

        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.project_fp_config = os.path.join(self.script_dir, "project_config.json")
        self._project_config_dict = json_load(self.project_fp_config)

        self._urdf_filepath = self._project_config_dict["urdf_fps"][robot_name]["urdf"]
        self._srdf_filepath = self._project_config_dict["urdf_fps"][robot_name]["srdf"]

        self.handler = self._load_handler(robot_name, self._urdf_filepath, self._srdf_filepath)


        #Realtime Mimic Request and Result Handlers
        realtime_mimic_result_topic = Topic(f"robotic_territories/real_time_mimic_result/{project_name}/{robot_name}", RealtimeMimicResultMessage) # TODO: Pull project name from config dir
        self.realtime_publisher = Publisher(realtime_mimic_result_topic, transport=self.mqtt)

        realtime_mimic_request_topic = Topic(f"robotic_territories/real_time_mimic_request/{project_name}/{robot_name}", RealtimeMimicRequestMessage) # TODO: Pull project name from config dir
        self.realtime_subscriber = Subscriber(realtime_mimic_request_topic, callback=self._on_message_realtime_mimic, transport=self.mqtt)
        self.realtime_subscriber.subscribe()

        #User Initiated Mimic Request and Result Handlers
        user_initiated_mimic_result_topic = Topic(f"robotic_territories/mimic_result/{project_name}/{robot_name}", MimicTrajectoryResultMessage) # TODO: Pull project name from config dir
        self.user_initiated_publisher = Publisher(user_initiated_mimic_result_topic, transport=self.mqtt)

        user_initiated_mimic_request_topic = Topic(f"robotic_territories/mimic_request/{project_name}/{robot_name}", MimicTrajectoryResultMessage) # TODO: Pull project name from config dir
        self.user_initiated_subscriber = Subscriber(user_initiated_mimic_request_topic, callback=self._on_message_user_initiated_mimic, transport=self.mqtt)
        self.user_initiated_subscriber.subscribe()

        print(f"RobotManager : [RobotManager] Subscribed to: robotic_territories mimic topics for project '{project_name}' and robot '{robot_name}'")

    #TODO: ADD THE CORRECT TOOL AS AN ACM...

    def _load_handler(self, robot_name, urdf_filepath, srdf_filepath):
        if robot_name == "UR20" or robot_name == "UR31" or robot_name == "UR32":
            #Load the information & Handler for UR robots
            pass
        else:
            #Load the information & Handler for ABB robots
            pass

    # TODO: Adjust code below ###########################################################################################################

    def _load_transformations_from_file(self, file_path): #TODO: Fix Transformation file path
        # Load the transformations from the JSON file
        transform_dict = json_load(file_path)
        inverse_transform = transform_dict["inverse"]
        transform = transform_dict["transform"]
        return inverse_transform, transform

    def _transform_incoming_requested_frame(self, frame): #TODO: Fix Transformation file path

        TX_FILEPATH = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\mqtt_transformations.json"
        inverse_transform, transform = self._load_transformations_from_file(TX_FILEPATH)
        inverse_frame = frame.transformed(inverse_transform)
        return inverse_frame

    def _transform_out_robot_baseframe(self, frame): #TODO: Fix Transformation file path
        TX_FILEPATH = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\mqtt_transformations.json"
        inverse_transform, transform = self._load_transformations_from_file(TX_FILEPATH)
        tx_frame = frame.transformed(transform)
        return tx_frame

    def _save_requested_frame(self, msg: RealtimeMimicRequestMessage):
        global requested_frames

        if msg.initial_request:
            requested_frames = []
            print("RobotManager : [RobotManager] Initial request received, cleared requested_frames.")
        else:
            requested_frames.append(msg.requested_robot_frame)
            print("RobotManager : [RobotManager] Appended requested frame to global list.")

        file_path = os.path.join(
            os.path.dirname(__file__),
            "requested_frames_pybullet.json"
        )
        json_dump(requested_frames, file_path)
        print(f"RobotManager : [RobotManager] Dumped requested_frames to {file_path}")
    
    def _on_message_realtime_mimic(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name
        if robot_name not in self.handlers:
            print(f"RobotManager : [RobotManager] No handler found for robot '{robot_name}'")
            return

        handler = self.handlers[robot_name]
        self._save_requested_frame(msg)

        msg.requested_robot_frame = self._transform_incoming_requested_frame(msg.requested_robot_frame)
        ik_config = handler.handle_msg_request_ik_target(msg)
        # ik_config = handler.handle_msg_request_compas_fab_itter(msg)
        # ik_config = handler.handle_msg_request_recursive_solver(msg)
        # ik_config = handler.handle_msg_request(msg)

        #TODO: NEED TO TRANSFORM BACK TO ROBOT BASEFRAME, BUT JUST SEE IF IT PRINTS FIRST....

        if ik_config:
            #TODO: UPDATE THE KEEPING TRACK OF THE MESSAGES
            result = RealtimeMimicResultMessage(
                robot_name=robot_name,
                return_message=f"IK computed for {msg.message} with {robot_name} and an ik solution of {ik_config}",
            )
            self.publisher.publish(result)
            print(f"RobotManager : [RobotManager] Published IK result for robot {robot_name}")
        else:
            print(f" RobotManager : [RobotManager] No result to publish for robot {robot_name}")

    def _on_message_user_initiated_mimic(self, msg: RealtimeMimicRequestMessage):
        print(f"RobotManager : [RobotManager] Received user-initiated mimic request: {msg}")
        return  #TODO: Implement user-initiated mimic request handling

    #TODO: Adjust code above ###########################################################################################################

PROJECT_NAME = "robotic_territories_testing_base_frame" #TODO: Pull from config dir
BROKER = "broker.hivemq.com" #TODO: Pull from config dir
# BROKER = "localhost"
requested_frames = []

if __name__ == "__main__":
    manager = CommunicationManager(PROJECT_NAME, broker=BROKER)
    print("[RobotManager] Listening for mimic requests... (Press Ctrl+C to exit)")
    try:
        while True:
            pass  # Keep the process alive
    except KeyboardInterrupt:
        print("[RobotManager] Shutdown requested.")
