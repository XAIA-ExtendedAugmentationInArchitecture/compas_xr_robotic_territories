from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage

from realtime_mimic_roshandler import URRealtimeMimicHandler
from realtime_mimic_pbhandler import URRealtimeMimicHandlerPyB
from compas.data import json_load, json_dump
import os

#TODO: Tranfromation is still comming from GH explort load... it should be from streaming the .py data
class RobotManager:

    def __init__(self, project_name, broker='localhost', mqtt_port=1883):
        self.mqtt = MqttTransport(broker, mqtt_port)

        # self.handlers = {
        #     # "UR3": URRealtimeMimicHandler(
        #     #     robot_name="ur3",
        #     #     robot_ip="192.168.0.200",     # TODO: UPDATE WITH UR3 IP
        #     #     ros_ip="127.0.0.1",
        #     #     ros_port=11312
        #     # ),
        #     "UR20": URRealtimeMimicHandler("UR20", "192.168.1.10")
        # }

        self.handlers = {
            # "UR3": URRealtimeMimicHandler(
            #     robot_name="ur3",
            #     robot_ip="192.168.0.200",     # TODO: UPDATE WITH UR3 IP
            #     ros_ip="127.0.0.1",
            #     ros_port=11312
            # ),
            #TODO: TESTING BASE SLOW SPEED
            # "UR20": URRealtimeMimicHandlerPyB("UR20", "192.168.1.10", 
            #                                     r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf",
            #                                     r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.srdf",
            #                                     radius=0.015,
            #                                     nowait=False)
            
            # #TODO: TESTING MEDIUM SPEED
            # "UR20": URRealtimeMimicHandlerPyB("UR20", "192.168.1.10", 
            #                         r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf",
            #                         r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.srdf",
            #                         speed=1.0, acceleration=1.0, nowait=True, radius=0.015)

            # #TODO: TESTING FASTER SPEED
            "UR20": URRealtimeMimicHandlerPyB("UR20", "192.168.1.10", 
                                    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf",
                                    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.srdf",
                                    speed=1.5, acceleration=2.0, radius=0.025, nowait=False)

            # #TODO: TESTING FASTEST SPEED
            # "UR20": URRealtimeMimicHandlerPyB("UR20", "192.168.1.10", 
            #                         r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf",
            #                         r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.srdf",
            #                         speed=2.0, acceleration=3.0, nowait=True, radius=0.04)
        }

        result_topic = Topic(f"robotic_territories/real_time_mimic_result/{project_name}", RealtimeMimicResultMessage)
        self.publisher = Publisher(result_topic, transport=self.mqtt)

        request_topic = Topic(f"robotic_territories/real_time_mimic_request/{project_name}", RealtimeMimicRequestMessage)
        self.subscriber = Subscriber(request_topic, callback=self._on_message, transport=self.mqtt)
        self.subscriber.subscribe()

        print(f"RobotManager : [RobotManager] Subscribed to: robotic_territories/real_time_mimic_request/{project_name}")

    #TODO: ADD THE CORRECT TOOL AS AN ACM...

    def _load_transformations_from_file(self, file_path):
        # Load the transformations from the JSON file
        transform_dict = json_load(file_path)
        inverse_transform = transform_dict["inverse"]
        transform = transform_dict["transform"]
        return inverse_transform, transform

    def _transform_incoming_requested_frame(self, frame):

        TX_FILEPATH = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\mqtt_transformations.json"
        inverse_transform, transform = self._load_transformations_from_file(TX_FILEPATH)
        inverse_frame = frame.transformed(inverse_transform)
        return inverse_frame

    def _transform_out_robot_baseframe(self, frame):
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
    
    def _on_message(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name
        if robot_name not in self.handlers:
            print(f"RobotManager : [RobotManager] No handler found for robot '{robot_name}'")
            return

        handler = self.handlers[robot_name]
        self._save_requested_frame(msg)

        msg.requested_robot_frame = self._transform_incoming_requested_frame(msg.requested_robot_frame)
        ik_config = handler.handle_msg_request_compas_fab_itter(msg)
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


PROJECT_NAME = "robotic_territories_testing_base_frame"
BROKER = "broker.hivemq.com"
# BROKER = "localhost"
requested_frames = []

if __name__ == "__main__":
    manager = RobotManager(PROJECT_NAME, broker=BROKER)
    print("[RobotManager] Listening for mimic requests... (Press Ctrl+C to exit)")
    try:
        while True:
            pass  # Keep the process alive
    except KeyboardInterrupt:
        print("[RobotManager] Shutdown requested.")
