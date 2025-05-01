from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage

from realtime_mimic_roshandler import RealtimeMimicROSHandler, URRealtimeMimicHandler
from compas.data import json_load, json_dump

#TODO: Tranfromation is still comming from GH explort load... it should be from streaming the .py data
class RobotManager:

    def __init__(self, project_name, broker='localhost', mqtt_port=1883):
        self.mqtt = MqttTransport(broker, mqtt_port)

        self.handlers = {
            # "UR3": URRealtimeMimicHandler(
            #     robot_name="ur3",
            #     robot_ip="192.168.0.200",     # TODO: UPDATE WITH UR3 IP
            #     ros_ip="127.0.0.1",
            #     ros_port=11312
            # ),
            "UR20": URRealtimeMimicHandler("UR20", "192.168.1.10")
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


    def _on_message(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name
        if robot_name not in self.handlers:
            print(f"RobotManager : [RobotManager] No handler found for robot '{robot_name}'")
            return

        handler = self.handlers[robot_name]
        msg.requested_robot_frame = self._transform_incoming_requested_frame(msg.requested_robot_frame)
        ik_config = handler.handle_msg_request(msg)

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

if __name__ == "__main__":
    manager = RobotManager(PROJECT_NAME, broker=BROKER)
    print("[RobotManager] Listening for mimic requests... (Press Ctrl+C to exit)")
    try:
        while True:
            pass  # Keep the process alive
    except KeyboardInterrupt:
        print("[RobotManager] Shutdown requested.")
