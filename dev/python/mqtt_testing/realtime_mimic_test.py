import time

from compas_eve import Message
from compas_eve import Publisher
from compas_eve import Subscriber
from compas_eve import Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage


def handle_mimic_request(msg: RealtimeMimicRequestMessage):
    print(f"[Realtime Mimic Request] Robot Name: {msg.robot_name}, Message: {msg.message}, Header: {msg.header}")

TOPIC_BASE = "robotic_territories/real_time_mimic_request/"
PROJECT_NAME = "robotic_territories_testing_base_frame"
topic_str = f"{TOPIC_BASE}{PROJECT_NAME}"

topic = Topic(topic_str, RealtimeMimicRequestMessage)
server = MqttTransport("localhost", 1883)

subcriber = Subscriber(topic, callback=handle_mimic_request, transport=server)
subcriber.subscribe()

print("Waiting for messages, press CTRL+C to cancel")

while True:
    time.sleep(1)