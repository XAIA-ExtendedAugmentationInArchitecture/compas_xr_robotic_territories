from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage

from realtime_mimic_roshandler import RealtimeMimicROSHandler


class RobotManager:
    def __init__(self, broker='localhost', mqtt_port=1883):
        self.mqtt = MqttTransport(broker, mqtt_port)

        self.handlers = {
            "ur3": RealtimeMimicROSHandler("ur3", ip="192.168.1.10", ros_port=9091),
            "ur20": RealtimeMimicROSHandler("ur20", ip="192.168.1.20", ros_port=9090),
            "abb120": RealtimeMimicROSHandler("abb120", ip="192.168.1.50", ros_port=9095)
        }

        result_topic = Topic("robotic_territories/ik_results", RealtimeMimicResultMessage)
        self.publisher = Publisher(result_topic, transport=self.mqtt)

        request_topic = Topic("robotic_territories/ik_requests", RealtimeMimicRequestMessage)
        self.subscriber = Subscriber(request_topic, callback=self._on_message, transport=self.mqtt)
        self.subscriber.subscribe()

        print("[RobotManager] Subscribed to: robotic_territories/ik_requests")

    def _on_message(self, msg: RealtimeMimicRequestMessage):
        robot_name = msg.robot_name
        if robot_name not in self.handlers:
            print(f"[RobotManager] No handler found for robot '{robot_name}'")
            return

        handler = self.handlers[robot_name]
        ik_config = handler.handle(msg)

        if ik_config:
            result = RealtimeMimicResultMessage(
                robot_name=robot_name,
                message=f"IK computed for {msg.message}",
                configuration=ik_config,
                header=msg.header
            )
            self.publisher.publish(result)
            print(f"[RobotManager] Published IK result for robot {robot_name}")
        else:
            print(f"[RobotManager] No result to publish for robot {robot_name}")


# if __name__ == "__main__":
#     manager = RobotManager()
#     print("[RobotManager] Listening for mimic requests... (Press Ctrl+C to exit)")
#     try:
#         while True:
#             pass  # Keep the process alive
#     except KeyboardInterrupt:
#         print("[RobotManager] Shutdown requested.")
