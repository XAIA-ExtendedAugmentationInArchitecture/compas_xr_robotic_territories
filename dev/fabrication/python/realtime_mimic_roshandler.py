import time
from compas.geometry import Frame
from compas_robots import Configuration
from compas.data import json_dump

from compas_fab.backends import RosClient
from compas_eve import Subscriber, Publisher, Topic
from compas_eve.mqtt import MqttTransport
from compas_xr.mqtt import RealtimeMimicRequestMessage, RealtimeMimicResultMessage


class RealtimeMimicROSHandler:

    def __init__(self, robot_name, robot_ip, ros_ip='127.0.0.1', ros_port=9090,
                 broker='localhost', mqtt_port=1883):
        self.robot_name = robot_name
        self.robot_ip = robot_ip
        self.ros_ip = ros_ip
        self.ros_port = ros_port
        self.broker = broker
        self.mqtt_port = mqtt_port

        self.ros_client = RosClient(ros_ip, ros_port)
        self.ros_client.run(5)
        if not self.ros_client.is_connected:
            raise Exception(f"[{robot_name}] Failed to connect to ROS at {ros_ip}:{ros_port}")

        self.robot = self._load_robot()
        self.ik_solutions = []
        self._got_initial_config = False

        self.mqtt = MqttTransport(broker, mqtt_port)
        self._setup_subscriber()
        self._setup_publisher()

        print(f"[{robot_name}] Handler initialized")

    def _load_robot(self):
        robot = self.ros_client.load_robot(load_geometry=False, precision=12)
        robot.client = self.ros_client
        return robot

    def _setup_subscriber(self):
        topic_str = f"robotic_territories/{self.robot_name}/mimic"
        topic = Topic(topic_str, RealtimeMimicRequestMessage)
        self.subscriber = Subscriber(topic, callback=self._on_message, transport=self.mqtt)
        self.subscriber.subscribe()
        print(f"[{self.robot_name}] Subscribed to: {topic_str}")

    def _setup_publisher(self):
        topic = Topic("robotic_territories/ik_results", RealtimeMimicResultMessage)
        self.publisher = Publisher(topic, transport=self.mqtt)

    def _get_start_config(self):
        if not self._got_initial_config:
            self._got_initial_config = True
            return self.robot.zero_configuration()
        return self.ik_solutions[-1] if self.ik_solutions else self.robot.zero_configuration()

    def _parse_frame_from_msg(self, msg: RealtimeMimicRequestMessage) -> Frame:
        # This assumes msg.requested_robot_frame is already a compas Frame
        return msg.requested_robot_frame or Frame.worldXY()

    def _compute_ik(self, frame: Frame, start_config: Configuration) -> Configuration:
        return self.robot.inverse_kinematics(frame, start_configuration=start_config)

    def _execute_motion(self, configuration: Configuration):
        # Placeholder: override this method for specific robot execution (RTDE, RobotWare, etc.)
        print(f"[{self.robot_name}] Executing motion (not implemented): {configuration.joint_values}")

    def _on_message(self, msg: RealtimeMimicRequestMessage):
        print(f"[{self.robot_name}] Received: {msg.message} from device {msg.header.device_id}")

        frame = self._parse_frame_from_msg(msg)
        start_config = self._get_start_config()

        ik_config = self._compute_ik(frame, start_config)
        self.ik_solutions.append(ik_config)

        self._execute_motion(ik_config)

        result_msg = RealtimeMimicResultMessage(
            robot_name=self.robot_name,
            message=f"IK solution computed for {msg.message}",
            configuration=ik_config,
            header=msg.header
        )
        self.publisher.publish(result_msg)