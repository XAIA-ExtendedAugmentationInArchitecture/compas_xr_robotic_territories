from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import RosClient
from compas_xr.mqtt import RealtimeMimicRequestMessage

class RealtimeMimicROSHandler:

    def __init__(self, robot_name, ip, ros_ip='127.0.0.1', ros_port=9090):
        self.robot_name = robot_name
        self.robot_ip = ip
        self.ros_client = RosClient(ros_ip, ros_port)
        self.ros_client.run(5)

        if not self.ros_client.is_connected:
            raise ConnectionError(f"[{robot_name}] Could not connect to ROS at {ros_ip}:{ros_port}")

        self.robot = self._load_robot()
        self.ik_solutions = []
        self._got_initial_config = False
        print(f"[{robot_name}] Handler initialized")

    def _load_robot(self):
        robot = self.ros_client.load_robot(load_geometry=False, precision=12)
        robot.client = self.ros_client
        return robot

    def handle(self, msg: RealtimeMimicRequestMessage) -> Configuration:
        print(f"[{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame or Frame.worldXY()
        start_config = self.robot.zero_configuration() if not self.ik_solutions else self.ik_solutions[-1]

        try:
            ik_config = self.robot.inverse_kinematics(frame, start_configuration=start_config)
            self.ik_solutions.append(ik_config)
            self._execute_motion(ik_config)
            return ik_config
        except Exception as e:
            print(f"[{self.robot_name}] IK computation failed: {e}")
            return None

    def _execute_motion(self, config: Configuration):
        print(f"[{self.robot_name}] (Sim) Executing: {config.joint_values}")

