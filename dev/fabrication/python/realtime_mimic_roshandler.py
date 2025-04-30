from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import RosClient
from compas_xr.mqtt import RealtimeMimicRequestMessage
from compas_xr.mqtt import RealtimeMimicResultMessage
from control import fabrication as rtde
import compas_rrc

class RealtimeMimicROSHandler:

    def __init__(self, robot_name, robot_ip, ros_ip='127.0.0.1', ros_port=9090):
        self.robot_name = robot_name
        self.robot_ip = robot_ip
        self.ros_client = RosClient(ros_ip, ros_port)
        self.ros_client.run(5)

        if not self.ros_client.is_connected:
            raise ConnectionError(f"RealtimeMimicROSHandler: [{robot_name}] Could not connect to ROS at {ros_ip}:{ros_port}")

        self.robot = self._load_robot()
        self.ik_solutions = []
        self._got_initial_config = False
        print(f"RealtimeMimicROSHandler : [{robot_name}] Handler initialized")

    def _load_robot(self):
        robot = self.ros_client.load_robot(load_geometry=False, precision=12)
        robot.client = self.ros_client
        return robot

    def _get_current_configuration(self):
        # This method should be implemented to get the current configuration of the robot
        # For example, using RTDE or another method to get the joint values
        raise NotImplementedError("This method should be implemented to get the current configuration of the robot.")

    def handle_msg_request(self, msg: RealtimeMimicRequestMessage) -> Configuration:
        print(f"RealtimeMimicROSHandler : [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame
        if msg.initial_request:
            print(f"RealtimeMimicROSHandler : [{self.robot_name}] Initial request received setting IK solution to empty list")
            self.ik_solutions = []
        start_config = self._get_current_configuration() if not self.ik_solutions else self.ik_solutions[-1]

        try:
            ik_config = self.robot.inverse_kinematics(frame, start_configuration=start_config)
            self.ik_solutions.append(ik_config)
            self._execute_motion(ik_config)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicROSHandler : [{self.robot_name}] IK computation failed: {e}")
            return None

    def _execute_motion(self, config: Configuration):
        print(f" RealtimeMimicROSHandler : [{self.robot_name}] (Sim) Executing: {config.joint_values}")


class URRealtimeMimicHandler(RealtimeMimicROSHandler):
    
    def __init__(self, robot_name, robot_ip, ros_ip='127.0.0.1', ros_port=9090, speed=0.6, acceleration=0.1, radius=0.006, nowait=False):
        super().__init__(robot_name, robot_ip, ros_ip, ros_port)
        self.speed = speed
        self.acceleration = acceleration
        self.radius = radius
        self.nowait = nowait
        print(f"URRealtimeMimicHandler: [{robot_name}] UR handler initialized")

    def _get_current_configuration(self):
        config = rtde.get_config_TEST(self.robot_ip)
        config_zero = self.robot.zero_configuration()
        return config_zero

    def _execute_motion(self, config: Configuration):
        print(f"URRealtimeMimicHandler: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)



