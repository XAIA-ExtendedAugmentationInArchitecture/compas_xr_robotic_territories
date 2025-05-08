from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import RosClient
from compas_xr.mqtt import RealtimeMimicRequestMessage
from compas_xr.mqtt import RealtimeMimicResultMessage
from control import fabrication as rtde
import compas_rrc as rrc
from compas.data import json_load, json_dump

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
            fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\test_config_vis.json"
            json_dump(self.ik_solutions, fp=fp, pretty=True)
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
        # config = rtde.get_config(self.robot_ip)
        # return config
        config_zero = self.robot.zero_configuration()
        return config_zero

    def _execute_motion(self, config: Configuration):
        print(f"URRealtimeMimicHandler: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

class ABBRealtimeMimicHandler(RealtimeMimicROSHandler):
    
    def __init__(self, robot_name, robot_ip, abb_client, ros_ip='127.0.0.1', ros_port=9090, speed=100, nowait=False):
        super().__init__(robot_name, robot_ip, ros_ip, ros_port)
        self.speed = speed
        self.nowait = nowait

        self.ros_rrc = rrc.RosClient()
        self.ros_rrc.run()

        #TODO: CHECK NAME '/robLL_track' IS CORRECT
        self.abb = rrc.AbbClient(self.ros_rrc, abb_client)
        print(f"ABBRealtimeMimicHandler: [{robot_name}] Connected to ABB controller via RRC")

    def _get_current_configuration(self):
        robot_joints, _ = self.abb.send_and_wait(rrc.GetJoints())
        print(f"[{self.robot_name}] Current joints from controller: {robot_joints}")
        return self.robot.zero_configuration()

    def _execute_motion(self, config: Configuration):
        print(f"ABBRealtimeMimicHandler: [{self.robot_name}] Executing motion: {config.joint_values}")
        
        # Convert radians to degrees for ABB
        joint_values_deg = [v * 180.0 / 3.1415926 for v in config.joint_values]
        rax = rrc.RobotJoints(*joint_values_deg)
        ext_axes = [0.0] * 6  # TODO: Placeholder for external axes

        result = self.abb.send_and_wait(rrc.MoveToJoints(rax, ext_axes, self.speed, rrc.Zone.FINE))
        print(f"[{self.robot_name}] Motion complete: {result}")

    def close(self):
        self.ros_rrc.close()
        self.ros_rrc.terminate()
        print(f"[{self.robot_name}] ABB RRC connection closed")

