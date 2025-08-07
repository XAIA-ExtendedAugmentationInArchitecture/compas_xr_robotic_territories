from compas.geometry import Frame
from compas_robots import Configuration
from compas_fab.backends import RosClient
from compas_xr.mqtt import RealtimeMimicRequestMessage
from compas_xr.mqtt import RealtimeMimicResultMessage
from compas.data import json_load, json_dump

from ..control import fabrication as rtde
from ..control.joint_value_streamer import RTDEStateStreamer
from ..control.joint_value_streamer import ABBStateStreamer

import compas_rrc as rrc

class MimicROSHandler:

    def __init__(self, robot_name, robot_ip, tool_info_fp, additional_static_collision_meshes_fp=None, ros_ip='127.0.0.1', ros_port=9090):
        self.robot_name = robot_name
        self.robot_ip = robot_ip
        self.ros_client = RosClient(ros_ip, ros_port)
        self.ros_client.run(5)

        if not self.ros_client.is_connected:
            raise ConnectionError(f"RealtimeMimicROSHandler: [{robot_name}] Could not connect to ROS at {ros_ip}:{ros_port}")

        self.robot = self._load_robot()
        self._load_and_attach_tool(tool_info_fp)
        if additional_static_collision_meshes_fp:
            self.additional_static_collison_meshes = self._load_additional_static_collision_meshes(additional_static_collision_meshes_fp)
        else:
            self.additional_static_collison_meshes = None

        self.ik_solutions = []
        self._got_initial_config = False
        print(f"RealtimeMimicROSHandler : [{robot_name}] Handler initialized")

    ####################################################################################################
    # LOAD ROBOT
    ####################################################################################################

    def _load_robot(self):
        robot = self.ros_client.load_robot(load_geometry=False, precision=12)
        robot.client = self.ros_client
        return robot

    ####################################################################################################
    # Attaching TOOLS and COLLION MESHES
    ####################################################################################################

    def _load_and_attach_tool(self, tool_info_fp):
        if not tool_info_fp:
            raise ValueError("Tool information file path is required.")

        tool_info = json_load(tool_info_fp)
        print(f"MIMICROSHANDLER: [{self.robot_name}] Loading tool from {tool_info}")

    def _load_additional_static_collision_meshes(self, additional_static_collision_meshes_fp):
        if not additional_static_collision_meshes_fp:
            raise ValueError("Additional collision meshes file path is required.")
        additional_meshes = json_load(additional_static_collision_meshes_fp)
        print(f"MIMICROSHANDLER: [{self.robot_name}] Loading additional collision meshes from {additional_meshes}")
        #TODO: Return static collision meshes.

    ####################################################################################################
    # METHODS FOR CHILD CLASSES.
    ####################################################################################################    

    def _get_current_configuration(self):
        # This method should be implemented to get the current configuration of the robot
        # For example, using RTDE or another method to get the joint values
        raise NotImplementedError("This method should be implemented to get the current configuration of the robot.")

    def _send_to_config(self, config: Configuration):
        # print(f" RealtimeMimicROSHandler : [{self.robot_name}] (Sim) Executing: {config.joint_values}")
        raise NotImplementedError("This method should be implemented to get the current configuration of the robot.")

    ####################################################################################################
    # MESSAGE HANDLERS
    ####################################################################################################

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
            self._send_to_config(ik_config)
            fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\fabrication\python\test_config_vis_pb.json"
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicROSHandler : [{self.robot_name}] IK computation failed: {e}")
            return None


class URMimicHandlerROS(MimicROSHandler):
    
    def __init__(self, robot_name, robot_ip,  tool_info_fp, additional_static_collision_meshes_fp=None, ros_ip='127.0.0.1', ros_port=9090, speed=0.6, acceleration=0.1, radius=0.006, nowait=False):
        super().__init__(robot_name=robot_name, robot_ip=robot_ip,  tool_info_fp=tool_info_fp, additional_static_collision_meshes_fp=additional_static_collision_meshes_fp, ros_ip=ros_ip, ros_port=ros_port)

        #Feedback Streamer
        self.robot_state_streamer = RTDEStateStreamer(robot_ip=robot_ip, poll_delay=0.001)

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

    def _send_to_config(self, config: Configuration):
        print(f"URRealtimeMimicHandler: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

class ABBMimicHandlerROS(MimicROSHandler):
    
    def __init__(self, robot_name, robot_ip, abb_client, ros_ip='127.0.0.1', ros_port=9090, speed=100, nowait=False):
        super().__init__(robot_name, robot_ip, ros_ip, ros_port)
        self.speed = speed
        self.nowait = nowait

        self.ros_rrc = rrc.RosClient()
        self.ros_rrc.run()

        #Feedback Streamer
        self.robot_state_streamer = ABBStateStreamer(robot_ip=robot_ip, poll_delay=0.001)

        #TODO: CHECK NAME '/robLL_track' IS CORRECT
        self.abb = rrc.AbbClient(self.ros_rrc, abb_client)
        print(f"ABBRealtimeMimicHandler: [{robot_name}] Connected to ABB controller via RRC")

    def _get_current_configuration(self):
        robot_joints, _ = self.abb.send_and_wait(rrc.GetJoints())
        print(f"[{self.robot_name}] Current joints from controller: {robot_joints}")
        return self.robot.zero_configuration()

    def _send_to_config(self, config: Configuration):
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

