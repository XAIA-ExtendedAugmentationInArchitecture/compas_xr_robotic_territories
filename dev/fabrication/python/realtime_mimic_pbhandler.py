import os
import time
from compas.data import json_dump
from compas_fab.backends import PyBulletClient
from compas_robots import Configuration
import compas_fab
import compas_rrc as rrc
from compas_xr.mqtt import RealtimeMimicRequestMessage
from control import fabrication as rtde
import pybullet as pb

class RealtimeMimicPyBulletHandler:

    def __init__(self, robot_name, urdf_path, srdf_path=None):
        self.robot_name = robot_name
        self.urdf_path = os.path.normpath(urdf_path)
        if srdf_path:
            self.srdf_path = os.path.normpath(srdf_path)
        else:
            self.srdf_path = None
        self.client = PyBulletClient()
        self.client.__enter__()  # For manual control over context
        self.robot = self._load_robot()
        self.semantics = self._load_semantics()
        self.ik_solutions = []
        self._got_initial_config = False
        print(f"RealtimeMimicPyBulletHandler: [{robot_name}] Handler initialized")

    def _load_robot(self):
        urdf_file = compas_fab.get(self.urdf_path)
        robot = self.client.load_robot(urdf_file)
        return robot

    def _load_semantics(self):
        if self.srdf_path:
            print("Semantics Loaded")
            semantics = self.client.load_semantics(self.robot, srdf_filename=self.srdf_path)
        else:
            print("Sementics Not loaded")
            semantics = None
        return semantics
        # self.disable_self_collision_between_links("base_link_inertia", "shoulder_link")
        

    def disable_self_collision_between_links(self, link_name_a, link_name_b):
        """Disable collision between two robot links using the PyBullet API directly."""
        robot_uid = self.client.client_id

        def get_link_index(name):
            if name == "base_link" or name.endswith("_inertia"):
                return -1  # base
            for i in range(pb.getNumJoints(robot_uid)):
                joint_info = pb.getJointInfo(robot_uid, i)
                if joint_info[12].decode("utf-8") == name:
                    return i
            return None

        idx_a = get_link_index(link_name_a)
        idx_b = get_link_index(link_name_b)

        if idx_a is None or idx_b is None:
            print(f"Could not find link indices for '{link_name_a}' or '{link_name_b}'")
            return

        pb.setCollisionFilterPair(robot_uid, robot_uid, idx_a, idx_b, enableCollision=0)
        print(f"Ignoring collision between '{link_name_a}' and '{link_name_b}'")

    def check_self_collision_ignoring(self, ignored_pairs, threshold=0.001):
        robot_uid = self.client.client_id

        for i in range(pb.getNumJoints(robot_uid)):
            for j in range(i + 1, pb.getNumJoints(robot_uid)):
                name_i = pb.getJointInfo(robot_uid, i)[12].decode("utf-8")
                name_j = pb.getJointInfo(robot_uid, j)[12].decode("utf-8")

                # Skip if the pair is in the ignore list
                if (name_i, name_j) in ignored_pairs or (name_j, name_i) in ignored_pairs:
                    continue

                contacts = pb.getClosestPoints(
                    bodyA=robot_uid, bodyB=robot_uid,
                    distance=threshold, linkIndexA=i, linkIndexB=j
                )

                for c in contacts:
                    if c[8] < threshold:  # 8 = contact distance
                        #A collision that matters!
                        return True
        return False

    def _get_current_configuration(self):
        if not self.ik_solutions:
            return self.robot.zero_configuration()
        else:
            "using last configuration as start configuration"
        return self.ik_solutions[-1]

    def log_self_collisions(self, ignored_pairs=None, threshold=0.001):
        """Log all robot self-collisions within a given distance threshold,
        ignoring specific link pairs if provided.

        Parameters
        ----------
        ignored_pairs : list of tuple of str, optional
            Pairs of link names to ignore.
        threshold : float
            Distance threshold for considering proximity as collision.
        """
        ignored_pairs = ignored_pairs or []
        robot_uid = self.client.client_id
        printed = set()

        for i in range(pb.getNumJoints(robot_uid)):
            for j in range(i + 1, pb.getNumJoints(robot_uid)):
                name_i = pb.getJointInfo(robot_uid, i)[12].decode('utf-8')
                name_j = pb.getJointInfo(robot_uid, j)[12].decode('utf-8')

                if (name_i, name_j) in ignored_pairs or (name_j, name_i) in ignored_pairs:
                    continue

                contacts = pb.getClosestPoints(robot_uid, robot_uid, distance=threshold, linkIndexA=i, linkIndexB=j)
                if contacts:
                    pair = tuple(sorted((name_i, name_j)))
                    if pair not in printed:
                        min_dist = min(contact[8] for contact in contacts)  # contact distance
                        print(f"Detected proximity/contact between: {pair[0]} ↔ {pair[1]} (dist={min_dist:.4f} m)")
                        printed.add(pair)

        if not printed:
            print("No self-collisions detected.")

    def handle_msg_request(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.ik_solutions = []

        start_config = self._get_current_configuration()

        try:
            ik_config = self.robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options={"link_name": "tool0"}) #TODO: This tool0 param is hard coded for the UR20, it should be checked with the UR3 and ABB. Or passed as a paramater for the tool frame as well.
            self.client.set_robot_configuration(self.robot, ik_config)
            self.client.step_simulation()
            is_collision = self.client.check_robot_self_collision(self.robot)
            if is_collision:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK solution is in collision. Logging details:")
                self.log_self_collisions()
                if self.ik_solutions:
                    self.client.set_robot_configuration(self.robot, self.ik_solutions[-1])
                    self.client.step_simulation()
                else:
                    self.client.set_robot_configuration(self.robot, self.robot.zero_configuration())
                    self.client.step_simulation()

                for _ in range(5):
                    self.client.step_simulation()
                    time.sleep(0.05)
                print("NOT SETTING CONFIG: IN COLLISION")
                return None
            self.ik_solutions.append(ik_config)

            self._execute_motion(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_msg_request_old(self, msg: RealtimeMimicRequestMessage) -> Configuration:
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.ik_solutions = []

        start_config = self._get_current_configuration()

        try:
            ik_config = self.robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options={"link_name": "tool0"}) #TODO: This tool0 param is hard coded for the UR20, it should be checked with the UR3 and ABB. Or passed as a paramater for the tool frame as well.
            self.client.set_robot_configuration(self.robot, ik_config)
            self.client.step_simulation()
            ignored = [
                    ("base_link_inertia", "shoulder_link"),
                    ("shoulder_link", "upper_arm_link"),
                    ("forearm_link", "upper_arm_link"),
                    ("forearm_link", "wrist_1_link"),
                    ("forearm_link", "wrist_2_link"),
                    ("forearm_link", "wrist_3_link"),
                    ("wrist_1_link", "wrist_2_link"),
                    ("wrist_2_link", "wrist_3_link"),
                ]
            is_collision = self.check_self_collision_ignoring(ignored)
            if is_collision:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK solution is in collision. Logging details:")
                self.log_self_collisions(ignored_pairs=ignored, threshold=0.001)
                # print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK solution is in collision. Skipping execution.")
                if self.ik_solutions:
                    self.client.set_robot_configuration(self.robot, self.ik_solutions[-1])
                else:
                    self.client.set_robot_configuration(self.robot, self.robot.zero_configuration())
                return None
            self.ik_solutions.append(ik_config)
            
            # time.sleep(0.5)  # For visualization step pacing
            self._execute_motion(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def _execute_motion(self, config: Configuration):
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] (Sim) Executing: {config.joint_values}")

    def shutdown(self):
        self.client.__exit__(None, None, None)



class URRealtimeMimicHandlerPyB(RealtimeMimicPyBulletHandler):
    
    def __init__(self, robot_name, robot_ip, urdf_path, srdf_path=None, speed=0.6, acceleration=0.1, radius=0.006, nowait=False):
        super().__init__(robot_name, urdf_path, srdf_path=srdf_path)
        self.robot_ip = robot_ip
        self.speed = speed
        self.acceleration = acceleration
        self.radius = radius
        self.nowait = nowait
        print(f"URRealtimeMimicHandlerPyB: [{robot_name}] UR handler initialized")

    def _get_current_configuration(self):
        config = rtde.get_config_TEST(self.robot_ip)
        # config = rtde.get_config(self.robot_ip)
        # return config
        config_zero = self.robot.zero_configuration()
        return config_zero

    def _execute_motion(self, config: Configuration):
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

#TODO: FIX later (need to compute IK in PyBullet and send to ABB using ROSClient in RRC)
class ABBRealtimeMimicHandler(RealtimeMimicPyBulletHandler):
    
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

