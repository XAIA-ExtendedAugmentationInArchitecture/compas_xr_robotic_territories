import os
import time
from compas.data import json_dump
from compas_fab.backends import PyBulletClient
from compas_robots import Configuration
from compas.geometry import Frame
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

    def _get_current_configuration(self):
        if not self.ik_solutions:
            return self.robot.zero_configuration()
        else:
            "using last configuration as start configuration"
        return self.ik_solutions[-1]
    
    def _execute_motion_target(self, frame: Frame):
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] (Sim) Executing motion to target frame: {frame}")

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

    def handle_msg_request_recursive_solver(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.ik_solutions = []
            start_config = self._get_current_configuration()
        else:
            if len(self.ik_solutions) == 0:
                start_config = self._get_current_configuration()
            else:
                start_config = self.ik_solutions[-1]

        try:
            options = {"link_name": "tool0"}
            max_attempts = 8
            ik_config = self.find_valid_ik_recursive(frame, start_config, options, max_tries=max_attempts)
            if ik_config is None:
                self.client.set_robot_configuration(self.robot, start_config)
                self.client.step_simulation()
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] No valid IK solution found after {max_attempts} attempts.")
                return None

            self.ik_solutions.append(ik_config)
            self._execute_motion(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_msg_request_compas_fab_itter(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.ik_solutions = []
            start_config = self._get_current_configuration()
        else:
            if len(self.ik_solutions) == 0:
                start_config = self._get_current_configuration()
            else:
                start_config = self.ik_solutions[-1]

        try:
            options = dict(
                link_name="tool0",
                high_accuracy_threshold=1e-6,
                high_accuracy_max_iter=8
            )
            ik_config = self.find_best_valid_ik_compas_fab_itter_ik(frame, start_config, options)

            if ik_config is None:
                self.client.set_robot_configuration(self.robot, start_config)
                self.client.step_simulation()
                return None

            self.ik_solutions.append(ik_config)
            self._execute_motion(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_msg_request(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.ik_solutions = []
            start_config = self._get_current_configuration()
        else:
            if len(self.ik_solutions) == 0:
                start_config = self._get_current_configuration()
            else:
                start_config = self.ik_solutions[-1]

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
                return None
            self.ik_solutions.append(ik_config)

            self._execute_motion(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_msg_request_ik_target(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        try:
            if msg.initial_request:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
                self.ik_solutions = []
                start_config = self._get_current_configuration()
                options = dict(
                    link_name="tool0",
                    high_accuracy_threshold=1e-6,
                    high_accuracy_max_iter=8
                )
                ik_config = self.find_best_valid_ik_compas_fab_itter_ik(frame, start_config, options)

                if ik_config is None:
                    self.client.set_robot_configuration(self.robot, start_config)
                    self.client.step_simulation()
                    return None

                self.ik_solutions.append(ik_config)
                self._execute_motion(frame)
            else:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Non-initial request received. Executing motion to target frame.")
                self._execute_motion_target(frame)
                return frame
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None


    def find_minimum_movement_config(self, start_config, candidate_configs):
        def joint_distance(c):
            return self.configuration_difference(start_config, c, return_sum=False)
        return min(candidate_configs, key=joint_distance)

    def configuration_difference(self, config1, config2, return_sum=False):
        diffs = [abs(a - b) for a, b in zip(config1.joint_values, config2.joint_values)]
        return sum(diffs) if return_sum else diffs

    def _execute_motion(self, config: Configuration):
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] (Sim) Executing: {config.joint_values}")

    def shutdown(self):
        self.client.__exit__(None, None, None)

    def find_valid_ik_recursive(self, frame, start_config, options=None, max_tries=10, attempt=0):
        if attempt >= max_tries:
            print(f"[{self.robot_name}] Max IK attempts ({max_tries}) reached.")
            return None

        try:
            ik_config = self.robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options=options)
            self.client.set_robot_configuration(self.robot, ik_config)
            self.client.step_simulation()

            if self.client.check_robot_self_collision(self.robot):
                print(f"[{self.robot_name}] Attempt {attempt + 1}: IK result in collision. Trying again...")
                return self.find_valid_ik_recursive(frame, start_config, options, max_tries, attempt + 1)
            else:
                print(f"[{self.robot_name}] Found collision-free IK solution on attempt {attempt + 1}.")
                return ik_config
        except Exception as e:
            print(f"[{self.robot_name}] IK exception at attempt {attempt + 1}: {e}")
            return self.find_valid_ik_recursive(frame, start_config, options, max_tries, attempt + 1)

    def find_best_valid_ik_compas_fab_itter_ik(self, frame, start_config, options, max_results=20):
        """
        Iteratively searches for valid IK solutions and selects the closest one
        based on joint difference.

        Parameters
        ----------
        frame : compas.geometry.Frame
            The target frame for the IK.
        start_config : Configuration
            The starting configuration to compare against.
        options : dict
            Additional options to pass to the IK solver.
        max_results : int, optional
            Max number of solutions to evaluate.

        Returns
        -------
        Configuration or None
            The best collision-free IK solution found, or None if none valid.
        """
        options["max_results"] = max_results
        valid_configs = []

        for candidate in self.robot.iter_inverse_kinematics(
            frame_WCF=frame,
            start_configuration=start_config,
            options=options
        ):
            self.client.set_robot_configuration(self.robot, candidate)
            self.client.step_simulation()

            if not self.client.check_robot_self_collision(self.robot):
                valid_configs.append(candidate)

        if not valid_configs:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] No valid collision-free IK solutions found.")
            return None

        return self.find_minimum_movement_config(start_config, valid_configs)


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
        # config = rtde.get_config_TEST(self.robot_ip)
        config = rtde.get_config(self.robot_ip)
        return config
        # config_zero = self.robot.zero_configuration()
        # return config_zero

    def _execute_motion(self, config: Configuration):
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_joints_blend(config, self.speed, self.acceleration, blend=self.radius, nowait=self.nowait, ip=self.robot_ip)
        
        # rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    def _execute_motion_target(self, frame: Frame):
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion to target frame: {frame}")
        # rtde.move_to_target(frame, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_target(frame, self.speed, self.acceleration, nowait=True, ip=self.robot_ip)

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

