import os
import time
from compas.data import json_dump, json_load
from compas_fab.backends import PyBulletClient
from compas_robots import Configuration
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.backends.pybullet.exceptions import CollisionError

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from compas.geometry import Frame
import compas_fab
import compas_rrc as rrc

from compas_xr.mqtt import RealtimeMimicRequestMessage

from ..control import fabrication as rtde
from ..control.joint_value_streamer import RTDEStateStreamer
from ..control.joint_value_streamer import ABBStateStreamer
from ..control.move_j_gate import MoveJGate
from ..control.servo_gate import ServoJGate
import pybullet as pb

import time
from typing import List

from compas_fab.robots import Tool, CollisionMesh

class MimicPyBulletHandler:

    def __init__(self, robot_name, urdf_path, tool_info_fp, additional_static_collision_meshes_fp=None, srdf_path=None):
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

        self._load_and_attach_tool(tool_info_fp, self.robot)
        if additional_static_collision_meshes_fp:
            self.additional_static_collison_meshes = self._load_additional_static_collision_meshes(additional_static_collision_meshes_fp)
        else:
            self.additional_static_collison_meshes = None

        self.realtime_mimic_ik_solutions = []
        self._got_initial_config = False

        self._prev_cfg_cache = None      # last safe Configuration to restore sim to
        self._last_visual_step_t = 0.0   # throttle timestamp for sim stepping

        print(f"RealtimeMimicPyBulletHandler: [{robot_name}] Handler initialized")

    ####################################################################################################
    # LOAD ROBOT AND SEMANTICS
    ####################################################################################################

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

    ####################################################################################################
    # Attaching TOOLS and COLLION MESHES
    ####################################################################################################

    def _load_and_attach_tool(self, tool_info_fp, robot):
        if not tool_info_fp:
            raise ValueError("Tool information file path is required.")

        tool_info = json_load(tool_info_fp)
        visual_mesh = tool_info.get("visual_mesh", None)
        collision_mesh = tool_info.get("collision_mesh", None)
        tcf_frame = tool_info.get("tcf", None)
        if not visual_mesh or not collision_mesh or not tcf_frame:
            raise ValueError("Tool information must contain 'visual_mesh' and 'collision_mesh' and 'tcf'.")

        # collision_mesh = CollisionMesh(collision_mesh, "tool_cm")
        tool = Tool(visual=visual_mesh, collision=collision_mesh, frame_in_tool0_frame=tcf_frame, connected_to="tool0")
        robot.attach_tool(tool)
        print(f"MIMICPYBULLETHANDLER: [{self.robot_name}] Loading tool from {tool_info}")

    def _load_additional_static_collision_meshes(self, additional_attached_collision_meshes_fp):
        if not additional_attached_collision_meshes_fp:
            raise ValueError("Additional collision meshes file path is required.")
        additional_meshes = json_load(additional_attached_collision_meshes_fp)
        print(f"MIMICPYBULLETHANDLER: [{self.robot_name}] Loading additional collision meshes from {additional_meshes}")

    ####################################################################################################
    # Configuration & IK Solvers
    ####################################################################################################

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

    def try_fast_ik(self, frame, start_config=None, do_collision_check=True, extra_seed=False, visual_hz=30):
        """
        Fast IK (single seed + optional last_good). On success, optionally mirror to sim,
        throttled to `visual_hz` (default 30 Hz).
        """
        if start_config is None:
            start_config = self._get_latest_joint_values_from_stream_as_configuration()

        options = {"link_name": "tool0"}

        def solve_once(seed):
            if seed is None:
                return None
            try:
                return self.robot.inverse_kinematics(
                    frame_WCF=frame, start_configuration=seed, options=options
                )
            except StopIteration:
                return None
            except Exception:
                return None

        def collision_free(cfg):
            if cfg is None or not do_collision_check:
                return cfg is not None

            restore_cfg = (self._get_latest_joint_values_from_stream_as_configuration()
                        or getattr(self, "_prev_cfg_cache", None)
                        or self.robot.zero_configuration())
            try:
                self.client.set_robot_configuration(self.robot, cfg)
                try:
                    self.client.check_robot_self_collision(self.robot)  # raises on collision
                    collides = False
                except Exception as e:
                    collides = (e.__class__.__name__ == "CollisionError") or True
            finally:
                self.client.set_robot_configuration(self.robot, restore_cfg)
                self._prev_cfg_cache = restore_cfg
            return not collides

        def maybe_visualize(cfg):
            # throttle sim updates to avoid slowdown
            now = time.perf_counter()
            last = getattr(self, "_last_visual_step_t", 0.0)
            if now - last >= 1.0 / max(1, visual_hz):
                self.client.set_robot_configuration(self.robot, cfg)  # commit accepted IK to sim
                self.client.step_simulation()                         # single step
                self._last_visual_step_t = now

        # Attempt 1: current joints seed
        ik = solve_once(start_config)
        if ik and collision_free(ik):
            maybe_visualize(ik)
            return ik

        # Optional tiny fallback: last good IK seed
        if extra_seed and self.realtime_mimic_ik_solutions:
            ik = solve_once(self.realtime_mimic_ik_solutions[-1])
            if ik and collision_free(ik):
                maybe_visualize(ik)
                return ik

        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK failed.")
        return None

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

    ####################################################################################################
    # Configuration HELPERS
    ####################################################################################################

    def find_minimum_movement_config(self, start_config, candidate_configs):
        def joint_distance(c):
            return self.configuration_difference(start_config, c, return_sum=False)
        return min(candidate_configs, key=joint_distance)

    def configuration_difference(self, config1, config2, return_sum=False):
        diffs = [abs(a - b) for a, b in zip(config1.joint_values, config2.joint_values)]
        return sum(diffs) if return_sum else diffs
    
    ####################################################################################################
    # EMPTY METHODS FOR CHILD CLASSES.
    ####################################################################################################

    def _get_current_configuration(self):
        if not self.realtime_mimic_ik_solutions:
            return self.robot.zero_configuration()
        else:
            "using last configuration as start configuration"
        return self.realtime_mimic_ik_solutions[-1]
    
    def _send_to_target(self, frame: Frame):
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] (Sim) Executing motion to target frame: {frame}")

    def _send_to_configuration(self, config: Configuration):
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] (Sim) Executing: {config.joint_values}")

    def _get_latest_joint_values_from_stream(self):
        """
        This method should be implemented to get the latest joint values from the RTDE or other streaming source.
        For example, using RTDEStateStreamer or ABBStateStreamer.
        """
        raise NotImplementedError("This method should be implemented on the child classes.")

    def _get_latest_joint_values_from_stream_as_configuration(self):
        raise NotImplementedError("This method should be implemented on the child classes.")

    def _get_latest_tcp_from_stream(self):
        """
        This method should be implemented to get the latest joint values from the RTDE or other streaming source.
        For example, using RTDEStateStreamer or ABBStateStreamer.
        """
        raise NotImplementedError("This method should be implemented on the child classes.")

    # def  _send_to_configuration_through_servoj_gate(self, config: Configuration):
    #     raise NotImplementedError("This method should be implemented on the child classes.")

    def _send_to_configuration_through_gate(self, config: Configuration):
        raise NotImplementedError("This method should be implemented on the child classes.")

    def shutdown(self):
        self.client.__exit__(None, None, None)

    ####################################################################################################
    # MESSAGE HANDLERS RealtimeMimicResquestMessage
    ####################################################################################################

    def handle_realtime_msg_request_recursive_solver(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration()

        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration()

            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

        try:
            options = {"link_name": "tool0"}
            max_attempts = 8
            ik_config = self.find_valid_ik_recursive(frame, start_config, options, max_tries=max_attempts)
            if ik_config is None:
                self.client.set_robot_configuration(self.robot, start_config)
                self.client.step_simulation()
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] No valid IK solution found after {max_attempts} attempts.")
                return None

            self.realtime_mimic_ik_solutions.append(ik_config)
            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_compas_fab_itter(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration()
        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration()
            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

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

            self.realtime_mimic_ik_solutions.append(ik_config)
            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration()
        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration()
            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

        try:
            ik_config = self.robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options={"link_name": "tool0"}) #TODO: This tool0 param is hard coded for the UR20, it should be checked with the UR3 and ABB. Or passed as a paramater for the tool frame as well.
            self.client.set_robot_configuration(self.robot, ik_config)
            self.client.step_simulation()
            is_collision = self.client.check_robot_self_collision(self.robot)
            if is_collision:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK solution is in collision. Logging details:")
                self.log_self_collisions()
                if self.realtime_mimic_ik_solutions:
                    self.client.set_robot_configuration(self.robot, self.realtime_mimic_ik_solutions[-1])
                    self.client.step_simulation()
                else:
                    self.client.set_robot_configuration(self.robot, self.robot.zero_configuration())
                    self.client.step_simulation()
                return None
            self.realtime_mimic_ik_solutions.append(ik_config)

            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_ik_target(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        try:
            if msg.initial_request:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
                self.realtime_mimic_ik_solutions = []
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration()
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

                self.realtime_mimic_ik_solutions.append(ik_config)
                self._send_to_configuration(frame)
            else:
                print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Non-initial request received. Executing motion to target frame.")
                self._send_to_target(frame)
                return frame
        except Exception as e:
            print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_fastest_ik(self, msg: RealtimeMimicRequestMessage):
        # pull the requested frame
        frame = msg.requested_robot_frame

        # choose a seed
        if msg.initial_request:
            # reset history on first call (optional)
            self.realtime_mimic_ik_solutions = []

        if msg.initial_request or not self.realtime_mimic_ik_solutions:
            start_cfg = self._get_latest_joint_values_from_stream_as_configuration()
            if start_cfg is None:
                # rare fallback if stream isn't ready yet
                start_cfg = self.robot.zero_configuration()
        else:
            start_cfg = self.realtime_mimic_ik_solutions[-1]

        # fast IK: current-joints seed, optional single fallback (last good)
        ik = self.try_fast_ik(
            frame,
            start_config=start_cfg,
            do_collision_check=True,
            extra_seed=True   # set False if you truly want only one seed
        )
        if ik is None:
            return None  # try_fast_ik printed the single failure line

        # remember and command (no servoj)
        self.realtime_mimic_ik_solutions.append(ik)
        # self._send_to_configuration(ik)   # uses your RTDE move_to_joints path
        #TODO: Comment me in if you want to run.... # self._send_to_configuration_through_gate(ik)   # uses your RTDE move_to_joints path
        return ik

    def handle_user_defined_msg_request(self, msg: RealtimeMimicRequestMessage) -> List[JointTrajectory]:
        """
        This method can be overridden by child classes to handle custom message requests.
        """
        print(f"RealtimeMimicPyBulletHandler: [{self.robot_name}] Handling user-defined request: {msg.message} from {msg.header.device_id}")
        raise NotImplementedError("This method should be implemented in child classes.")

    # def handle_realtime_msg_request_servoj_gate(self, msg: RealtimeMimicRequestMessage) -> Configuration:
    #     raise NotImplementedError("This method should be implemented in child classes.")

    ####################################################################################################
    # MESSAGE HANDLERS MimicResquestMessage
    ####################################################################################################




class URMimicHandlerPyB(MimicPyBulletHandler):

    def __init__(self, robot_name, robot_ip, urdf_path, tool_info_fp, additional_static_collision_meshes_fp=None, srdf_path=None, speed=0.6, acceleration=0.1, radius=0.006, nowait=False):
        super().__init__(robot_name, urdf_path, tool_info_fp, additional_static_collision_meshes_fp, srdf_path=srdf_path)

        self.robot_state_streamer = RTDEStateStreamer(robot_ip=robot_ip, poll_delay=0.001)
        self.robot_state_streamer.start()

        self.robot_ip = robot_ip
        self.speed = speed
        self.acceleration = acceleration
        self.radius = radius
        self.nowait = nowait 

        # persist RTDE connections once
        self.rtde_ctrl = RTDEControl(self.robot_ip)
        self.rtde_recv = RTDEReceive(self.robot_ip)

        # # create one gate instance using your JSON speed/accel
        # self.movej_gate = MoveJGate(
        #     self.rtde_ctrl,
        #     speed=self.speed,
        #     accel=self.acceleration,
        #     nowait=True,          # async so your loop doesn’t block
        #     min_dt=0.18,          # ~5–6 Hz max send rate
        #     min_dq=0.015          # ~0.86° deadband
        # )
    
        # self.movej_gate = MoveJGate(
        #     self.rtde_ctrl,
        #     speed=self.speed,
        #     accel=self.acceleration,
        #     nowait=True,
        #     min_dt=0.18,     # still used when blending can't flush yet
        #     min_dq=0.015,
        #     blend_radius=0.01,   # 10 mm is a good start
        #     blend_batch=4,       # 3–5 points works well
        #     blend_every=0.35     # flush ~3×/s
        # )

        #TODO: THIS WAS THE BEST......
        # self.movej_gate = MoveJGate(self.rtde_ctrl,
        #     speed=self.speed, accel=0.9,
        #     min_dt=999,            # disable single sends
        #     min_dq=1e9,            # disable single sends
        #     blend_radius=0.012,    # 12 mm blend
        #     blend_batch=4,         # 3–5 points
        #     blend_every=0.40       # ~2.5 Hz flush
        # )

        self.servo_gate = ServoJGate(
            rtde_ctrl=self.rtde_ctrl,
            rtde_recv=self.rtde_recv,
            speed_cap=max(0.4, min(0.9, self.speed if self.speed else 0.8)),  # rad/s
            accel_cap=max(0.8, min(1.8, self.acceleration if self.acceleration else 1.5)),  # rad/s^2
            dt_nominal=1/125.0,
            lookahead=0.10,
            gain=280,
            target_alpha=0.25,   # smoothing on targets
            k_speed=1.6,         # adaptive speed scaling
            tau=0.30,            # accel ≈ speed / tau
            cmd_alpha=0.35,      # smoothing on caps
            min_dt_send=0.010,   # don’t spam faster than 100 Hz
            min_dq=0.0,
            verbose=False
        )

        print(f"URRealtimeMimicHandlerPyB: [{robot_name}] UR handler initialized")

    ####################################################################################################
    # Killing the streamer and closing the connection
    ####################################################################################################

    def __enter__(self): return self
    def __exit__(self, exc_type, exc, tb): self.close()
    def __del__(self):
        try: self.close()
        except: pass

    def close(self):
        # stop streamer
        try: self.robot_state_streamer.stop()
        except: pass
        # stop servoj cleanly
        try: self.servo_gate.stop()
        except: pass
        # stop any running script on controller (safe to call)
        try: self.rtde_ctrl.stopScript()
        except: pass
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] closed")

    ####################################################################################################
    # Implemented through standard RTDE functions in fabrication.py
    ####################################################################################################

    def _get_current_configuration(self):
        # config = rtde.get_config_TEST(self.robot_ip)
        config = rtde.get_config(self.robot_ip)
        return config
        # config_zero = self.robot.zero_configuration()
        # return config_zero

    def _send_to_configuration(self, config: Configuration):
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        # rtde.move_to_joints_blend(config, self.speed, self.acceleration, blend=self.radius, nowait=self.nowait, ip=self.robot_ip)
        
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    def _send_to_target(self, frame: Frame):
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion to target frame: {frame}")
        # rtde.move_to_target(frame, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_target(frame, self.speed, self.acceleration, nowait=True, ip=self.robot_ip)
        # rtde.move_to_target_TEST(frame, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    ####################################################################################################
    # Implemented through Streamer Class Interface
    ####################################################################################################

    #TODO: TESTING BIG TIME
    def handle_realtime_msg_request_servoj_gate(self, msg: RealtimeMimicRequestMessage):
        """Compute IK fast and stream joints via servoj (no threads)."""
        frame = msg.requested_robot_frame

        # reset history on first call
        if msg.initial_request:
            self.realtime_mimic_ik_solutions = []

        # choose seed: current joints if no history, else last good
        if msg.initial_request or not self.realtime_mimic_ik_solutions:
            start_cfg = self._get_latest_joint_values_from_stream_as_configuration()
            if start_cfg is None:
                start_cfg = self.robot.zero_configuration()
        else:
            start_cfg = self.realtime_mimic_ik_solutions[-1]

        # fast IK (single seed + optional fallback); 30 Hz sim mirror inside
        ik = self.try_fast_ik(frame,
                            start_config=start_cfg,
                            do_collision_check=True,
                            extra_seed=True,
                            visual_hz=30)

        if ik:
            # remember and command through servoj
            self.realtime_mimic_ik_solutions.append(ik)
            self.servo_gate.set_target(ik.joint_values)
            self.servo_gate.tick()
            return ik

        # IK failed: keep feeding servo with last known good (or current measured)
        if self.realtime_mimic_ik_solutions:
            self.servo_gate.set_target(self.realtime_mimic_ik_solutions[-1].joint_values)
        else:
            q = self._get_latest_joint_values_from_stream()
            if q is not None:
                self.servo_gate.set_target(q)
        self.servo_gate.tick()
        return None

    def _send_to_configuration_through_gate(self, config: Configuration):
        sent = self.movej_gate.maybe_send(config.joint_values)
        if sent:
            print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] moveJ sent (speed={self.speed}, accel={self.acceleration})")

    def _send_to_configuration_through_servoj_gate(self, config: Configuration):
        # Stream smoothed joints via servoj, no threads, non-blocking
        self.servo_gate.set_target(config.joint_values)
        self.servo_gate.tick()

    def _get_latest_joint_values_from_stream(self):
        state = self.robot_state_streamer.get_latest()
        if state:
            q, tcp = state
            return q
        else:
            print(f"[{self.robot_name}] No JOINTVALUE state data available yet.")
            return None

    def _get_latest_joint_values_from_stream_as_configuration(self):
        q = self._get_latest_joint_values_from_stream()
        if q:
            joint_names = self.robot.get_configurable_joint_names()
            joint_types = self.robot.get_configurable_joint_types()
            if len(q) != len(joint_names):
                raise ValueError(f"Length of joint values ({len(q)}) does not match number of configurable joints ({len(joint_names)}).")
            return Configuration(joint_names=joint_names, joint_types=joint_types, joint_values=q)
        else:
            print(f"[{self.robot_name}] No JOINTVALUE state data available yet.")
            return None

    def _get_latest_tcp_from_stream(self):
        state = self.robot_state_streamer.get_latest()
        if state:
            q, tcp = state
            return tcp
        else:
            print(f"[{self.robot_name}] No TCP state data available yet.")
            return None




#TODO: FIX later (need to compute IK in PyBullet and send to ABB using ROSClient in RRC)
class ABBMimicHandlerPyB(MimicPyBulletHandler):
    
    def __init__(self, robot_name, robot_ip, abb_client_name, ros_ip='127.0.0.1', ros_port=9090, speed=100, nowait=False):
        super().__init__(robot_name, robot_ip, ros_ip, ros_port)

        self.robot_state_streamer = ABBStateStreamer(robot_ip=robot_ip, poll_delay=0.001)
        self.robot_state_streamer.start()

        self.speed = speed
        self.nowait = nowait

        self.ros_rrc = rrc.RosClient()
        self.ros_rrc.run()

        #TODO: CHECK NAME '/robLL_track' IS CORRECT
        self.abb = rrc.AbbClient(self.ros_rrc, abb_client_name)
        print(f"ABBRealtimeMimicHandler: [{robot_name}] Connected to ABB controller via RRC")

    ####################################################################################################
    # Killing the streamer and closing the connection
    ####################################################################################################

    def __enter__(self): return self
    def __exit__(self, exc_type, exc, tb): self.close()
    def __del__(self):
        try: self.close()
        except: pass

    def close(self):
        try:
            self.robot_state_streamer.stop()
            self.ros_rrc.close()
            self.ros_rrc.terminate()
        finally:
            print(f"ABBRealtimeMimicHandler: [{self.robot_name}] closed")

    ####################################################################################################
    # Execution
    ####################################################################################################

    def _get_current_configuration(self):
        robot_joints, _ = self.abb.send_and_wait(rrc.GetJoints())
        print(f"[{self.robot_name}] Current joints from controller: {robot_joints}")
        return self.robot.zero_configuration()

    def _send_to_configuration(self, config: Configuration):
        print(f"ABBRealtimeMimicHandler: [{self.robot_name}] Executing motion: {config.joint_values}")
        
        # Convert radians to degrees for ABB
        joint_values_deg = [v * 180.0 / 3.1415926 for v in config.joint_values]
        rax = rrc.RobotJoints(*joint_values_deg)
        ext_axes = [0.0] * 6  # TODO: Placeholder for external axes

        result = self.abb.send_and_wait(rrc.MoveToJoints(rax, ext_axes, self.speed, rrc.Zone.FINE))
        print(f"[{self.robot_name}] Motion complete: {result}")

    def _send_to_target(self, frame: Frame):
        raise NotImplementedError("ABBRealtimeMimicHandlerPyB : WIP - Target frame motion not implemented yet.")
        print(f"URRealtimeMimicHandlerPyB: [{self.robot_name}] (Sim) Executing UR motion to target frame: {frame}")

