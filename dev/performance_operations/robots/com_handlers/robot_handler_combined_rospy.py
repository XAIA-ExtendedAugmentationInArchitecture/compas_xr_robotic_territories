import os
import time
from compas.data import json_dump, json_load
from compas_fab.backends import PyBulletClient
from compas_robots import Configuration
from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
from compas_fab.backends.pybullet.exceptions import CollisionError

from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

from compas.geometry import Frame, Quaternion, Vector, Translation, Rotation
import compas_fab
import compas_rrc as rrc

from compas_xr.mqtt import RealtimeMimicRequestMessage, MimicTrajectoryRequestMessage, ExecuteMimicTrajectoryRequestMessage, RealtimeMimicIOToggleRequestMessage

from ..control import fabrication as rtde
from ..control.joint_value_streamer import RTDEStateStreamer
from ..control.joint_value_streamer import ABBStateStreamer
from ..control.move_j_gate import MoveJGate
from ..control.servo_gate import ServoJGate
import pybullet as pb

import time
from typing import List, Optional
import math
import numpy as np

from compas_fab.robots import Tool, CollisionMesh
from compas_fab.backends.pybullet.planner import PyBulletPlanner

from pybullet_planning import plan_joint_motion, set_joint_positions

from compas_fab.backends import RosClient

class RobotHandlerCombinedBackends:

    def __init__(self, robot_name, urdf_path, tool_info_fp, ros_ip='127.0.0.1', ros_port=9090, additional_static_collision_meshes_fp=None, group="manipulator", srdf_path=None):
        self.robot_name = robot_name
        
        #Things for both backends
        self.tool = self._load_and_create_tool_for_backends(tool_info_fp)
        self.group = group

        #Pybullet Inputs
        self.urdf_path = os.path.normpath(urdf_path)
        if srdf_path:
            self.srdf_path = os.path.normpath(srdf_path)
        else:
            self.srdf_path = None

        self.pyb_client = PyBulletClient()
        self.pyb_client.__enter__()
        self.pyb_robot = self._pyb_load_robot()
        self.pyb_semantics = self._pyb_load_semantics()

        self._attach_tool_to_robot(tool=self.tool, robot=self.pyb_robot, backendname="PyBullet")
        if additional_static_collision_meshes_fp:
            self.additional_static_collison_meshes = self._load_additional_static_collision_meshes(additional_static_collision_meshes_fp)
        else:
            self.additional_static_collison_meshes = None


        #ROS Inputs
        #TODO: See if You need a PlanningScene for ROS
        self.ros_client = RosClient(ros_ip, ros_port)
        self.ros_client.run(5)
        if not self.ros_client.is_connected:
            raise ConnectionError(f"CombinedBackendHandler: [{robot_name}] Could not connect to ROS at {ros_ip}:{ros_port}")
        else:
            print(f"CombinedBackendHandler: [{robot_name}] Connected to ROS at {ros_ip}:{ros_port}")
        self.ros_robot = self._ros_load_robot()
        self._attach_tool_to_robot(tool=self.tool, robot=self.ros_robot, backendname="ROS")

        #Post Processing Things for both Robots
        if self.additional_static_collison_meshes:
            self._pyb_add_additional_static_collision_meshes_to_scene(self.additional_static_collison_meshes)
            self._ros_add_additional_static_collision_meshes_to_scene(self.additional_static_collison_meshes)
        else:
            print(f"CombinedBackendHandler: [{robot_name}] No additional static collision meshes to add to either backend")

        # Message handling attributes
        self.realtime_mimic_ik_solutions = []
        self._got_initial_config = False
        self._prev_cfg_cache = None      # last safe Configuration to restore sim to
        self._last_visual_step_t = 0.0   # throttle timestamp for sim stepping

        print(f"CombinedBackendHandler: [{robot_name}] Handler initialized")

    ####################################################################################################
    # LOAD ROBOT AND SEMANTICS
    ####################################################################################################

    def _pyb_load_robot(self):
        urdf_file = compas_fab.get(self.urdf_path)
        robot = self.pyb_client.load_robot(urdf_file)
        return robot

    def _pyb_load_semantics(self):
        if self.srdf_path:
            print("Semantics Loaded")
            semantics = self.pyb_client.load_semantics(self.pyb_robot, srdf_filename=self.srdf_path)
        else:
            print("Sementics Not loaded")
            semantics = None
        return semantics

    def _ros_load_robot(self):
        robot = self.ros_client.load_robot(load_geometry=False, precision=12)
        robot.client = self.ros_client
        return robot

    ####################################################################################################
    # Attaching TOOLS and COLLION MESHES
    ####################################################################################################

    def _load_and_create_tool_for_backends(self, tool_info_fp):
        if not tool_info_fp:
            raise ValueError("Tool information file path is required.")

        tool_info = json_load(tool_info_fp)
        visual_mesh = tool_info.get("visual_mesh", None)
        collision_mesh = tool_info.get("collision_mesh", None)
        tcf_frame = tool_info.get("tcf", None)
        if not visual_mesh or not collision_mesh or not tcf_frame:
            raise ValueError("Tool information must contain 'visual_mesh' and 'collision_mesh' and 'tcf'.")

        tool = Tool(visual=visual_mesh, collision=collision_mesh, frame_in_tool0_frame=tcf_frame, connected_to="tool0")
        print(f"CombinedBackendHandler: [{self.robot_name}] Loading tool from {tool_info} for both backends")
        return tool

    def _load_additional_static_collision_meshes(self, additional_attached_collision_meshes_fp):
        if not additional_attached_collision_meshes_fp:
            raise ValueError("Additional collision meshes file path is required.")
        additional_meshes = json_load(additional_attached_collision_meshes_fp)
        print(f"CombinedBackendHandler: [{self.robot_name}] Loading additional collision meshes from {additional_meshes}")

    def _attach_tool_to_robot(self, tool, robot, backendname="PyBullet"):
        robot.attach_tool(tool, self.group)
        print(f"CombinedBackend: [{self.robot_name}] Attched Tool in Backend : {backendname}")

    def _pyb_add_additional_static_collision_meshes_to_scene(self, additional_collision_meshes):
        if not additional_collision_meshes:
            return

        print(f"CombinedBackend: [{self.robot_name}] Adding additional static collision meshes to PyBullet scene")

        # for mesh_info in additional_collision_meshes:
        #     mesh = CollisionMesh(mesh_info["mesh"], frame=mesh_info["frame"])
        #     self.pyb_client.add_collision_mesh(mesh)
        #     print(f"CombinedBackend: [{self.robot_name}] Added additional static collision mesh to PyBullet scene: {mesh_info['mesh']}")

    def _ros_add_additional_static_collision_meshes_to_scene(self, additional_collision_meshes):
        if not additional_collision_meshes:
            return

        print(f"CombinedBackend: [{self.robot_name}] Adding additional static collision meshes to ROS scene")

        #TODO: I am not sure if I need to add as Scene for the collision objects this should be checked.
        # for mesh_info in additional_collision_meshes:
        #     mesh = CollisionMesh(mesh_info["mesh"], frame=mesh_info["frame"])
        #     self.ros_client.add_collision_mesh(mesh)
        #     print(f"CombinedBackend: [{self.robot_name}] Added additional static collision mesh to ROS scene: {mesh_info['mesh']}")

    ####################################################################################################
    # Configuration & IK Solvers
    ####################################################################################################

    ##### PyBullet IK Methods #####

    def pyb_find_valid_ik_recursive(self, frame, start_config, options=None, max_tries=10, attempt=0):
        if attempt >= max_tries:
            print(f"[{self.robot_name}] Max IK attempts ({max_tries}) reached.")
            return None

        try:
            ik_config = self.pyb_robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options=options)
            self.pyb_client.set_robot_configuration(self.pyb_robot, ik_config)
            self.pyb_client.step_simulation()

            if self.pyb_client.check_robot_self_collision(self.pyb_robot):
                print(f"[{self.robot_name}] Attempt {attempt + 1}: IK result in collision. Trying again...")
                return self.pyb_find_valid_ik_recursive(frame, start_config, options, max_tries, attempt + 1)
            else:
                print(f"[{self.robot_name}] Found collision-free IK solution on attempt {attempt + 1}.")
                return ik_config
        except Exception as e:
            print(f"[{self.robot_name}] IK exception at attempt {attempt + 1}: {e}")
            return self.pyb_find_valid_ik_recursive(frame, start_config, options, max_tries, attempt + 1)

    def pyb_find_best_valid_ik_compas_fab_itter_ik(self, frame, start_config, options, max_results=20):
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

        #TODO: Maybe Remove this...
        self.pyb_client.set_robot_configuration(self.pyb_robot, start_config)
        self.pyb_client.step_simulation()


        for candidate in self.pyb_robot.iter_inverse_kinematics(
            frame_WCF=frame,
            start_configuration=start_config,
            options=options
        ):
            self.pyb_client.set_robot_configuration(self.pyb_robot, candidate)
            self.pyb_client.step_simulation()

            if not self.pyb_client.check_robot_self_collision(self.pyb_robot):
                valid_configs.append(candidate)

        if not valid_configs:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid collision-free IK solutions found.")
            return None

        return self.find_minimum_movement_config(start_config, valid_configs)

    def pyb_find_best_valid_ik_compas_fab_itter_ik_improved(self, frame, start_config, options=None, max_results=20):
        options = {} if options is None else dict(options)
        link = options.get("link_name", "tool0")

        # Phase A: coarse to get a feasible point (deterministic)
        coarse = dict(options)
        coarse.setdefault("high_accuracy_threshold", 1e-3)
        coarse.setdefault("high_accuracy_max_iter", 64)

        feasible = None
        tried = 0
        for cfg in self.pyb_robot.iter_inverse_kinematics(
            frame_WCF=frame,
            start_configuration=start_config,
            options=coarse
        ):
            tried += 1
            if self._pyb_is_valid(cfg):
                feasible = cfg
                break
            if tried >= max_results:
                break

        if feasible is None:
            print(f"[{self.robot_name}] No valid collision-free IK (coarse).")
            return None

        # Phase B: refine accurately from feasible cfg (deterministic, strict)
        strict = dict(options)
        strict.setdefault("high_accuracy_threshold", 1e-6)
        strict.setdefault("high_accuracy_max_iter", 128)

        refined = self.pyb_robot.inverse_kinematics(
            frame_WCF=frame,
            start_configuration=feasible,
            options=strict
        )

        if refined is None or not self._pyb_is_valid(refined):
            # If refinement drove it into a collision or failed numerically, keep feasible.
            return feasible if self._pyb_is_valid(feasible) else None

        # # Optional: verify pose error explicitly
        # ee = getattr(self.robot, "get_end_effector_link_name", lambda: link)()
        # tcp_world = self.robot.forward_kinematics(refined, link_name=ee) * getattr(self.robot, "tool", None).frame if getattr(self.robot, "tool", None) else self.robot.forward_kinematics(refined, link_name=ee)
        # pos_err = (tcp_world.point - frame.point).length
        return refined

    def pyb_try_fast_ik(self, frame, start_config=None, do_collision_check=True, extra_seed=False, visual_hz=30):
        """
        Fast IK (single seed + optional last_good). On success, optionally mirror to sim,
        throttled to `visual_hz` (default 30 Hz).
        """
        if start_config is None:
            start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")

        options = {"link_name": "tool0"}

        def solve_once(seed):
            if seed is None:
                return None
            try:
                return self.pyb_robot.inverse_kinematics(
                    frame_WCF=frame, start_configuration=seed, options=options
                )
            except StopIteration:
                return None
            except Exception:
                return None

        def collision_free(cfg):
            if cfg is None or not do_collision_check:
                return cfg is not None

            restore_cfg = (self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
                        or getattr(self, "_prev_cfg_cache", None)
                        or self.pyb_robot.zero_configuration())
            try:
                self.pyb_client.set_robot_configuration(self.pyb_robot, cfg)
                try:
                    self.pyb_client.check_robot_self_collision(self.pyb_robot)  # raises on collision
                    collides = False
                except Exception as e:
                    collides = (e.__class__.__name__ == "CollisionError") or True
            finally:
                self.pyb_client.set_robot_configuration(self.pyb_robot, restore_cfg)
                self._prev_cfg_cache = restore_cfg
            return not collides

        def maybe_visualize(cfg):
            # throttle sim updates to avoid slowdown
            now = time.perf_counter()
            last = getattr(self, "_last_visual_step_t", 0.0)
            if now - last >= 1.0 / max(1, visual_hz):
                self.pyb_client.set_robot_configuration(self.pyb_robot, cfg)  # commit accepted IK to sim
                self.pyb_client.step_simulation()                         # single step
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

        print(f"CombinedBackendHandler: [{self.robot_name}] IK failed.")
        return None

    def pyb_log_self_collisions(self, ignored_pairs=None, threshold=0.001):
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
        robot_uid = self.pyb_client.client_id
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

    def _pyb_is_valid(self, cfg):
        """Check if a configuration is collision-free in the current PyBullet scene."""
        self.pyb_client.set_robot_configuration(self.pyb_robot, cfg)
        self.pyb_client.step_simulation()
        return not self.pyb_client.check_robot_self_collision(self.pyb_robot)

    ##### ROS IK Methods #####

    def ros_find_ik(self, frame, start_config=None, options=None):
        if start_config is None:
            start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="ROS")

        ik_config = self.ros_robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options=options)
        if ik_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] No IK solution found in ROS.")
            return None

        return ik_config


    ####################################################################################################
    # Planning and Multi-Configuration Solving
    ####################################################################################################

    ##### PyBullet Planning Methods #####

    def pyb_plan_ik_for_frames_list_compas_fab_itter(self, frames_for_ik, start_config, options=None, max_results=20) -> List[Configuration]:
        """
        Plans a joint trajectory for a list of target frames using iterative IK search.

        Parameters
        ----------
        frames_for_ik : list of compas.geometry.Frame
            The target frames for the IK.
        start_config : Configuration
            The starting configuration to compare against.
        options : dict, optional
            Additional options to pass to the IK solver.
        max_results : int, optional
            Max number of solutions to evaluate per frame.

        Returns
        -------
        List[Configuration] or None
            The planned joint trajectory.
        """
        configurations = []
        current_config = start_config

        for idx, frame in enumerate(frames_for_ik):
            try:
                ik_config = self.pyb_find_best_valid_ik_compas_fab_itter_ik_improved(frame, current_config, options, max_results)
                if ik_config is None:
                    print(f"CombinedBackendHandler: [{self.robot_name}] No valid IK for frame {idx}. Aborting trajectory planning.")
                    return None
                configurations.append(ik_config)
            except Exception as e:
                print(f"CombinedBackendHandler: [{self.robot_name}] Error finding IK for frame {idx}: {e}")
                return None
        return configurations

    def _pyb_plan_trajectories_for_user_initiated_request(self, configurations: List[Configuration]) -> List[JointTrajectory]:

        if not configurations:
            print(f"CombinedBackendHandler: [{self.robot_name}] No configurations provided.")
            return []

        # Start from current robot state
        start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="Pybullet")
        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Could not read current joint state.")
            return []

        trajectories: List[JointTrajectory] = []
        prev = start_config

        for i, goal in enumerate(configurations):
            try:
                traj = self._pyb_plan_free_motion_trajectory_pybullet_planning_test(prev, goal)
            except Exception as e:
                print(
                    f"RealtimeMimicPyBulletHandler: [{self.robot_name}] "
                    f"Error planning trajectory leg {i} from {prev} to {goal}: {e}"
                )
                return []  # keep return type consistent

            if not traj:
                print(
                    f"RealtimeMimicPyBulletHandler: [{self.robot_name}] "
                    f"Planner returned None for leg {i} ({prev} -> {goal})."
                )
                return []

            trajectories.append(traj)
            prev = goal  # next leg starts where this one ends

        #TODO: TESTING THIS FOR NOW....
        json_dump(trajectories, fp=r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_data_saves\test_trajectory_pybullet_planning.json", pretty=True)
        return trajectories

    def _pyb_plan_free_motion_trajectory_pybullet_planning_test(self, start_config, goal_config, num_steps=10):
        if start_config is None or goal_config is None:
            raise ValueError("Start and end configurations must be provided.")

        # --- 1) pick the robot body (simplest heuristic: body with most joints)
        if pb.getNumBodies() == 0:
            print(f"[{self.robot_name}] No bodies in PyBullet world.")
            return None
        body_id = max(range(pb.getNumBodies()), key=lambda i: pb.getNumJoints(i))

        # --- 2) joint names -> indices (fresh each call; tiny overhead, very simple)
        name_to_idx = {pb.getJointInfo(body_id, j)[1].decode(): j
                    for j in range(pb.getNumJoints(body_id))}
        group_joint_names = self.pyb_robot.get_configurable_joint_names(self.group)
        try:
            joint_ids = [name_to_idx[n] for n in group_joint_names]
        except KeyError as e:
            missing = str(e).strip("'")
            print(f"[{self.robot_name}] Joint '{missing}' not found in PyBullet. "
                f"Available: {list(name_to_idx.keys())}")
            return None

        # --- 3) (optional) quick limits sanity
        if not self._pyb_is_valid(start_config):
            print(f"[{self.robot_name}] Start configuration violates limits.")
            return None
        if not self._pyb_is_valid(goal_config):
            print(f"[{self.robot_name}] Goal configuration violates limits.")
            return None

        # --- 4) mirror start state into sim (helps collision queries & planning)
        set_joint_positions(body_id, joint_ids, list(start_config.joint_values))

        # --- 5) plan with pybullet_planning (RRT-Connect style)
        path = plan_joint_motion(
            body=body_id,
            joints=joint_ids,
            end_conf=list(goal_config.joint_values),   # radians, same order as group_joint_names
            restarts=2,
            iterations=4000,
            obstacles=[],                              # todo: ADD COLLISION OBJECTS IF YOU WANT....
        )
        if path is None:
            print(f"[{self.robot_name}] RRT failed: no path.")
            return None

        # --- 6) wrap as compas_fab JointTrajectory (so your calling code stays the same)
        traj_points = []
        t = 0.0
        for q in path:
            traj_points.append(JointTrajectoryPoint(joint_values=list(q), joint_types=self.pyb_robot.get_configurable_joint_types(), joint_names=self.pyb_robot.get_configurable_joint_names()))
            t += 0.02
        traj = JointTrajectory(traj_points, self.pyb_robot.get_configurable_joint_names(), start_config) #TODO: ADD ATTACHED COLLISION MESH HERE.
        return traj

    ##### ROS Planning Methods #####

    def _ros_plan_ik_for_frames_list(self, frames_for_ik, start_config, options=None) -> List[Configuration]:
        configurations = []
        current_config = start_config

        for idx, frame in enumerate(frames_for_ik):
            try:
                ik_config = self.ros_find_ik(frame, current_config, options)
                if ik_config is None:
                    print(f"CombinedBackendHandler: [{self.robot_name}] No valid IK for frame {idx}. Aborting trajectory planning.")
                    return None
                print (f"CombinedBackendHandler ROSSSSSSSSSSSSSSSSS : [{self.robot_name}] Found IK for frame {idx}: {ik_config}, with joint names: {ik_config.joint_names}")
                configurations.append(ik_config)
            except Exception as e:
                print(f"CombinedBackendHandler ROSSSSSSSSSSSSSSSSS : [{self.robot_name}] Error finding IK for frame {idx}: {e}")
                return None
        return configurations

    #TODO: Needs to be updated for the target information (based on if it is suppose to get a cube or not)
    def _ros_plan_trajectories_for_user_initiated_request(self, configurations: List[Configuration]) -> List[JointTrajectory]:
        if not configurations:
            print(f"CombinedBackendHandler: [{self.robot_name}] No configurations provided.")
            return []

        start_config = self._get_latest_joint_values_from_stream_as_configuration()
        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Could not read current joint state.")
            return []

        ros_joint_names = self.ros_robot.get_configurable_joint_names(self.group)
        ros_joint_types = self.ros_robot.get_configurable_joint_types(self.group)
        if (not getattr(start_config, "joint_names", None)) or (list(start_config.joint_names) != list(ros_joint_names)):
            start_config = Configuration(
                joint_names=ros_joint_names,
                joint_types=ros_joint_types,
                joint_values=list(start_config.joint_values),
            )

        trajectories: List[JointTrajectory] = []
        prev = start_config
        tolerance_above = self._generate_default_tolerances(self.ros_robot.get_configurable_joints(self.group))
        tolerance_below = self._generate_default_tolerances(self.ros_robot.get_configurable_joints(self.group))
        options_dict = dict(planner_id='TRRT', link_name="tool0")

        for i, goal in enumerate(configurations):
            try:
                if (not getattr(goal, "joint_names", None)) or (list(goal.joint_names) != list(ros_joint_names)):
                    goal = Configuration(
                        joint_names=ros_joint_names,
                        joint_types=ros_joint_types,
                        joint_values=list(goal.joint_values),
                    )

                start_cfg = prev if i == 0 else configurations[i-1]
                if (not getattr(start_cfg, "joint_names", None)) or (list(start_cfg.joint_names) != list(ros_joint_names)):
                    start_cfg = Configuration(
                        joint_names=ros_joint_names,
                        joint_types=ros_joint_types,
                        joint_values=list(start_cfg.joint_values),
                    )

                goal_constraints = self.ros_robot.constraints_from_configuration(
                    configuration=goal,
                    tolerances_above=tolerance_above,
                    tolerances_below=tolerance_below,
                    group=self.group
                )

                trajectory = self.ros_robot.plan_motion(
                    goal_constraints,
                    start_configuration=start_cfg,
                    group=self.group,
                    options=options_dict
                )

                if trajectory is None:
                    print(f"CombinedBackendHandler: [{self.robot_name}] Planner returned None for leg {i} ({prev} -> {goal}).")
                    return []

                if not getattr(trajectory, "joint_names", None):
                    trajectory.joint_names = list(ros_joint_names)
                for pt in trajectory.points:
                    if not getattr(pt, "joint_names", None):
                        pt.joint_names = list(ros_joint_names)
                    if not getattr(pt, "joint_types", None):
                        pt.joint_types = list(ros_joint_types)

                print(f"CombinedBackendHandler: [{self.robot_name}] Planned leg {i} with {len(trajectory.points)} points; joints={trajectory.joint_names}.")
                trajectories.append(trajectory)
                prev = goal

            except Exception as e:
                print(f"CombinedBackendHandler: [{self.robot_name}] Error planning trajectory leg {i} from {prev} to {goal}: {e}")
                return []

        return trajectories

    #######################################################################################################################
    # TODO: THESE ARE FOR PYBULLET BUT NEVER USED OR TESTED : Testing for creating cartesian motion with TryIK fast.
    #######################################################################################################################

    def slerp_quat(self, q0: Quaternion, q1: Quaternion, t: float) -> Quaternion:
        # compas Quaternion supports slerp via classmethod in newer versions; do manual if needed
        dot = q0.w*q1.w + q0.x*q1.x + q0.y*q1.y + q0.z*q1.z
        if dot < 0.0:
            q1 = Quaternion(-q1.w, -q1.x, -q1.y, -q1.z)
            dot = -dot
        if dot > 0.9995:
            # linear approx
            w = q0.w + t*(q1.w - q0.w)
            x = q0.x + t*(q1.x - q0.x)
            y = q0.y + t*(q1.y - q0.y)
            z = q0.z + t*(q1.z - q0.z)
            qq = Quaternion(w, x, y, z)
            qq.unitize()
            return qq
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        s0 = math.sin(theta_0 - theta) / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return Quaternion(
            s0*q0.w + s1*q1.w,
            s0*q0.x + s1*q1.x,
            s0*q0.y + s1*q1.y,
            s0*q0.z + s1*q1.z,
        )

    def frame_to_quat(self, frame: Frame) -> Quaternion:
        return Quaternion.from_frame(frame)

    def interpolate_frames(self, f0: Frame, f1: Frame, n: int) -> List[Frame]:
        # n segments => n+1 frames
        pts = []
        p0 = np.array([f0.point.x, f0.point.y, f0.point.z], dtype=float)
        p1 = np.array([f1.point.x, f1.point.y, f1.point.z], dtype=float)
        q0 = self.frame_to_quat(f0)
        q1 = self.frame_to_quat(f1)
        frames = []
        for i in range(n+1):
            t = i / float(n)
            p = (1-t)*p0 + t*p1
            q = self.slerp_quat(q0, q1, t)
            R = q.to_rotation_matrix()
            # rebuild x/y axes from rotation matrix
            from compas.geometry import Vector
            xaxis = Vector(*R[0])
            yaxis = Vector(*R[1])
            frames.append(Frame(p, xaxis, yaxis))
        return frames

    def max_joint_jump(self, prev_vals: List[float], new_vals: List[float]) -> float:
        return max(abs(a-b) for a, b in zip(prev_vals, new_vals))

    def plan_cartesian_with_ikfast(self,
        robot,                          # callable: (Frame, seed_cfg) -> Optional[Configuration]
        start_cfg: Configuration,
        start_frame: Frame,
        goal_frame: Frame,
        group: str = "manipulator",
        num_steps: int = 50,                    # user‑controlled subdivision
        dt: float = 0.02,
        jump_threshold: float = 1.5,            # rad; max per-step joint jump
        collision_check: bool = True,
        fallback_compas_ik: bool = True,        # try robot.inverse_kinematics if IKFast fails
        bisect_on_failure: bool = True,         # adaptive refine around failures
    ) -> Optional[JointTrajectory]:

        # Pre-check: start_cfg valid
        if not self._pyb_is_valid(start_cfg):
            print("[cartesian] start_cfg violates limits")
            return None

        frames = self.interpolate_frames(start_frame, goal_frame, num_steps)
        traj = JointTrajectory(robot.model, group)
        t = 0.0
        seed = start_cfg
        prev_vals = list(seed.values)

        i = 0
        while i < len(frames):
            f = frames[i]

            cfg = self.pyb_try_fast_ik(f, seed)  # primary solver
            if cfg is None and fallback_compas_ik:
                cfg = self.pyb_robot.inverse_kinematics(f, start_configuration=seed, group=group)

            if cfg is None:
                if bisect_on_failure and num_steps < 800:
                    # insert midpoint between frames[i-1] and frames[i], retry
                    if i == 0:
                        # fail at first step → cannot bisect
                        print(f"[cartesian] IK failed at first step {i}")
                        return None
                    # insert extra waypoint between frames[i-1] and frames[i]
                    mid = self.interpolate_frames(frames[i-1], frames[i], 2)[1]
                    frames.insert(i, mid)
                    num_steps += 1
                    continue
                print(f"[cartesian] IK failed at step {i}")
                return None

            # limits + optional collision
            if not self._pyb_is_valid(cfg):
                if bisect_on_failure and num_steps < 800:
                    if i == 0:
                        print(f"[cartesian] invalid/colliding at first step {i}")
                        return None
                    mid = self.interpolate_frames(frames[i-1], frames[i], 2)[1]
                    frames.insert(i, mid)
                    num_steps += 1
                    continue
                print(f"[cartesian] invalid/colliding at step {i}")
                return None

            # jump detection for continuity
            step_jump = self.max_joint_jump(prev_vals, cfg.joint_values)
            if step_jump > jump_threshold:
                if bisect_on_failure and num_steps < 800:
                    mid = self.interpolate_frames(frames[i-1], frames[i], 2)[1] if i > 0 else None
                    if mid is not None:
                        frames.insert(i, mid)
                        num_steps += 1
                        continue
                print(f"[cartesian] joint jump {step_jump:.3f} > {jump_threshold:.3f} at step {i}")
                return None

            # accept waypoint
            traj.points.append(JointTrajectoryPoint(values=cfg.joint_values, time_from_start=t))
            t += dt
            seed = cfg
            prev_vals = list(cfg.joint_values)
            i += 1

        return traj

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

    def _generate_default_tolerances(self, joints):
        DEFAULT_TOLERANCE_METERS = .001
        DEFAULT_TOLERANCE_RADIANS = math.radians(1)

        return [
            DEFAULT_TOLERANCE_METERS if j.is_scalable()
            else DEFAULT_TOLERANCE_RADIANS
            for j in joints
        ]    

    ####################################################################################################
    # EMPTY METHODS FOR CHILD CLASSES.
    ####################################################################################################

    def _get_current_configuration(self):
        if not self.realtime_mimic_ik_solutions:
            return self.pyb_robot.zero_configuration()
        else:
            "using last configuration as start configuration"
        return self.realtime_mimic_ik_solutions[-1]
    
    def _send_to_target(self, frame: Frame):
        print(f"CombinedBackendHandler: [{self.robot_name}] (Sim) Executing motion to target frame: {frame}")

    def _send_to_configuration(self, config: Configuration):
        print(f"CombinedBackendHandler: [{self.robot_name}] (Sim) Executing: {config.joint_values}")

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

    def _send_to_trajectory_RT(self, trajectory: JointTrajectory, io_begining_end_none):
        raise NotImplementedError("This method should be implemented on the child classes.")

    def _send_to_configuration_through_gate(self, config: Configuration):
        raise NotImplementedError("This method should be implemented on the child classes.")

    def shutdown(self):
        self.pyb_client.__exit__(None, None, None)

    def _toggle_tool_io(self, signal: int, value: int):
        raise NotImplementedError("This method should be implemented on the child classes.")

    def _execute_inference_pick_and_place(self, trajectory_list: List[JointTrajectory]):
        raise NotImplementedError("This method should be implemented on the child classes.")
    
    ####################################################################################################
    # MESSAGE HANDLERS RealtimeMimicResquestMessage
    ####################################################################################################

    def handle_realtime_msg_request_recursive_solver(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"CombinedBackendHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")

        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")

            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

        try:
            options = {"link_name": "tool0"}
            max_attempts = 8
            ik_config = self.pyb_find_valid_ik_recursive(frame, start_config, options, max_tries=max_attempts)
            if ik_config is None:
                self.pyb_client.set_robot_configuration(self.pyb_robot, start_config)
                self.pyb_client.step_simulation()
                print(f"CombinedBackendHandler: [{self.robot_name}] No valid IK solution found after {max_attempts} attempts.")
                return None

            self.realtime_mimic_ik_solutions.append(ik_config)
            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"CombinedBackendHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_compas_fab_itter(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: test run on the robot.
        """
        Checks recursively until it findes a valid IK that is collision free and returns it, but has a max attempt of 8. It returns the first solution without collision.
        """
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"CombinedBackendHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

        try:
            options = dict(
                link_name="tool0",
                high_accuracy_threshold=1e-6,
                high_accuracy_max_iter=8
            )
            ik_config = self.pyb_find_best_valid_ik_compas_fab_itter_ik(frame, start_config, options)

            if ik_config is None:
                self.pyb_client.set_robot_configuration(self.pyb_robot, start_config)
                self.pyb_client.step_simulation()
                return None

            self.realtime_mimic_ik_solutions.append(ik_config)
            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"CombinedBackendHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        if msg.initial_request:
            print(f"CombinedBackendHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
            self.realtime_mimic_ik_solutions = []
            # start_config = self._get_current_configuration()
            start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
        else:
            if len(self.realtime_mimic_ik_solutions) == 0:
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
            else:
                start_config = self.realtime_mimic_ik_solutions[-1]

        try:
            ik_config = self.pyb_robot.inverse_kinematics(frame_WCF=frame, start_configuration=start_config, options={"link_name": "tool0"}) #TODO: This tool0 param is hard coded for the UR20, it should be checked with the UR3 and ABB. Or passed as a paramater for the tool frame as well.
            self.pyb_client.set_robot_configuration(self.pyb_robot, ik_config)
            self.pyb_client.step_simulation()
            is_collision = self.pyb_client.check_robot_self_collision(self.pyb_robot)
            if is_collision:
                print(f"CombinedBackendHandler: [{self.robot_name}] IK solution is in collision. Logging details:")
                self.pyb_log_self_collisions()
                if self.realtime_mimic_ik_solutions:
                    self.pyb_client.set_robot_configuration(self.pyb_robot, self.realtime_mimic_ik_solutions[-1])
                    self.pyb_client.step_simulation()
                else:
                    self.pyb_client.set_robot_configuration(self.pyb_robot, self.pyb_robot.zero_configuration())
                    self.pyb_client.step_simulation()
                return None
            self.realtime_mimic_ik_solutions.append(ik_config)

            self._send_to_configuration(ik_config)
            fp = os.path.join(os.path.dirname(__file__), "ik_configurations_pybullet.json")
            json_dump(self.realtime_mimic_ik_solutions, fp=fp, pretty=True)
            return ik_config
        except Exception as e:
            print(f"CombinedBackendHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_ik_target(self, msg: RealtimeMimicRequestMessage) -> Configuration: #TODO: Using this one, and check the visualization, but run on the robot.
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling request: {msg.message} from {msg.header.device_id}")
        frame = msg.requested_robot_frame

        try:
            if msg.initial_request:
                print(f"CombinedBackendHandler: [{self.robot_name}] Initial request received. Resetting IK solutions.")
                self.realtime_mimic_ik_solutions = []
                # start_config = self._get_current_configuration()
                start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
                options = dict(
                    link_name="tool0",
                    high_accuracy_threshold=1e-6,
                    high_accuracy_max_iter=8
                )
                ik_config = self.pyb_find_best_valid_ik_compas_fab_itter_ik(frame, start_config, options)

                if ik_config is None:
                    self.pyb_client.set_robot_configuration(self.pyb_robot, start_config)
                    self.pyb_client.step_simulation()
                    return None

                self.realtime_mimic_ik_solutions.append(ik_config)
                self._send_to_configuration(frame)
            else:
                print(f"CombinedBackendHandler: [{self.robot_name}] Non-initial request received. Executing motion to target frame.")
                self._send_to_target(frame)
                return frame
        except Exception as e:
            print(f"CombinedBackendHandler: [{self.robot_name}] IK computation failed: {e}")
            return None

    def handle_realtime_msg_request_fastest_ik(self, msg: RealtimeMimicRequestMessage):
        # pull the requested frame
        frame = msg.requested_robot_frame

        # choose a seed
        if msg.initial_request:
            # reset history on first call (optional)
            self.realtime_mimic_ik_solutions = []

        if msg.initial_request or not self.realtime_mimic_ik_solutions:
            start_cfg = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
            if start_cfg is None:
                # rare fallback if stream isn't ready yet
                start_cfg = self.pyb_robot.zero_configuration()
        else:
            start_cfg = self.realtime_mimic_ik_solutions[-1]

        # fast IK: current-joints seed, optional single fallback (last good)
        ik = self.pyb_try_fast_ik(
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

    def handle_realtime_mimic_io_toggle_request(self, msg: RealtimeMimicIOToggleRequestMessage):
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling IO toggle request: {msg} from {msg.header.device_id}")
        try:
            self._toggle_tool_io(signal=msg.signal, value=msg.value)
            print(f"CombinedBackendHandler: [{self.robot_name}] Successfully toggled IO signal {msg.signal} to {msg.value}.")
            return True
        except Exception as e:
            print(f"CombinedBackendHandler: [{self.robot_name}] Failed to toggle IO signal: {e}")
            return False

    ####################################################################################################
    # MESSAGE HANDLERS MimicResquestMessage
    ####################################################################################################

    def handle_user_iniated_msg_request(self, msg: RealtimeMimicRequestMessage) -> List[JointTrajectory]:
        """
        This method can be overridden by child classes to handle custom message requests.
        """
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling user-defined request: {msg} from {msg.header.device_id}")
        start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="PyBullet")
        options = dict(
                link_name="tool0",
                high_accuracy_threshold=1e-6,
                high_accuracy_max_iter=8
            )

        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid start configuration found. Returning empty trajectory.")
        configs_for_planning = self.pyb_plan_ik_for_frames_list_compas_fab_itter(msg.robot_frames, start_config, options=options, max_results=20)
        if configs_for_planning is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid configurations found for planning. Returning empty trajectory.")
        else:
            print(f"CombinedBackendHandler: [{self.robot_name}] Valid configurations found for planning. Found {len(configs_for_planning)} configs for planning.")        

        trajectories = self._pyb_plan_trajectories_for_user_initiated_request(configurations=configs_for_planning)
        if len(trajectories) < 1:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid trajectories found for planning. Returning empty trajectory.")
            return None
        
        print(f"CombinedBackendHandler: [{self.robot_name}] Valid trajectories found for planning. Found {len(trajectories)} trajectories for planning.")
        return trajectories

    def handle_user_iniated_msg_request_ROS(self, msg: RealtimeMimicRequestMessage) -> List[JointTrajectory]:
        """
        This method can be overridden by child classes to handle custom message requests.
        """
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling user-defined request: {msg} from {msg.header.device_id}")
        start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="ROS")
        options = dict(
                link_name="tool0",
                high_accuracy_threshold=1e-6,
                high_accuracy_max_iter=8
            )

        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid start configuration found. Returning empty trajectory.")
        configs_for_planning = self._ros_plan_ik_for_frames_list(msg.robot_frames, start_config, options=options)
        if configs_for_planning is None:
            print(f"CombinedBackendHandler ROSSSSSSSSSSSSSSSSSSSSSSSSSSSSS : [{self.robot_name}] No valid configurations found for planning. Returning empty trajectory.")
        else:
            print(f"CombinedBackendHandler ROSSSSSSSSSSSSSSSSSSSSSSSSSSSSS : [{self.robot_name}] Valid configurations found for planning. Found {len(configs_for_planning)} configs for planning.")        

        trajectories = self._ros_plan_trajectories_for_user_initiated_request(configurations=configs_for_planning)
        if len(trajectories) < 1:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid trajectories found for planning. Returning empty trajectory.")
            return None
        
        # print(f"CombinedBackendHandler: [{self.robot_name}] Valid trajectories found for planning. Found {len(trajectories)} trajectories for planning.")
        return trajectories

    def handle_user_initiated_mimic_execution(self, msg: ExecuteMimicTrajectoryRequestMessage, trajectory_list: List[JointTrajectory]):
        """
        This method should be overridden by child classes to handle custom mimic execution requests.
        """
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling user-defined mimic execution request: {msg} from {msg.header.device_id}")
        #TODO: MESSAGE NEEDS TO BE EXTENDED TO HANDLE AN IO ON OFF LIST OF THINGS
        if not trajectory_list:
            print(f"CombinedBackendHandler: [{self.robot_name}] No trajectories to execute.")
            return False

        for traj in trajectory_list:
            try:
                #TODO: The IO Beginning, End, None needs to be controled by the message or planning (when to turn on and off the IO).
                self._send_to_trajectory_RT(trajectory=traj, io_begining_end_none=0)
                print(f"CombinedBackendHandler: [{self.robot_name}] Successfully executed trajectory.")
            except Exception as e:
                print(f"CombinedBackendHandler: [{self.robot_name}] Failed to execute trajectory: {e}")
                return False
        return True

    #####################################################################################################
    # Inference Handlers
    #####################################################################################################

    def _align_config(self, cfg: Configuration, ros_joint_names, ros_joint_types) -> Configuration:
        if (not getattr(cfg, "joint_names", None)) or (list(cfg.joint_names) != list(ros_joint_names)):
            return Configuration(
                joint_names=list(ros_joint_names),
                joint_types=list(ros_joint_types),
                joint_values=list(cfg.joint_values),
            )
        return cfg

    def _stamp_joint_meta(self, traj: JointTrajectory, ros_joint_names, ros_joint_types) -> None:
        if not getattr(traj, "joint_names", None):
            traj.joint_names = list(ros_joint_names)
        for pt in traj.points:
            if not getattr(pt, "joint_names", None):
                pt.joint_names = list(ros_joint_names)
            if not getattr(pt, "joint_types", None):
                pt.joint_types = list(ros_joint_types)

    def _reverse_trajectory(self, traj: JointTrajectory) -> JointTrajectory:
        # Reverse point order and rebuild time_from_start cumulatively
        if not traj.points:
            return JointTrajectory(joint_names=list(getattr(traj, "joint_names", []) or []), points=[])

        trajectory_points = traj.points
        reversed_points = trajectory_points[::-1]
        start_configuration = reversed_points[0]

        return JointTrajectory(joint_names=list(traj.joint_names), trajectory_points=reversed_points, start_configuration=start_configuration, attached_collision_meshes=traj.attached_collision_meshes)

    # TODO: attach/update collision meshes based on placed/unplaced dicts before planning
    def _ros_plan_trajectories_for_inference(self, configurations: List[Configuration],
                                            placed_blocks_dict, unplaced_blocks_dict) -> List[JointTrajectory]:
        """
        Expects configurations in this order (length >= 5):
            0: exit_safe
            1: approach_pick
            2: pick
            3: approach_place
            4: place
        Plans legs:
            start -> 0 (cartesian)
            0 -> 1 (free)
            1 -> 2 (cartesian)
            reverse of [1 -> 2] (cartesian reversed: 2 -> 1)
            1 -> 3 (free)
            3 -> 4 (cartesian)
        """
        if not configurations or len(configurations) < 5:
            print(f"CombinedBackendHandler: [{self.robot_name}] Need at least 5 configs (exit_safe, approach_pick, pick, approach_place, place).")
            return []

        # Start config from stream
        start_config = self._get_latest_joint_values_from_stream_as_configuration()
        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Could not read current joint state.")
            return []

        ros_joint_names = self.ros_robot.get_configurable_joint_names(self.group)
        ros_joint_types = self.ros_robot.get_configurable_joint_types(self.group)
        start_config = self._align_config(start_config, ros_joint_names, ros_joint_types)

        # Align all provided configs
        cfgs = [self._align_config(c, ros_joint_names, ros_joint_types) for c in configurations]
        exit_safe, approach_pick, pick, approach_place, place = cfgs[:5]

        trajectories: List[JointTrajectory] = []

        # Common options
        free_options = dict(planner_id='TRRT', link_name="tool0")
        # Tune max_step/jump_threshold if needed for your setup
        cart_options = dict(link_name="tool0", avoid_collisions=True, max_step=0.05, jump_threshold=0.0)

        # Helper: cartesian plan from A cfg to B cfg via B's EE frame
        def plan_cartesian(a_cfg: Configuration, b_cfg: Configuration) -> Optional[JointTrajectory]:
            b_frame = self.ros_robot.forward_kinematics(b_cfg, options=dict(link_name="tool0"), group=self.group)
            if b_frame is None:
                print(f"CombinedBackendHandler: [{self.robot_name}] FK failed for cartesian target.")
                return None
            traj = self.ros_robot.plan_cartesian_motion(
                [b_frame],
                start_configuration=a_cfg,
                group=self.group,
                options = dict(link_name=cart_options["link_name"],
                max_step=cart_options["max_step"])
            )
            return traj

        # Helper: free plan from A cfg to B cfg
        def plan_free(a_cfg: Configuration, b_cfg: Configuration) -> Optional[JointTrajectory]:
            goal_constraints = self.ros_robot.constraints_from_configuration(
                configuration=b_cfg,
                tolerances_above=self._generate_default_tolerances(self.ros_robot.get_configurable_joints(self.group)),
                tolerances_below=self._generate_default_tolerances(self.ros_robot.get_configurable_joints(self.group)),
                group=self.group
            )
            traj = self.ros_robot.plan_motion(
                goal_constraints,
                start_configuration=a_cfg,
                group=self.group,
                options=free_options
            )
            return traj

        # ---- Leg 1: start -> exit_safe (cartesian)
        traj = plan_cartesian(start_config, exit_safe)
        if traj is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Leg 1 cartesian failed.")
            return []
        self._stamp_joint_meta(traj, ros_joint_names, ros_joint_types)
        trajectories.append(traj)

        # ---- Leg 2: exit_safe -> approach_pick (free)
        traj = plan_free(exit_safe, approach_pick)
        if traj is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Leg 2 free failed.")
            return []
        self._stamp_joint_meta(traj, ros_joint_names, ros_joint_types)
        trajectories.append(traj)

        # ---- Leg 3: approach_pick -> pick (cartesian)
        traj_pick_fwd = plan_cartesian(approach_pick, pick)
        if traj_pick_fwd is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Leg 3 cartesian (approach_pick->pick) failed.")
            return []
        self._stamp_joint_meta(traj_pick_fwd, ros_joint_names, ros_joint_types)
        trajectories.append(traj_pick_fwd)

        # ---- Leg 4: reverse of Leg 3 (pick -> approach_pick)
        traj_pick_rev = self._reverse_trajectory(traj_pick_fwd)
        self._stamp_joint_meta(traj_pick_rev, ros_joint_names, ros_joint_types)
        trajectories.append(traj_pick_rev)

        # ---- Leg 5: approach_pick -> approach_place (free)
        traj = plan_free(approach_pick, approach_place)
        if traj is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Leg 5 free failed.")
            return []
        self._stamp_joint_meta(traj, ros_joint_names, ros_joint_types)
        trajectories.append(traj)

        # ---- Leg 6: approach_place -> place (cartesian)
        traj = plan_cartesian(approach_place, place)
        if traj is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Leg 6 cartesian failed.")
            return []
        self._stamp_joint_meta(traj, ros_joint_names, ros_joint_types)
        trajectories.append(traj)

        print(f"CombinedBackendHandler: [{self.robot_name}] Planned {len(trajectories)} legs (C, F, C, C-rev, F, C).")
        return trajectories

    ######### TODO: Helper Functions ####################################################################

    def offset_frame_by_distance(self, frame: Frame, vector: Vector, distance: float ) -> Frame:
        direction = vector.unitized()
        offset_vector = direction * distance
        tx = Translation.from_vector(offset_vector)
        new_frame = frame.transformed(tx)
        return new_frame

    def flip_plane_test(self, frame: Frame) -> Frame:
            """Return a new Frame with its plane normal (z) flipped 180°.
            Keeps right-handedness by also flipping x.
            """
            p = frame.point
            x = -frame.xaxis
            z = -frame.zaxis
            out = Frame(p, x, z)

            # sanity: right-handed check
            if (out.xaxis.cross(out.yaxis)).dot(out.zaxis) < 0.999:
                raise ValueError("flip_plane produced a non right-handed frame.")
            return out

    def flip_frame_around_axis(self, frame: Frame, axis: str = "z") -> Frame:
        """
        Return a new frame that is flipped 180 degrees around the given axis.

        Parameters
        ----------
        frame : compas.geometry.Frame
            The frame to flip.
        axis : str, optional
            The local axis to flip around ("x", "y", or "z"). Default is "z".

        Returns
        -------
        Frame
            A new flipped frame.
        """
        # Pick the axis vector
        if axis == "x":
            axis_vec = frame.xaxis
        elif axis == "y":
            axis_vec = frame.yaxis
        elif axis == "z":
            axis_vec = frame.zaxis
        else:
            raise ValueError("Axis must be 'x', 'y', or 'z'.")

        # Define rotation (180° about chosen axis, at frame origin)
        R = Rotation.from_axis_and_angle(axis_vec, math.radians(180), frame.point)

        # Copy frame (to avoid mutating input) and apply
        flipped = frame.copy()
        flipped.transform(R)
        return flipped

    #TODO: Needs to have collision meshes attached. and planning options.
    def handle_planning_for_inference(self, closest_target_frame, transformed_target, 
                                    transformed_completed_items_dict, transformed_incompleted_items_dict, 
                                    closest_target_name):
        print(f"CombinedBackendHandler: [{self.robot_name}] Handling inference planning request for target: {closest_target_name}")

        start_config = self._get_latest_joint_values_from_stream_as_configuration(backend="ROS")
        if start_config is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid start configuration found. Returning empty trajectory.")
            return []

        current_tool_frame = self.ros_robot.forward_kinematics(start_config, options=dict(link_name="tool0"), group=self.group)
        if current_tool_frame is None:
            print(f"CombinedBackendHandler: [{self.robot_name}] Could not compute current tool frame. Returning empty trajectory.")
            return []

        # Frames # TODO: These Frames Need to be flipped. THIS NEEDS TO BE DONE BIG TIME.....
        # exit_safe_frame       = self.offset_frame_by_distance(current_tool_frame, current_tool_frame.zaxis, -0.2)    # 20 cm up
        # pick_frame = self.offset_frame_by_distance(closest_target_frame, closest_target_frame.zaxis, 0.15) # pick from 15 cm below
        # approach_pick_frame   = self.offset_frame_by_distance(pick_frame, pick_frame.zaxis, 0.4)                    # 40 cm above pick
        # place_frame = self.offset_frame_by_distance(transformed_target, transformed_target.zaxis, 0.15) # place from 15 cm above
        # approach_place_frame  = self.offset_frame_by_distance(place_frame, place_frame.zaxis, 0.4)                  # 40 cm above place
        #TODO: Testing flipping of planes
        closest_target_frame = self.flip_frame_around_axis(closest_target_frame, axis="y")
        # closest_target_frame = self.flip_plane_test(closest_target_frame)
        transformed_target = self.flip_frame_around_axis(transformed_target, axis="y")
        # transformed_target = self.flip_plane_test(transformed_target)

        exit_safe_frame       = self.offset_frame_by_distance(current_tool_frame, current_tool_frame.zaxis, -0.2)    # 20 cm up
        pick_frame = self.offset_frame_by_distance(closest_target_frame, closest_target_frame.zaxis, -0.15) # pick from 15 cm below
        approach_pick_frame   = self.offset_frame_by_distance(pick_frame, pick_frame.zaxis, -0.4)                    # 40 cm above pick
        place_frame = self.offset_frame_by_distance(transformed_target, transformed_target.zaxis, -0.15) # place from 15 cm above
        approach_place_frame  = self.offset_frame_by_distance(place_frame, place_frame.zaxis, -0.4)                  # 40 cm above place

        data = {}
        data["current_tool_frame"] = current_tool_frame
        data["closest_target_frame"] = closest_target_frame
        data["transformed_target"] = transformed_target
        data["exit_safe_frame"] = exit_safe_frame
        data["pick_frame"] = pick_frame
        data["approach_pick_frame"] = approach_pick_frame
        data["place_frame"] = place_frame
        data["approach_place_frame"] = approach_place_frame
        data["completed_items_dict"] = transformed_completed_items_dict
        data["incompleted_items_dict"] = transformed_incompleted_items_dict
        data["colsest_target_name"] = closest_target_name
        data["current_config"] = start_config
        json_dump(data, os.path.join(os.path.dirname(__file__), "planning_frames_debug.json"), pretty=True)


        # Correct order for IK (match the planning legs below)
        frames_for_ik = [
            exit_safe_frame,        # idx 0
            approach_pick_frame,    # idx 1
            pick_frame,             # idx 2
            approach_place_frame,   # idx 3
            place_frame,            # idx 4
        ]

        ik_options = dict(link_name="tool0", high_accuracy_threshold=1e-6, high_accuracy_max_iter=8)

        configs_for_planning = self._ros_plan_ik_for_frames_list(frames_for_ik, start_config, options=ik_options)
        if not configs_for_planning:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid configurations found for planning. Returning empty trajectory.")
            return []

        # Plan per the required sequence
        # Plan per the required sequence
        try:
            trajectories = self._ros_plan_trajectories_for_inference(
                configurations=configs_for_planning,
                placed_blocks_dict=transformed_completed_items_dict,
                unplaced_blocks_dict=transformed_incompleted_items_dict,
            )
        except Exception as e:
            print(
                f"CombinedBackendHandler: [{self.robot_name}] "
                f"Error while planning trajectories: {e}. Returning empty trajectory."
            )
            return []

        if len(trajectories) < 1:
            print(f"CombinedBackendHandler: [{self.robot_name}] No valid trajectories found for planning. Returning empty trajectory.")
            return []
        print(f"CombinedBackendHandler: [{self.robot_name}] Valid trajectories found for planning. Found {len(trajectories)} trajectories for planning.")
        return trajectories


        # For now just load information from a file.
        # fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\testing\random_trajectory_for_testing.json"
        # sample_message = json_load(fp)
        # trajectories = sample_message["trajectories"]
        # robot_base_frame = sample_message["robot_base_frame"]
        # robot_name = sample_message["robot_name"]
        # return trajectories, robot_base_frame, robot_name


class URMimicHandlerCombined(RobotHandlerCombinedBackends):

    def __init__(self, robot_name, robot_ip, urdf_path, tool_info_fp, ros_ip='127.0.0.1', ros_port=9090, additional_static_collision_meshes_fp=None, group="manipulator", srdf_path=None, io=0, speed=0.6, acceleration=0.1, radius=0.006, nowait=False):
        super().__init__(robot_name, urdf_path, tool_info_fp, ros_ip=ros_ip, ros_port=ros_port, additional_static_collision_meshes_fp=additional_static_collision_meshes_fp, group=group, srdf_path=srdf_path)

        self.robot_state_streamer = RTDEStateStreamer(robot_ip=robot_ip, poll_delay=0.001, sim=False)
        self.robot_state_streamer.start()

        self.robot_ip = robot_ip
        self.speed = speed
        self.acceleration = acceleration
        self.radius = radius
        self.nowait = nowait 
        self.io = io

        # TODO: Commented out to test the combined handler without RTDE connections
        # persist RTDE connections once
        self.rtde_ctrl = RTDEControl(self.robot_ip)
        self.rtde_recv = RTDEReceive(self.robot_ip)
        # TODO: Commented out to test the combined handler without RTDE connections

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

        #TODO: THIS WAS THE BEST...... I don't know if I use this or not, but it was the best for MoveJGate. Comment back in if you want to use
        # self.movej_gate = MoveJGate(self.rtde_ctrl,
        #     speed=self.speed, accel=0.9,
        #     min_dt=999,            # disable single sends
        #     min_dq=1e9,            # disable single sends
        #     blend_radius=0.012,    # 12 mm blend
        #     blend_batch=4,         # 3–5 points
        #     blend_every=0.40       # ~2.5 Hz flush
        # )
        #TODO: THIS WAS THE BEST...... I don't know if I use this or not, but it was the best for MoveJGate.

        # self.servo_gate = ServoJGate(
        #     rtde_ctrl=self.rtde_ctrl,
        #     rtde_recv=self.rtde_recv,
        #     speed_cap=max(0.4, min(0.9, self.speed if self.speed else 0.8)),  # rad/s
        #     accel_cap=max(0.8, min(1.8, self.acceleration if self.acceleration else 1.5)),  # rad/s^2
        #     dt_nominal=1/125.0,
        #     lookahead=0.10,
        #     gain=280,
        #     target_alpha=0.25,   # smoothing on targets
        #     k_speed=1.6,         # adaptive speed scaling
        #     tau=0.30,            # accel ≈ speed / tau
        #     cmd_alpha=0.35,      # smoothing on caps
        #     min_dt_send=0.010,   # don’t spam faster than 100 Hz
        #     min_dq=0.0,
        #     verbose=False
        # )


        # last attempt to make it smoother.
        #TODO: Commented in before actual execution...
        # self.servo_gate = ServoJGate(
        #     rtde_ctrl=self.rtde_ctrl,
        #     rtde_recv=self.rtde_recv,
        #     # caps a bit lower → smoother
        #     speed_cap=0.65,          # was 0.8
        #     accel_cap=1.10,          # was 1.5

        #     # UR controller
        #     dt_nominal=1/125.0,
        #     lookahead=0.12,          # was 0.10
        #     gain=240,                # was 280

        #     # filtering + adaptive scaling
        #     target_alpha=0.18,       # LOWER = more smoothing (was 0.25)
        #     k_speed=1.2,             # was 1.6
        #     tau=0.38,                # accel = speed/tau (bigger tau = softer accel), was 0.30
        #     cmd_alpha=0.55,          # smoother speed/acc commands (was 0.35)

        #     # sending cadence + deadband
        #     min_dt_send=0.012,       # was 0.010
        #     min_dq=0.01,             # ~0.57° joint deadband to avoid chatter
        #     verbose=False
        # )
        # #TODO: Commented in before actual execution...

        pace = 0.5  # 50% speed
        self.servo_gate = ServoJGate(
            rtde_ctrl=self.rtde_ctrl,
            rtde_recv=self.rtde_recv,
            speed_cap=0.65 * pace,
            accel_cap=1.10 * pace,
            dt_nominal=1/125.0,
            lookahead=0.12,
            gain=240,
            target_alpha=0.18,
            k_speed=1.2 * pace,  # include if k_speed scales velocity
            tau=0.38,
            cmd_alpha=0.55,
            min_dt_send=0.012,
            min_dq=0.01,
            verbose=False
        )

        print(f"URCombinedBackendHandler: [{robot_name}] UR handler initialized")

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
        print(f"URCombinedBackendHandler: [{self.robot_name}] closed")

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
        print(f"URCombinedBackendHandler: [{self.robot_name}] (Sim) Executing UR motion: {config.joint_values}")
        # rtde.move_to_joints(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        # rtde.move_to_joints_blend(config, self.speed, self.acceleration, blend=self.radius, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_joints_TEST(config, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    def _send_to_target(self, frame: Frame):
        print(f"URCombinedBackendHandler: [{self.robot_name}] (Sim) Executing UR motion to target frame: {frame}")
        # rtde.move_to_target(frame, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)
        rtde.move_to_target(frame, self.speed, self.acceleration, nowait=True, ip=self.robot_ip)
        # rtde.move_to_target_TEST(frame, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    def _send_to_trajectory_RT(self, trajectory: JointTrajectory, io_begining_end_none):
        print(f"URCombinedBackendHandler: [{self.robot_name}] (Sim) Executing UR trajectory: {trajectory}")
        rtde.send_to_single_trajectory_robotic_territories(trajectory.points, self.speed, self.acceleration, self.radius, self.rtde_ctrl, 0, self.io)
        # rtde.send_to_single_trajectory_robotic_territories_TEST(trajectory, self.speed, self.acceleration, self.radius, self.robot_ip, io_begining_end_none, vaccum_io=self.io)
        # rtde.send_to_single_trajectory_robotic_territories(trajectory, self.speed, self.acceleration, nowait=self.nowait, ip=self.robot_ip)

    def _toggle_tool_io(self, signal: int, value: int):
        rtde.set_tool_digital_io(signal, value, self.robot_ip)
        print(f"URCombinedBackendHandler: [{self.robot_name}] (Sim) Toggled tool IO signal {signal} to {value}.")

    def _execute_inference_pick_and_place(self, trajectory_list: List[JointTrajectory]):
        print (f"URCombinedBackendHandler: [{self.robot_name}] Executing inference pick-and-place trajectories of Length {len(trajectory_list)}.")
        print (f"URCOMBINEDBACKENDHANDLER: IMPLEMENT THE ACTUAL EXECUTION HERE....through RTDE custom routine.")
        rtde.send_pick_and_place_trajectory_RT_inference(trajectory_list, self.speed, self.acceleration, self.rtde_ctrl, self.radius, self.robot_ip)
    ####################################################################################################
    # Implemented through Streamer Class Interface
    ####################################################################################################

    #TODO: TESTING BIG TIME
    def handle_realtime_msg_request_servoj_gate(self, msg: RealtimeMimicRequestMessage):
        """Compute IK fast and stream joints via servoj (no threads)."""
        frame = msg.requested_robot_frame

        # reset history on first call
        if msg.initial_request:
            fp = os.path.join(os.path.dirname(__file__), "realtime_mimic_ik_solutions.json")
            json_dump(self.realtime_mimic_ik_solutions, fp, pretty=True)
            self.realtime_mimic_ik_solutions = []

        # choose seed: current joints if no history, else last good
        if msg.initial_request or not self.realtime_mimic_ik_solutions:
            start_cfg = self._get_latest_joint_values_from_stream_as_configuration()
            if start_cfg is None:
                start_cfg = self.pyb_robot.zero_configuration()
        else:
            start_cfg = self.realtime_mimic_ik_solutions[-1]

        # fast IK (single seed + optional fallback); 30 Hz sim mirror inside
        ik = self.pyb_try_fast_ik(frame,
                            start_config=start_cfg,
                            do_collision_check=True,
                            extra_seed=True,
                            visual_hz=30)

        if ik:
            # # #TODO: Comment me in if you want to run on sim only....
            self.realtime_mimic_ik_solutions.append(ik)
            print(f"[{self.robot_name}] (SIM) would send with servoj, skipping actual send.")
            fp = os.path.join(os.path.dirname(__file__), "realtime_mimic_ik_solutions.json")
            json_dump(self.realtime_mimic_ik_solutions, fp, pretty=True)
            return ik 
            # # #TODO: Comment me in if you want to run on sim only....
            # remember and command through servoj
            self.realtime_mimic_ik_solutions.append(ik)
            self.servo_gate.set_target(ik.joint_values)
            self.servo_gate.tick()
            return ik
        # # #TODO: Comment me in if you want to run on sim only...
        # IK failed: keep feeding servo with last known good (or current measured)
        print(f"[{self.robot_name}] (SIM) IK failed, would normally keep feeding servo — skipping send.")
        return None
        # # #TODO: Comment me in if you want to run on sim only...
        # # IK failed: keep feeding servo with last known good (or current measured)
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
            print(f"URCombinedBackendHandler: [{self.robot_name}] moveJ sent (speed={self.speed}, accel={self.acceleration})")

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

    def _get_latest_joint_values_from_stream_as_configuration(self, backend="ROS"):
        q = self._get_latest_joint_values_from_stream()
        if q:
            if backend == "ROS":
                joint_names = self.ros_robot.get_configurable_joint_names()
                joint_types = self.ros_robot.get_configurable_joint_types()
            elif backend == "PyBullet":
                joint_names = self.pyb_robot.get_configurable_joint_names()
                joint_types = self.pyb_robot.get_configurable_joint_types()
            else:
                raise ValueError(f"Unknown backend: {backend}. Supported backends are 'ROS' and 'PyBullet'.")
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


#TODO: FIX later (need to compute IK in PyBullet and send to ABB using ROSClient in RRC) #TODO: REALLY FIX INPUTS LATER.
class ABBMimicHandlerCombined(RobotHandlerCombinedBackends):
    
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
        print(f"ABBCombinedBackendHandler: [{robot_name}] Connected to ABB controller via RRC")

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
            print(f"ABBCombinedBackendHandler: [{self.robot_name}] closed")

    ####################################################################################################
    # Execution
    ####################################################################################################

    def _get_current_configuration(self):
        robot_joints, _ = self.abb.send_and_wait(rrc.GetJoints())
        print(f"[{self.robot_name}] Current joints from controller: {robot_joints}")
        return self.pyb_robot.zero_configuration()

    def _send_to_configuration(self, config: Configuration):
        print(f"ABBCombinedBackendHandler: [{self.robot_name}] Executing motion: {config.joint_values}")
        
        # Convert radians to degrees for ABB
        joint_values_deg = [v * 180.0 / 3.1415926 for v in config.joint_values]
        rax = rrc.RobotJoints(*joint_values_deg)
        ext_axes = [0.0] * 6  # TODO: Placeholder for external axes

        result = self.abb.send_and_wait(rrc.MoveToJoints(rax, ext_axes, self.speed, rrc.Zone.FINE))
        print(f"[{self.robot_name}] Motion complete: {result}")

    def _send_to_target(self, frame: Frame):
        raise NotImplementedError("ABBCombinedBackendHandler : WIP - Target frame motion not implemented yet.")
        print(f"URCombinedBackendHandler: [{self.robot_name}] (Sim) Executing UR motion to target frame: {frame}")

