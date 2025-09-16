import time
from itertools import product

import numpy as np
import pybullet as p
import pybullet_data

URDF_PATH = "urdf/ur_description/urdf/ur20.urdf"
TIP_IDX = 8

import numpy as np
import pybullet as p
import pybullet_data
import time
from typing import Tuple, Dict, Any

class UR20BlockEnv:
    def __init__(self, render: bool = True):
        """
        Initialize the UR20 robot environment.
        
        Args:
            render (bool): Whether to render the environment
        """
        self.render_env = render
        self.urdf_path = URDF_PATH
        self.tip_link_index = TIP_IDX  # End effector link index
        
        # Action and observation spaces
        self.action_space_low = np.array([-1.0, -1.0, -1.0])
        self.action_space_high = np.array([1.0, 1.0, 1.0])

        self._xyz_action_scale = 0.1

        self.default_orientation = [0, 0, 0]

        self.block_ids = []
        self.n_blocks = 1
        self.block_half_len = 0.08
        self.block_half = [self.block_half_len] * self.n_blocks
        
        points = []
        for y, z in product(np.linspace(-0.5, 0.5, 7), np.linspace(0.25, 1.0, 6)):
            points.append([0.5, y, z])

        self._sample_min = np.array([0.25, -0.5, 0.25])
        self._sample_max = np.array([1.0, 0.5, 1.0])

        self._box_sample_min = np.array([0.25, -0.5, self.block_half_len])
        self._box_sample_max = np.array([1.0, 0.5, self.block_half_len])

        # Initialize environment
        self._setup_simulation()

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        floor_id = p.loadURDF("plane.urdf", [0,0,0], useFixedBase=True)
        p.changeDynamics(floor_id, -1, lateralFriction=1.0, restitution=0.0)

        # Load robot only once during initialization
        self.robot_id, self.joints = self._load_robot()

        # Store initial joint positions for fast reset
        self.initial_joint_positions = self._get_initial_joint_positions()

        # Spawn blocks
        self._load_blocks()
        
        self._ctrl_dt = 0.02
        self._sim_dt = 0.004

        self.targets = np.array(points)
        self.episode_target = None
        self.episode_counter = 0
        self.episode_return = 0
        self.max_episode_length = 50

    def _load_blocks(self):
        # (re)create collision & visual once (cheap but cleaner)
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.block_half)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.block_half,
                                     rgbaColor=[0.8, 0.2, 0.2, 1])  # red-ish

        for _ in range(self.n_blocks):
            pos = np.random.uniform(self._box_sample_min, self._box_sample_max)   # or your own spawn region
            orn = [0, 0, 0, 1]
            bid = p.createMultiBody(baseMass=0.1,            # non-zero so robot can push/lift
                                    baseCollisionShapeIndex=col_id,
                                    baseVisualShapeIndex=vis_id,
                                    basePosition=pos,
                                    baseOrientation=orn)
            # optional: tweak friction, restitution, etc.
            p.changeDynamics(bid, -1, lateralFriction=1.0, rollingFriction=0.001, restitution=0.0)
            self.block_ids.append(bid)

    def _setup_simulation(self) -> None:
        """Setup the PyBullet simulation environment."""
        if self.render_env:
            p.connect(p.GUI)
            p.resetDebugVisualizerCamera(
                cameraYaw=80,
                cameraPitch=0,
                cameraDistance=2,
                cameraTargetPosition=[2, 0, 1]
            )
        else:
            p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

    def _load_robot(self) -> Tuple[int, list]:
        """
        Load the UR20 robot into the simulation.
        
        Returns:
            Tuple[int, list]: Robot ID and list of joint information
        """
        robot_id = p.loadURDF(
            self.urdf_path,
            [0, 0, 0],
            p.getQuaternionFromEuler(self.default_orientation),
            useFixedBase=True,
        )
        
        joint_type = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
        joints = []
        
        for joint_id in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, joint_id)
            data = {
                "jointID": info[0],
                "jointName": info[1].decode("utf-8"),
                "jointType": joint_type[info[2]],
                "jointLowerLimit": info[8],
                "jointUpperLimit": info[9],
                "jointMaxForce": info[10],
                "jointMaxVelocity": info[11],
            }
            if data["jointType"] != "FIXED":
                joints.append(data)
                
        return robot_id, joints
    
    def _get_initial_joint_positions(self) -> list:
        """
        Get the initial joint positions that correspond to home position.
        Uses IK to find joint positions for [0, 0, 0.75] position with self.default_orientation orientation.
        
        Returns:
            list: Initial joint positions
        """
        home_position = [0.5, 0.5, 0.5]  # A reasonable home position above the base
        home_orientation = p.getQuaternionFromEuler(self.default_orientation)
        
        initial_joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            self.tip_link_index,
            home_position,
            home_orientation,
            maxNumIterations=100,
            residualThreshold=0.01,
        )
        
        return initial_joint_positions

    def _get_end_effector_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get the current state of the end effector.
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: Position and orientation of the end effector
        """
        state = p.getLinkState(self.robot_id, self.tip_link_index)
        position = np.array(state[0])
        orientation = np.array(state[1])
        return position, orientation
    
    def _get_blocks_state(self):
        pos_arr = []
        orn_arr = []
        for bid in self.block_ids:
            pos, orn = p.getBasePositionAndOrientation(bid)
            pos_arr.append(pos), orn_arr.append(orn)
        return np.concatenate(pos_arr, dtype=np.float32), np.concatenate(orn_arr, dtype=np.float32)
        
    def _reset_joint_positions(self) -> None:
        """Reset all robot joints to their initial positions."""
        for i, joint_pos in enumerate(self.initial_joint_positions):
            p.resetJointState(
                self.robot_id,
                self.joints[i]["jointID"],
                joint_pos
            )
            
        # Step simulation a few times to stabilize
        for _ in range(10):
            p.stepSimulation()

    def _reset_block_positions(self):
        for bid in self.block_ids:
            pos = np.random.uniform(self._box_sample_min, self._box_sample_max)
            orn = np.array([0, 0, 0, 1])
            p.resetBasePositionAndOrientation(bid, pos, orn)
            p.resetBaseVelocity(bid, [0, 0, 0], [0, 0, 0])

    def reset(self) -> np.ndarray:
        """
        Reset the environment to initial state.
        
        Returns:
            np.ndarray: Initial observation
        """
        # Reset joint positions instead of full simulation reset
        self._reset_joint_positions()
        self._reset_block_positions()

        self.episode_target = self._random_target()
        self.episode_counter = 0
        self.episode_return = 0
        
        # Get initial state
        initial_pos, initial_ori = self._get_end_effector_state()
        blocks_pos, blocks_ori = self._get_blocks_state()
        
        return np.concatenate([initial_pos, initial_ori, blocks_pos, blocks_ori]), self.episode_target, {"current_oreintation": initial_ori}

    def _apply_action(self, target_pos) -> None:
        # Calculate inverse kinematics
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.tip_link_index,
            target_pos,
            p.getQuaternionFromEuler(self.default_orientation),
            maxNumIterations=100,
            residualThreshold=0.01,
        )
        
        # Apply joint positions
        for joint_id in range(6):
            p.setJointMotorControl2(
                self.robot_id,
                self.joints[joint_id]["jointID"],
                p.POSITION_CONTROL,
                targetPosition=joint_poses[joint_id],
            )

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Take a step in the environment.
        
        Args:
            action (np.ndarray): Target position [delta-y, delta-z]
            
        Returns:
            Tuple[np.ndarray, float, bool, dict]: Observation, reward, done, info
        """
        self.episode_counter += 1

        # Clip action to valid range
        action = np.clip(action, self.action_space_low, self.action_space_high)
        
        # Get new state
        prev_pos, prev_ori = self._get_end_effector_state()
        target_pos = prev_pos + action * self._xyz_action_scale

        # Apply action
        self._apply_action(target_pos)
        
        # Step simulation
        for _ in range(int( self._ctrl_dt // self._sim_dt )):
            p.stepSimulation()
        
        # Get new state
        current_pos, current_ori = self._get_end_effector_state()
        blocks_pos, blocks_ori = self._get_blocks_state()

        # Check if we're close enough to target
        termination = False

        success = np.linalg.norm(current_pos - self.episode_target) < 0.05
        success_easy = np.linalg.norm(current_pos - self.episode_target) < 0.1
        
        truncation = self.episode_counter >= self.max_episode_length
        
        self.episode_return += int(success)

        info = {
            "distance_to_target": np.linalg.norm(current_pos - target_pos),
            "current_position": current_pos,
            "current_oreintation": current_ori,
            "target_position": target_pos,
            "success_easy": success_easy,
        }
        
        return np.concatenate([current_pos, current_ori, blocks_pos, blocks_ori]), int(success), termination, truncation, info
    
    def _random_target(self) -> np.ndarray:
        # target = self.targets[np.random.randint(len(self.targets))]
        target = np.random.uniform(low = self._sample_min, high=self._sample_max)

        # if self.render:
        if self.render_env:
            if hasattr(self, "_target_body_id") and self._target_body_id is not None:
                p.removeBody(self._target_body_id)
            shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02]*3, rgbaColor=[0, 255, 0, 255],)
            self._target_body_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape_id, basePosition=target)

        return target

    def close(self) -> None:
        """Close the environment and disconnect from PyBullet."""
        p.disconnect()
    
    def render(self, mode='human') -> None:
        """
        Render the environment.
        Note: In this case, rendering is handled by PyBullet GUI.
        """
        if self.render_env:
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    
def print_joint_info(robot_id):
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}")