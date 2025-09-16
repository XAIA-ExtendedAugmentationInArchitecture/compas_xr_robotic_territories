import time
from itertools import product

import numpy as np
import pybullet as p
import pybullet_data

# URDF_PATH = "urdf/ur_20_attached_ee_attached_base/urdf/ur20_ee_ab.urdf"
URDF_PATH = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\performance_operations\robots\planning\ur_20_attached_ee_attached_base\urdf\ur20_ee_ab.urdf"
GRIPPER_LINK = "piab_gripper"

import numpy as np
import pybullet as p
import pybullet_data
import time
from typing import Tuple, Dict, Any

class UR20PickandPlaceOrientBaseEnv:
    def __init__(self, reward_type: int = 1, render: bool = True):
        """
        Initialize the UR20 robot environment.
        
        Args:
            render (bool): Whether to render the environment
        """
        self.render_env = render
        self.urdf_path = URDF_PATH

        self.reward_type = reward_type
        
        # Action and observation spaces
        self.action_space_low = np.array([-1.0, -1.0, -1.0, -1.0, -1.0])
        self.action_space_high = np.array([1.0, 1.0, 1.0, 1.0, 1.0])

        self._xyz_action_scale = 0.05
        self._yaw_action_scale = 0.1

        self.default_position = [1.9, 0.0, 1.32]
        self.default_orientation = [np.pi, 0, 0]

        self._box_sample_min = np.array([1.4, -0.5, 0.15, 0])
        self._box_sample_max = np.array([1.5, 0.5, 0.15, np.pi/2])

        self._sample_min = np.array([1.4, -0.5, 0.15, 0])
        self._sample_max = np.array([1.5, 0.5, 0.15, np.pi/2])
        
        self.block_ids = []
        self.n_blocks = 1
        self.block_half_len = 0.15
        self.block_half = [self.block_half_len] * 3

        # Initialize environment
        self._setup_simulation()
        self.robot_id, self.joints = self._load_robot()

        # self.joints:
        # {'jointID': 2,
        # 'jointLowerLimit': -6.283185307179586,
        # 'jointMaxForce': 730.0,
        # 'jointMaxVelocity': 2.0943951023931953,
        # 'jointName': 'shoulder_pan_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 6.283185307179586},
        # {'jointID': 3,
        # 'jointLowerLimit': -6.283185307179586,
        # 'jointMaxForce': 730.0,
        # 'jointMaxVelocity': 2.0943951023931953,
        # 'jointName': 'shoulder_lift_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 6.283185307179586},
        # {'jointID': 4,
        # 'jointLowerLimit': -3.141592653589793,
        # 'jointMaxForce': 430.0,
        # 'jointMaxVelocity': 2.6179938779914944,
        # 'jointName': 'elbow_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 3.141592653589793},
        # {'jointID': 5,
        # 'jointLowerLimit': -6.283185307179586,
        # 'jointMaxForce': 100.0,
        # 'jointMaxVelocity': 3.6651914291880923,
        # 'jointName': 'wrist_1_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 6.283185307179586},
        # {'jointID': 6,
        # 'jointLowerLimit': -6.283185307179586,
        # 'jointMaxForce': 100.0,
        # 'jointMaxVelocity': 3.6651914291880923,
        # 'jointName': 'wrist_2_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 6.283185307179586},
        # {'jointID': 7,
        # 'jointLowerLimit': -6.283185307179586,
        # 'jointMaxForce': 100.0,
        # 'jointMaxVelocity': 3.6651914291880923,
        # 'jointName': 'wrist_3_joint',
        # 'jointType': 'REVOLUTE',
        # 'jointUpperLimit': 6.283185307179586}

        self.lower_limits = [j["jointLowerLimit"] for j in self.joints]
        self.upper_limits = [j["jointUpperLimit"] for j in self.joints]

        self.upper_limits[1] = 0.0
        self.lower_limits[2] = 0.0

        # self.upper_limits[-1] = 3.14
        # self.lower_limits[-1] = 0

        self.joint_ranges = [u - l for l, u in zip(self.lower_limits, self.upper_limits)]

        self.tip_idx = self._find_link_index(GRIPPER_LINK)
        self.bottom_tip_idx = self._find_link_index("gripper_tip")

        self.initial_joint_positions = self._get_initial_joint_positions()
        # self.initial_joint_positions = np.array( (-0.1310012058866315, -0.5401946426628275, -2.6671099770742977e-05, -1.0691459053237282, -1.5675774819740553, -1.7017354491100412) )

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        floor_id = p.loadURDF("plane.urdf", [0,0,0], useFixedBase=True)
        p.changeDynamics(floor_id, -1, lateralFriction=1.0, restitution=0.0)


        # Spawn blocks
        self._load_blocks()

        self.grabbing = False
        self.constraint_id = None

        self.episode_target = None
        self.episode_target_quadrant = None
        self.episode_counter = 0
        self.episode_return = 0
        self.episode_success = 0
        self.episode_reached_box = 0
        self.episode_grabbed_box = 0
        self.max_episode_length = 200 + 100*self.n_blocks
        
    
    def _setup_simulation(self) -> None:
        """Setup the PyBullet simulation environment."""
        if self.render_env:
            self.client_id = p.connect(p.GUI)
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

        self._ctrl_dt = 0.02
        self._sim_dt = 0.005
    
    def _load_blocks(self):
        # (re)create collision & visual once (cheap but cleaner)
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.block_half)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.block_half,
                                     rgbaColor=[0.8, 0.2, 0.2, 1])  # red-ish

        for _ in range(self.n_blocks):
            posorn = np.random.uniform(self._box_sample_min, self._box_sample_max)   # or your own spawn region
            pos = posorn[:3]
            yaw = posorn[-1]
            orn = p.getQuaternionFromEuler( np.array( [0, 0, yaw] ) )
            bid = p.createMultiBody(baseMass=0.1,            # non-zero so robot can push/lift
                                    baseCollisionShapeIndex=col_id,
                                    baseVisualShapeIndex=vis_id,
                                    basePosition=pos,
                                    baseOrientation=orn)
            # optional: tweak friction, restitution, etc.
            p.changeDynamics(bid, -1, lateralFriction=1.0, rollingFriction=0.001, restitution=0.0)
            self.block_ids.append(bid)
    
    def _load_robot(self) -> Tuple[int, list]:
        """
        Load the UR20 robot into the simulation.
        
        Returns:
            Tuple[int, list]: Robot ID and list of joint information
        """
        robot_id = p.loadURDF(
            self.urdf_path,
            [0, 0, 0],
            p.getQuaternionFromEuler([0,0,0]),
            useFixedBase=True,
            flags=p.URDF_USE_SELF_COLLISION,
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
        home_position = self.default_position  # A reasonable home position above the base
        home_orientation = p.getQuaternionFromEuler(self.default_orientation)
        
        initial_joint_positions = p.calculateInverseKinematics(
            self.robot_id,
            self.tip_idx,
            home_position,
            home_orientation,
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
            maxNumIterations=100,
            residualThreshold=0.01,
        )
        
        return initial_joint_positions
    
    def _find_link_index(self, link_name: str) -> int:
        for j in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, j)[12].decode() == link_name:
                return j
        raise ValueError(f"Link {link_name} not found")

    def _actuated_joints(self):
        joints = []
        for j in range(p.getNumJoints(self.robot_id)):
            if p.getJointInfo(self.robot_id, j)[2] != p.JOINT_FIXED:
                joints.append(j)
        return joints
    
    def _get_end_effector_state(self):
        s = p.getLinkState(self.robot_id, self.tip_idx)
        return np.array(s[0]), np.array(s[1])
        
    def _reset_joint_positions(self) -> None:
        
        """Reset all robot joints to their initial positions."""
        for i, jd in enumerate(self.joints):
            q = self.initial_joint_positions[i]
            p.resetJointState(self.robot_id, jd["jointID"], q)

            # overwrite the previous motor command
            p.setJointMotorControl2(self.robot_id,
                                    jd["jointID"],
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=q,
                                    force=jd["jointMaxForce"])
                
        # Step simulation a few times to stabilize
        for _ in range(2):
            p.stepSimulation()
        
    def _random_target(self, target_pos=None, target_yaw=None) -> np.ndarray:

        if target_pos is not None:
            assert target_yaw is not None
            # assert np.all( target_pos >= self._sample_min[:3] ) and np.all( target_pos <= self._sample_max[:3])
            # assert target_yaw >= self._sample_min[-1] and target_yaw <= self._sample_max[-1] 

            target_quat = p.getQuaternionFromEuler( np.array( [0, 0, target_yaw] ) )
            target = np.concatenate( [target_pos, np.array( [target_yaw] ) ] )

        else:
            target = np.random.uniform(low = self._sample_min, high=self._sample_max)
        
            target_pos = target[:3]
            target_quat = p.getQuaternionFromEuler( np.array( [0, 0, target[3]] ) )

        # if self.render:
        if self.render_env:
            if hasattr(self, "_target_body_id") and self._target_body_id is not None:
                p.removeBody(self._target_body_id)
            shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15]*3, rgbaColor=[0, 255, 0, 100],)
            self._target_body_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape_id, basePosition=target_pos, baseOrientation=target_quat)

        return target

    def _get_blocks_state(self):
        pos_arr = []
        orn_arr = []
        for bid in self.block_ids:
            pos, orn = p.getBasePositionAndOrientation(bid)
            pos_arr.append(pos), orn_arr.append(orn)
        return np.concatenate(pos_arr, dtype=np.float32), np.concatenate(orn_arr, dtype=np.float32)
        
    def _reset_block_positions(self, block_position=None, block_yaw=None):

        # for bid in self.block_ids:
        assert len(self.block_ids) == 1
        

        for bid in self.block_ids:
            if block_position is not None:
                # assert np.all( block_position >= self._box_sample_min[:3] ) and np.all( block_position <= self._box_sample_max[:3])
                # assert block_yaw >= self._box_sample_min[-1] and block_yaw <= self._box_sample_max[-1] 

                pos = block_position
                orn = p.getQuaternionFromEuler( np.array( [0, 0, block_yaw] ) )
            else:
                posorn = np.random.uniform(self._box_sample_min, self._box_sample_max)   # or your own spawn region
                pos = posorn[:3]
                orn = p.getQuaternionFromEuler( np.array( [0, 0, posorn[-1]] ) )

            p.resetBasePositionAndOrientation(bid, pos, orn)
            p.resetBaseVelocity(bid, [0, 0, 0], [0, 0, 0])

    def reset(self, block_position=None, block_yaw=None, target_position=None, target_yaw=None) -> np.ndarray:
        self._reset_joint_positions()
        self._reset_block_positions(block_position, block_yaw)
        self.episode_target = self._random_target(target_position, target_yaw)

        x,y,z = self.episode_target[0], self.episode_target[1], self.episode_target[2]
        if x > y and x > -y:
            self.episode_target_quadrant = 1
        elif x < y and y < -x:
            self.episode_target_quadrant = 3
        else:
            raise NotImplementedError
                
        self.episode_counter = 0
        self.episode_return = 0
        self.episode_success = 0
        self.episode_reached_box = 0
        self.episode_grabbed_box = 0
        
        # Get initial state
        initial_pos, initial_ori = self._get_end_effector_state()
        _, _, initial_yaw = p.getEulerFromQuaternion(initial_ori)
        
        blocks_pos, blocks_ori = self._get_blocks_state()
        _, _, blocks_yaw = p.getEulerFromQuaternion(blocks_ori)

        success_easy = np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.1 and np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.08

        contact_points = p.getClosestPoints( bodyA=self.robot_id, linkIndexA=self.bottom_tip_idx,
                                  bodyB=self.block_ids[0],               # -1 == “any other body”
                                  distance=0.01 )
        
        info = {
            "distance_to_target": np.linalg.norm(initial_pos - self.episode_target[:3]),
            "current_position": initial_pos,
            "current_gripper_yaw": initial_yaw,
            "blocks_pos": blocks_pos,
            "blocks_yaw": blocks_yaw,
            "current_oreintation": initial_ori,
            "target_position": self.episode_target[:3],
            "target_yaw": self.episode_target[3],
            "success_easy": success_easy,
            "contact_points": contact_points,
            "grabbed_once": False,
        }
        
        return np.concatenate([initial_pos, initial_ori, blocks_pos, blocks_ori, self.episode_target[:3]]), info

    def _apply_action(self, target_pos, taraget_yaw) -> None:

        target_orientation = np.copy( self.default_orientation )
        target_orientation[-1] = taraget_yaw

        # Calculate inverse kinematics
        joint_poses = p.calculateInverseKinematics(
            self.robot_id,
            self.tip_idx,
            target_pos,
            p.getQuaternionFromEuler(target_orientation),
            lowerLimits=self.lower_limits,
            upperLimits=self.upper_limits,
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

        return joint_poses
    
    def step(self, action: np.ndarray, base_joint_ctrl=0) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Take a step in the environment.
        
        Args:
            action (np.ndarray): Target position [delta-y, delta-z]
            
        Returns:
            Tuple[np.ndarray, float, bool, dict]: Observation, reward, done, info
        """
        self.episode_counter += 1

        contact_points = p.getClosestPoints( bodyA=self.robot_id, linkIndexA=self.bottom_tip_idx,
                                    bodyB=self.block_ids[0],               
                                    distance=0.01 )

        if base_joint_ctrl != 0:
            current = p.getJointState(self.robot_id, 2)[0]
            target  = float(np.clip(current + base_joint_ctrl, self.lower_limits[0], self.upper_limits[0]))
            p.setJointMotorControl2(self.robot_id, 2, p.POSITION_CONTROL,
                            targetPosition=target,
                            force=self.joints[0]["jointMaxForce"],
                            maxVelocity=self.joints[0]["jointMaxVelocity"])
        else:
            # Clip action to valid range
            action = np.clip(action, self.action_space_low, self.action_space_high)
            
            # Get new state
            prev_pos, prev_ori = self._get_end_effector_state()
            _, _, prev_yaw = p.getEulerFromQuaternion(prev_ori)
            
            target_pos = prev_pos + action[:3] * self._xyz_action_scale
            taraget_yaw = np.clip( prev_yaw + action[3] * self._yaw_action_scale, a_max=2*np.pi, a_min=-2*np.pi )

            if len(contact_points) != 0 and action[4] > 0 and not self.grabbing:
                
                blocks_pos, blocks_ori = self._get_blocks_state()

                poc_on_block = contact_points[0][6]
                
                if np.linalg.norm( poc_on_block - (blocks_pos + np.array([0,0,0.15])) ) < 0.02:

                    tip_pos, tip_orn = p.getLinkState(self.robot_id, self.bottom_tip_idx, computeForwardKinematics=True)[:2]

                    p.resetBasePositionAndOrientation(self.block_ids[0], tip_pos - np.array([0,0,0.14]), tip_orn)

                    self.constraint_id = p.createConstraint(
                        parentBodyUniqueId = self.robot_id,
                        parentLinkIndex    = self.bottom_tip_idx,
                        childBodyUniqueId  = self.block_ids[0],
                        childLinkIndex     = -1,
                        jointType          = p.JOINT_FIXED,
                        jointAxis          = [0, 0, 0],
                        parentFramePosition = [0, 0, 0],
                        childFramePosition  = [0, 0, 0],
                        parentFrameOrientation = [0, 0, 0, 1],
                        childFrameOrientation  = [0, 0, 0, 1],
                    )
                    
                    self.grabbing = True

            elif action[4] < 0:
                self.grabbing = False
                if self.constraint_id is not None:
                    p.removeConstraint(self.constraint_id)
                    self.constraint_id = None

            # Apply action
            joint_poses = self._apply_action(target_pos, taraget_yaw)
        
        # Step simulation
        for _ in range(int( self._ctrl_dt // self._sim_dt )):
            p.stepSimulation()
        
        # Get new state
        current_pos, current_ori = self._get_end_effector_state()
        _, _, current_yaw = p.getEulerFromQuaternion(current_ori)

        blocks_pos, blocks_ori = self._get_blocks_state()
        _, _, blocks_yaw = p.getEulerFromQuaternion(blocks_ori)
                
        # Check if we're close enough to target
        termination = False

        success = np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.05 and np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.08
        success_easy = np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.1 and np.linalg.norm(blocks_pos - self.episode_target[:3]) < 0.08
        
        truncation = self.episode_counter >= self.max_episode_length
        
        if self.reward_type == 0:
            pos_err = np.linalg.norm( current_pos - (blocks_pos + np.array([0,0,0.15 + 0.13]) ) )
            target_err = np.linalg.norm( blocks_pos -  self.episode_target[:3] )
            
            reward = ( 1 - np.tanh(5 * (pos_err)) ) * (1 - int(self.grabbing) ) \
            + int(self.grabbing) \
            + 2 * ( 1 - np.tanh(5 * (target_err)) ) * int(self.grabbing)
            
        elif self.reward_type == 1:
            reward = - np.linalg.norm( blocks_pos -  self.episode_target[:3] )
        elif self.reward_type == 2:
            reward = - np.linalg.norm( current_pos - (blocks_pos + np.array([0,0,0.15 + 0.13]) ) ) \
                        - 10 * np.linalg.norm( blocks_pos -  self.episode_target[:3] )
        elif self.reward_type == 3:
            reward = - np.linalg.norm( current_pos - (blocks_pos + np.array([0,0,0.15 + 0.13]) ) ) \
                        - 10 * np.linalg.norm( blocks_pos -  self.episode_target[:3] ) \
                        + int(self.grabbing)
        else:
            raise NotImplementedError

        self.episode_return += reward
        self.episode_success += success
        self.episode_grabbed_box += int(self.grabbing)
        self.episode_reached_box += int(len(contact_points) != 0)

        info = {
            # "distance_to_target": np.linalg.norm(blocks_pos - target_pos),
            "current_position": current_pos,
            "current_oreintation": current_ori,
            "current_gripper_yaw": current_yaw,
            "blocks_pos": blocks_pos,
            "blocks_yaw": blocks_yaw,
            "target_position": self.episode_target[:3],
            "target_yaw": self.episode_target[3],
            "success_easy": success_easy,
            "contact_points": contact_points,
            "joint_poses": np.array([p.getJointState(self.robot_id, j["jointID"])[0] for j in self.joints]),
            "is_grabbing": self.grabbing
        }

        if (termination or truncation):
            self.grabbing = False
            if self.constraint_id is not None:
                p.removeConstraint(self.constraint_id)
                self.constraint_id = None
        
        return np.concatenate([current_pos, current_ori, blocks_pos, blocks_ori, self.episode_target[:3]]), reward, termination, truncation, info

    def get_joint_name_angle_dict(self):
        return {
            j["jointName"]: p.getJointState(self.robot_id, j["jointID"])[0]
            for j in self.joints
        }


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

    def get_frame(self, width=640, height=480):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[2, 0, 1],
            distance=2,
            yaw=80,
            pitch=0,
            roll=0,
            upAxisIndex=2
        )
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=width/height, nearVal=0.1, farVal=100.0)
        img = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
        rgb = np.reshape(img[2], (height, width, 4))[:, :, :3]
        return rgb