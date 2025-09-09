import os
import time
import tyro

import optax
import pybullet as p
import numpy as np
import random
import jax
import jax.numpy as jnp
import flax.linen as nn
from itertools import product
from flax.training.train_state import TrainState
from pprint import pprint
import wandb
import wandb_osh
from wandb_osh.hooks import TriggerWandbSyncHook

from dataclasses import dataclass

from robots.planning.ur20_pick_place_orient_base_env import UR20PickandPlaceOrientBaseEnv

@dataclass
class Args:
    exp_name: str = os.path.basename(__file__)[: -len(".py")]
    seed: int = 1
    track: bool = False
    wandb_project_name: str = "ur20"
    wandb_entity: str = 'raj19'
    wandb_mode: str = 'online'
    wandb_dir: str = 'logs'
    wandb_group: str = '.'
    capture_video: bool = False
    checkpoint: bool = False

    env_id: str = 'reach'

    # Algorithm specific arguments
    max_ep_len: int = 50
    batch_size: int = 512
    rep_dim: int = 64

# env = UR20PickandPlaceEnv(render=True)
# env = UR20PickandPlaceBaseEnv(render=True)
# env = UR20PickandPlaceOrientBaseEnv(render=True)
# action_dim = 5

def get_pick_and_place_action(current_gripper_pos, current_gripper_yaw, current_cube_pos, current_cube_yaw, current_grab, current_plan, init_cube_pos, init_cube_yaw, target_cube_pos, target_cube_yaw, target_quadrant):

    if current_plan['reach_obj_xyz'] <= 1:
        init_cube_top_pos = init_cube_pos + np.array([0, 0, 0.15 + 0.13])
        
        correct_dir = (init_cube_top_pos - current_gripper_pos) / np.linalg.norm(init_cube_top_pos - current_gripper_pos)
        action = np.clip(correct_dir * 5, -1.0, 1.0)  

        if current_grab:
            current_plan['reach_obj_xyz'] += 1

        arm_objyaw_dist = np.abs( init_cube_yaw - current_gripper_yaw )

        action = np.concatenate( [action, np.zeros((1,)), np.ones((1,))] )
        action[-2] = np.clip( (init_cube_yaw - current_gripper_yaw) / arm_objyaw_dist, -1.0, 1.0)

    elif current_plan['reach_carry_height'] <= 1:
         action = np.array( [0, 0, 1, 0, 1] )
         
         if current_gripper_pos[2] >= 0.53:
            current_plan['reach_carry_height'] += 1

    elif current_plan['reach_carry_xy_orient'] <= 1:
        correct_dir = (target_cube_pos[:2] - current_gripper_pos[:2]) / np.linalg.norm(target_cube_pos[:2] - current_gripper_pos[:2])
        action = np.clip(correct_dir * 2, -1.0, 1.0)

        if current_plan['shortest_obj_yaw'] is None:
            symmetric_yaws = np.array([i * np.pi / 2 + target_cube_yaw for i in range(-2, 3)])

            d = np.argmin(np.abs(current_cube_yaw - symmetric_yaws))
            shortest_target_yaw = symmetric_yaws[d]

            current_plan['shortest_obj_yaw'] = shortest_target_yaw

        arm_objyaw_dist = np.abs( current_plan['shortest_obj_yaw'] - current_gripper_yaw )      
            
        action = np.concatenate( [action, np.zeros((1,)), np.zeros((1,)), np.ones((1,))] )
        action[-2] = np.clip( (current_plan['shortest_obj_yaw'] - current_gripper_yaw) / arm_objyaw_dist, -1.0, 1.0)

        if  np.linalg.norm(target_cube_pos[:2] - current_gripper_pos[:2]) < 0.05 and arm_objyaw_dist < 0.05:
            current_plan['reach_carry_xy_orient'] += 1

    elif current_plan['reach_target'] <= 1:

        target_top_pos = target_cube_pos + np.array([0, 0, 0.15 + 0.13])
        action = np.clip(5*(target_top_pos - current_gripper_pos), -1.0, 1.0)
        

        if  np.linalg.norm(target_top_pos - current_gripper_pos) < 0.01:
            current_plan['reach_target'] += 1

        action = np.concatenate( [action, np.zeros((1,)), np.ones((1,))] )
        
    else:
        action = np.array( [0, 0, 0, 0, -1] )

    return action

def plan_init():
    current_plan = {
        'go_to_quadrant':0,
        'reach_obj_xyz':0,
        'reach_carry_height':0,
        'reach_carry_xy_orient':0,
        'shortest_obj_yaw': None,
        'reach_target':0,
        'complete':0,
    }
    return current_plan

class ScriptedPolicy:

    def __init__(self, render=True):
        self.env = UR20PickandPlaceOrientBaseEnv(render=render)
        self.action_dim = 5

    def pick_and_place(self, init_cube_pos=None, init_cube_quat=None, target_cube_pos=None, target_cube_quat=None):
        obs, info = self.env.reset(init_cube_pos, init_cube_quat, target_cube_pos, target_cube_quat)
        done = False
        current_plan = plan_init()

        init_cube_pos = info['blocks_pos']
        init_cube_yaw = info['blocks_yaw']
        target_cube_pos = info['target_position']
        target_cube_yaw = info['target_yaw']
        target_quadrant = self.env.episode_target_quadrant

        trajectory_joint_angles, trajectory_eef_pos, trajectory_eef_quat  = [], [], []

        #TODO: need to know when isGrabbing (at least index of configuration should work)

        while not done:
            action = get_pick_and_place_action(obs[:3], info['current_gripper_yaw'], info['blocks_pos'], info['blocks_yaw'], self.env.grabbing, current_plan, init_cube_pos, init_cube_yaw, target_cube_pos, target_cube_yaw, target_quadrant)    
            next_obs, reward, termination, truncation, info = self.env.step(action, base_joint_ctrl=0.0)

            trajectory_joint_angles.append( info['joint_poses'] )
            trajectory_eef_pos.append( info['current_position'] ) 
            trajectory_eef_quat.append( info['current_oreintation'] )

            done = termination or truncation
            obs = next_obs
            
            time.sleep(0.01)

        return trajectory_joint_angles, trajectory_eef_pos, trajectory_eef_quat


# sp = ScriptedPolicy()

# random_initial_posyaw = np.random.uniform(sp.env._box_sample_min, sp.env._box_sample_max)   
# random_initial_pos = random_initial_posyaw[:3]
# random_initial_yaw = random_initial_posyaw[-1]

# random_initial_target_posyaw = np.random.uniform(low = sp.env._sample_min, high=sp.env._sample_max)
# random_initial_target_pos = random_initial_target_posyaw[:3]
# random_initial_target_yaw = random_initial_target_posyaw[-1]

# trajectory_joint_angles, trajectory_eef_pos, trajectory_eef_quat = sp.pick_and_place(
#                                                             random_initial_pos, 
#                                                             random_initial_yaw,
#                                                             random_initial_target_pos,
#                                                             random_initial_target_yaw,
#                                                         )

# print( trajectory_joint_angles, trajectory_eef_pos, trajectory_eef_quat )