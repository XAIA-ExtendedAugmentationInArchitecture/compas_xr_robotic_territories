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
from compas_fab.backends import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh

from compas_robots.resources import LocalPackageMeshLoader
from compas.datastructures import Mesh

class LoadingGripperHandler:

    def __init__(self, robot_name, urdf_path, srdf_path=None, ee_path=None):
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
        self.ee_mesh = self._load_ee_mesh(ee_path, self.client, self.robot)
        self.client.cache_robot(self.robot)

        # self.client.ensure_cached_robot_geometry(self.robot)
        # self.client.planner.add_attached_collision_mesh()
        # self.ee_mesh = self._load_ee_mesh(ee_path, self.client, self.robot)

        self.ik_solutions = []
        self._got_initial_config = False
        print(f"RealtimeMimicPyBulletHandler: [{robot_name}] Handler initialized")

    def _load_robot(self):
        urdf_file = compas_fab.get(self.urdf_path)
        robot = self.client.load_robot(urdf_file)

        # self.client.ensure_robot(robot=robot)
        # self.client.ensure_cached_robot_geometry(robot=robot)
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

    
    def _load_ee_mesh(self, ee_path, client, robot):
        
        mesh = Mesh.from_stl(compas_fab.get(ee_path))
        cm = CollisionMesh(mesh, 'tip')
        acm = AttachedCollisionMesh(cm, 'tool0')
        client.planner.add_attached_collision_mesh(acm, {'mass': 0.5, 'robot': robot})

        time.sleep(5)
        client.step_simulation()
        time.sleep(5)

        # client.remove_attached_collision_mesh('tip', {'robot': robot})


if __name__ == "__main__":

    test = LoadingGripperHandler("UR20", 
                                    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf",
                                    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.srdf",
                                    r"C:\Users\jk6372\Desktop\piab_gripper.stl")