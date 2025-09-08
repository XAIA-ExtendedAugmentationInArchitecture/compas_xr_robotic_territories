import time
from compas.datastructures import Mesh

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import CollisionMesh

import os
import time
from compas.data import json_dump
from compas_fab.backends import PyBulletClient
from compas_robots import Configuration
from compas.geometry import Frame
import compas_fab
import compas_rrc as rrc
from compas_xr.mqtt import RealtimeMimicRequestMessage

import pybullet as pb
from compas_fab.backends import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh

from compas_robots.resources import LocalPackageMeshLoader
from compas.datastructures import Mesh

class SimplePybulletLoadTest:

    def __init__(self, robot_name, urdf_path, srdf_path=None, time_sleep_delay=5):
        self.robot_name = robot_name
        self.urdf_path = os.path.normpath(urdf_path)
        self._time_sleep_delay = time_sleep_delay
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
        time.sleep(self._time_sleep_delay)
        return robot

    def _load_semantics(self):
        if self.srdf_path:
            print("Semantics Loaded")
            semantics = self.client.load_semantics(self.robot, srdf_filename=self.srdf_path)
        else:
            print("Sementics Not loaded")
            semantics = None
        return semantics

if __name__ == "__main__":

    test = SimplePybulletLoadTest("ABB", 
        urdf_path="C:\\Users\\jk6372\\Desktop\\00_princeton_projects\\00_robotic_territories\\00_git\\compas_xr_robotic_territories\\dev\\performance_operations\\robots\\urdf_abbs\\abb_irb4600s_robots\\urdf\\abb_irb4600s.urdf",
        srdf_path="C:\\Users\\jk6372\\Desktop\\00_princeton_projects\\00_robotic_territories\\00_git\\compas_xr_robotic_territories\\dev\\performance_operations\\robots\\urdf_abbs\\abb_irb4600s_moveit_config\\config\\abb_irb4600s.srdf",
        time_sleep_delay=5
    )   