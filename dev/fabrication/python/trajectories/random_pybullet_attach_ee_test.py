# import time

# from compas.datastructures import Mesh
# from compas_robots.resources import LocalPackageMeshLoader

# import compas_fab
# from compas_fab.backends import PyBulletClient
# from compas_fab.robots import AttachedCollisionMesh
# from compas_fab.robots import CollisionMesh


# with PyBulletClient() as client:
#     urdf_filepath = compas_fab.get(r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\urdf\ur20.urdf") #'robot_library/ur5_robot/urdf/robot_description.urdf')
#     # loader = LocalPackageMeshLoader(compas_fab.get('robot_library/ur5_robot'), '')
#     robot = client.load_robot(urdf_filepath)


#     mesh = Mesh.from_stl(compas_fab.get(r"C:\Users\jk6372\Desktop\piab_gripper.stl"))
#     cm = CollisionMesh(mesh, 'tip')
#     acm = AttachedCollisionMesh(cm, 'tool0')
#     client.add_attached_collision_mesh(acm, {'mass': 0.5, 'robot': robot})

#     time.sleep(1)
#     client.step_simulation()
#     time.sleep(1)

#     client.remove_attached_collision_mesh('tip', {'robot': robot})

#     time.sleep(1)


import time

from compas.datastructures import Mesh
from compas_robots.resources import LocalPackageMeshLoader

import compas_fab
from compas_fab.backends import PyBulletClient
from compas_fab.robots import AttachedCollisionMesh
from compas_fab.robots import CollisionMesh
from compas_robots import Configuration


with PyBulletClient() as client:

    # urdf_filepath = compas_fab.get(r'C:\Users\jk6372\Downloads\compas_fab-main\compas_fab-main\src\compas_fab\data\robot_library\ur5_robot\urdf\robot_description.urdf')
    # loader = LocalPackageMeshLoader(compas_fab.get(r'C:\Users\jk6372\Downloads\compas_fab-main\compas_fab-main\src\compas_fab\data\robot_library\ur5_robot'), '')

    urdf_filepath = compas_fab.get(r'C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_20\urdf\ur20.urdf')
    loader = LocalPackageMeshLoader(compas_fab.get(r'C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_20'), '')
    # loader = LocalPackageMeshLoader(compas_fab.get(r'C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\scripts\urdf\ur_description\meshes'), '')
    robot = client.load_robot(urdf_filepath, [loader])

    mesh = Mesh.from_stl(compas_fab.get(r'C:\Users\jk6372\Desktop\piab_gripper.stl'))
    cm = CollisionMesh(mesh, 'tip')
    acm = AttachedCollisionMesh(cm, 'tool0')
    client.add_attached_collision_mesh(acm, {'mass': 0.5, 'robot': robot})
    print ("client", dir(client))
    print ("client_planner", dir(client.planner))
    joint_values = [-0.5753701446854838, -1.3239161042593488, 2.010355607990115, 0.038020901891942904, -1.2224739484420444, 1.2182268325270744]
    joint_types = [ 0, 0, 0, 0, 0, 0]
    joint_names= ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    start_config = Configuration(joint_values, joint_types, joint_names)


    time.sleep(1)
    client.step_simulation()
    client.set_robot_configuration(robot, start_config)
    time.sleep(5)

    client.remove_attached_collision_mesh('tip', {'robot': robot})

    time.sleep(1)