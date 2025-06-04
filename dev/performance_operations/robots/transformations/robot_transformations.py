import numpy as np
from compas.geometry import Point, Quaternion, Frame
from compas.data import json_load, json_dump
from scipy.spatial.transform import Rotation as R

#TODO: ##################################################################C

"""
This file will make the correct transformations, but it needs to be supplimented by the 
optitrack tracking file, static robotic configuration files, and additional other information
for the creation of geometry.
"""

 #TODO : EXACT THINGS ###################################################C

  # TODO: 1 Robot Transformations Class (Translations from observed to actual)
  # TODO: 2 Overwrite the transformation in a static json configuration file
  # TODO: 3 Overwrite information on the FB for the AR Robot

#TODO: ##################################################################C

# ------------------------------------------------------------------------
# Coordinate Transform: Motive (Z forward, Y up, X left) → Rhino (Z up)
# ------------------------------------------------------------------------

MOTIVE_ZFWD_TO_RHINO_ZUP_4x4 = np.array([
    [-1, 0,  0, 0],  # X (left) → -X (right)
    [ 0, 0,  1, 0],  # Z (fwd)  → Y (fwd)
    [ 0, 1,  0, 0],  # Y (up)   → Z (up)
    [ 0, 0,  0, 1]
])

# ------------------------------------------------------------------------
# Helper Functions # TODO: These move to the motive file
# ------------------------------------------------------------------------

def get_motive_pose(pos_dict, quat_dict) -> tuple[Point, Quaternion]:
    point = Point(pos_dict["x"], pos_dict["y"], pos_dict["z"])
    quat = Quaternion(quat_dict["w"], quat_dict["x"], quat_dict["y"], quat_dict["z"])
    return point, quat

def transform_point(point: Point, matrix: np.ndarray) -> Point:
    p = np.array([point.x, point.y, point.z, 1.0])
    transformed = matrix @ p
    return Point(*transformed[:3])

def transform_quaternion(q: Quaternion, matrix: np.ndarray) -> Quaternion:
    R_motive = R.from_quat([q.x, q.y, q.z, q.w])
    C = matrix[:3, :3]
    R_rhino = C @ R_motive.as_matrix() @ C.T
    q_rhino = R.from_matrix(R_rhino).as_quat()
    return Quaternion(q_rhino[3], q_rhino[0], q_rhino[1], q_rhino[2])

# ------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------

if __name__ == "__main__":

    # Input/output file paths
    # fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing\recordings\20250603_142657\rigid_bodies_by_timestamp.json"
    fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing_2\recordings\20250603_182112\rigid_bodies_by_timestamp.json"
    outfp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing_2\recordings\20250603_182112\frame_transformation_tests_22.json"

    # Load data
    data = json_load(fp)
    ur20_data = data["UR20"]
    print(len(ur20_data))
    blue01_data = data["Blue01"]
    print(len(blue01_data))
    red01_data = data["Red01"]
    print(len(red01_data))
    origin_data = data["Origin"]

    # Prepare output
    item_data_dict = {
        "motive_frames": [],
        "rhino_frames": [],
    }

    data_dict = {
        "UR20": {
            "motive_frames": [],
            "rhino_frames": [],
        },
        "Blue01": {
            "motive_frames": [],
            "rhino_frames": [],
        },
        "Red01": {
            "motive_frames": [],
            "rhino_frames": [],
        },
    }

    # Convert UR20 poses
    for entry in ur20_data:
        point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

        point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
        quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

        frame_motive = Frame.from_quaternion(quat_motive, point_motive)
        frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

        data_dict["UR20"]["motive_frames"].append(frame_motive)
        data_dict["UR20"]["rhino_frames"].append(frame_rhino)

    # TODO: TEST THIS......

    # Convert Blue01 poses
    for entry in blue01_data:
        point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

        point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
        quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

        frame_motive = Frame.from_quaternion(quat_motive, point_motive)
        frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

        data_dict["Blue01"]["motive_frames"].append(frame_motive)
        data_dict["Blue01"]["rhino_frames"].append(frame_rhino)

    # Convert Red01 poses
    for entry in red01_data:
        point_motive, quat_motive = get_motive_pose(entry["position"], entry["rotation"])

        point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
        quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

        frame_motive = Frame.from_quaternion(quat_motive, point_motive)
        frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)

        data_dict["Red01"]["motive_frames"].append(frame_motive)
        data_dict["Red01"]["rhino_frames"].append(frame_rhino)

    # Convert Origin pose
    origin_pos = origin_data[0]["position"]
    origin_rot = origin_data[0]["rotation"]

    point_motive, quat_motive = get_motive_pose(origin_pos, origin_rot)
    point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
    quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)

    data_dict["origin_frame_motive"] = Frame.from_quaternion(quat_motive, point_motive)
    data_dict["origin_frame_rhino"] = Frame.from_quaternion(quat_rhino, point_rhino)

    # Write transformed output
    json_dump(data=data_dict, fp=outfp, pretty=True)