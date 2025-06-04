import numpy as np
from compas.geometry import Frame, Point, Vector, Quaternion
from compas.data import json_load, json_dump


#THIS IS THE ORIGIN TEST WE RAN
# MOTIVE_TO_RHINOZUP = np.array([
#     [1, 0, 0],
#     [0, 0, 1],
#     [0, 1, 0]
# ])

# MOTIVE_ZFWD_TO_RHINO_ZUP = np.array([
#     [0, 0, 1],  # X → Z
#     [1, 0, 0],  # Y → X
#     [0, 1, 0]   # Z → Y
# ])

MOTIVE_ZFWD_TO_RHINO_ZUP = np.array([
    [-1, 0,  0],  # X (left) → -X (right)
    [ 0, 0,  1],  # Z (fwd)  → Y (fwd)
    [ 0, 1,  0]   # Y (up)   → Z (up)
])

MOTIVE_XFWD_TO_RHINO_ZUP = np.array([
    [1, 0, 0],  # X stays X
    [0, 0, 1],  # Y → Z
    [0, 1, 0]   # Z → Y
])


def transform_frame(frame: Frame, transform: np.ndarray) -> Frame:
    origin_np = np.array(frame.point)
    xaxis_np = np.array(frame.xaxis)
    yaxis_np = np.array(frame.yaxis)

    origin_new = transform @ origin_np
    xaxis_new = transform @ xaxis_np
    yaxis_new = transform @ yaxis_np

    return Frame(Point(*origin_new), Vector(*xaxis_new), Vector(*yaxis_new))


def create_motive_frame(pos_dict, quaternion_dict) -> Frame:
    origin = Point(pos_dict['x'], pos_dict['y'], pos_dict['z'])
    quaternion = Quaternion(quaternion_dict['w'], quaternion_dict['x'], quaternion_dict['y'], quaternion_dict['z'])
    return Frame.from_quaternion(quaternion=quaternion, point=origin)


if __name__ == "__main__":

    fp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing\recordings\20250603_142657\rigid_bodies_by_timestamp.json"
    data = json_load(fp)

    ur20_data = data["UR20"]
    origin_data = data["Origin"]

    motive_frames = []
    rhino_frames = []

    data_dict = {}
    data_dict["motive_frames"] = []
    data_dict["rhino_frames"] = []

    for i in range(len(ur20_data)):
        position = ur20_data[i]["position"]
        rotation = ur20_data[i]["rotation"]
        timestamp = ur20_data[i]["timestamp"]
        motive_frame = create_motive_frame(position, rotation)
        rhino_frame = transform_frame(motive_frame, MOTIVE_ZFWD_TO_RHINO_ZUP)
        motive_frames.append(motive_frame)
        rhino_frames.append(rhino_frame)
        data_dict["motive_frames"].append(motive_frame)
        data_dict["rhino_frames"].append(rhino_frame)

    origin_frame_motive = create_motive_frame(origin_data[0]["position"], origin_data[0]["rotation"])
    origin_frame_rhino = transform_frame(origin_frame_motive, MOTIVE_ZFWD_TO_RHINO_ZUP)
    data_dict["origin_frame_motive"] = origin_frame_motive
    data_dict["origin_frame_rhino"] = origin_frame_rhino

    outfp = r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out\20250603_robot_transformation_testing\recordings\20250603_142657\frame_transformation_tests.json"
    json_dump(data=data_dict, fp=outfp, pretty=True)