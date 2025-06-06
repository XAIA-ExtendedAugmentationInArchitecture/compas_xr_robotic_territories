#=============================================================================
# Copyright © 2025 NaturalPoint, Inc. All Rights Reserved.
# 
# THIS SOFTWARE IS GOVERNED BY THE OPTITRACK PLUGINS EULA AVAILABLE AT https://www.optitrack.com/about/legal/eula.html 
# AND/OR FOR DOWNLOAD WITH THE APPLICABLE SOFTWARE FILE(S) (“PLUGINS EULA”). BY DOWNLOADING, INSTALLING, ACTIVATING 
# AND/OR OTHERWISE USING THE SOFTWARE, YOU ARE AGREEING THAT YOU HAVE READ, AND THAT YOU AGREE TO COMPLY WITH AND ARE
# BOUND BY, THE PLUGINS EULA AND ALL APPLICABLE LAWS AND REGULATIONS. IF YOU DO NOT AGREE TO BE BOUND BY THE PLUGINS
# EULA, THEN YOU MAY NOT DOWNLOAD, INSTALL, ACTIVATE OR OTHERWISE USE THE SOFTWARE AND YOU MUST PROMPTLY DELETE OR
# RETURN IT. IF YOU ARE DOWNLOADING, INSTALLING, ACTIVATING AND/OR OTHERWISE USING THE SOFTWARE ON BEHALF OF AN ENTITY,
# THEN BY DOING SO YOU REPRESENT AND WARRANT THAT YOU HAVE THE APPROPRIATE AUTHORITY TO ACCEPT THE PLUGINS EULA ON
# BEHALF OF SUCH ENTITY. See license file in root directory for additional governing terms and information.
#=============================================================================


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish a connection (by creating a NatNetClient),
# and receive data via a NatNet connection and decode it using the NatNetClient library.

import sys
import time
from opti_track_dependencies.samples.NatNetClient import NatNetClient
import opti_track_dependencies.samples.DataDescriptions
import opti_track_dependencies.samples.MoCapData

import time
import json
import time
import os
import datetime
import math

# imports for motive transofmrations
import numpy as np
from compas.geometry import Point, Quaternion, Frame
from compas.data import json_load, json_dump
from scipy.spatial.transform import Rotation as R
import simpleaudio as sa

#Custom Package Imports
from robots.transformations.robot_transformations import RobotTransformationsFromObserved


#TODO: TESTING : Turn into class? #############################################################################################################################

# Information Storage and Settings
output_by_frame_data = {}
current_rigid_body_locations = {}
output_by_timestamp_data = {}
last_write_time = 0
last_print_time = 0
WRITE_INTERVAL = 2  # seconds
POSITION_THRESHOLD = 0.01 # meters
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
rigid_body_names = {
    "1" : "Origin",
    "2" : "Red01",
    "3" : "Red02",
    "4" : "Red03",
    "5" : "Blue01",
    "6" : "Blue02",
    "7" : "Blue03",
    "8" : "UR20",
    "9" : "UR3Table",
    "10": "ABBTable"
} #TODO: This could be improved.

# Project Configuration Information
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_CONFIG_FP = os.path.join(SCRIPT_DIR, "project_config.json")
PROJECT_CONFIG_DICT = json_load(PROJECT_CONFIG_FP)
SESION_DIR_NAME = "20250603_robot_transformation_testing_2"

# Storage Directories file names and paths
BASE_DIR = os.path.join("recordings", "motive_recordings")
FRAME_RECORDINGS_DIR = "recordings"
TIME_STAMP_DIR = timestamp
RECORD_OUT_FILE_NAME = f"rigid_bodies_by_frame.json"
TIME_STAMP_RECORDINGS_FILE_NAME = f"rigid_bodies_by_timestamp.json"
VARIATION_FILE_OUT_NAME = "current_rigid_body_locations.json"

# Logging Directories Paths
OUTPUT_PATH = os.path.join(
    BASE_DIR,
    SESION_DIR_NAME
    )
RECORDINGS_DIR = os.path.join(
    OUTPUT_PATH,
    FRAME_RECORDINGS_DIR,
    TIME_STAMP_DIR
)

if not os.path.exists(OUTPUT_PATH):
    os.makedirs(OUTPUT_PATH)
if not os.path.exists(RECORDINGS_DIR):
    os.makedirs(RECORDINGS_DIR)

RECORD_OUT_PATH = os.path.join(
    RECORDINGS_DIR,
    RECORD_OUT_FILE_NAME
)
RIGID_BODIES_CURRENT_FILEPATH = os.path.join(
    OUTPUT_PATH,
    VARIATION_FILE_OUT_NAME
)
RIGID_BODIES_BY_TIMESTAMP_FILEPATH = os.path.join(
    RECORDINGS_DIR,
    TIME_STAMP_RECORDINGS_FILE_NAME
)

#Robot Transformation and Localization information
robot_transformer = None


def receive_rigid_body_frame_TEST(new_id, position, rotation):
    global last_print_time, output_by_frame_data, last_write_time, rigid_body_names, current_rigid_body_locations

    current_time = time.time()
    model_name = rigid_body_names.get(str(new_id), f"Unknown_{new_id}")

    frame_info = {
        "timestamp": current_time,
        "id": new_id,
        "position": {
            "x": position[0],
            "y": position[1],
            "z": position[2]
        },
        "rotation": {
            "x": rotation[0],
            "y": rotation[1],
            "z": rotation[2],
            "w": rotation[3]
        }
    }

    # Initialize list for this model name if needed
    if model_name not in output_by_frame_data:
        output_by_frame_data[model_name] = []

    # Append this frame's info
    output_by_frame_data[model_name].append(frame_info)

    update_rigid_body_location_if_changed(model_name, frame_info["position"], frame_info["rotation"])

    # Write to file if interval has passed
    if current_time - last_write_time > WRITE_INTERVAL:
        try:
            with open(RECORD_OUT_PATH, "w") as f:
                json.dump(output_by_frame_data, f, indent=2)
            print(f"[{time.strftime('%H:%M:%S')}] Wrote data to {OUTPUT_PATH}")
        except Exception as e:
            print(f"Error writing file: {e}")
        last_write_time = current_time

def update_rigid_body_location_if_changed(model_name, current_position, current_rotation, play_sound=False):
    global current_rigid_body_locations

    previous = current_rigid_body_locations.get(model_name)
    changed = False

    if previous is None:
        changed = True
    else:
        pos_changed = position_changed(current_position, previous["position"], POSITION_THRESHOLD)
        rot_changed = rotation_changed(current_rotation, previous["rotation"], angle_threshold_deg=2.0)
        changed = pos_changed or rot_changed

    if changed:
        current_rigid_body_locations[model_name] = {
            "position": current_position,
            "rotation": current_rotation
        }

        #TODO: Testing and needs to be improved....
        if (model_name == "UR20") or (model_name == "UR3Table") or (model_name == "ABBTable"):
            print(f"[{time.strftime('%H:%M:%S')}] Robot Position Changed: Robot {model_name} : current pos : {current_position}, current rotation : {current_rotation}")
            if play_sound:
                try:
                    sounds_dict = PROJECT_CONFIG_DICT.get("sounds", None)
                    robot_sound_path = sounds_dict.get("robot_moved", None)
                    if not robot_sound_path:
                        print("No sound path found for 'robot_moved'. Using default sound.")
                        pass
                    wave_obj = sa.WaveObject.from_wave_file(robot_sound_path)
                    play_obj = wave_obj.play()
                    # play_obj.wait_done() #todo: don't know if I need this see if it plays to the end without blocking.
                except Exception as e:
                    print(f"Error playing sound: {e}")

        else: #TODO: now this will also play a sound for the origin.... but we should create a seperation if we intende to make the origin mobile.
            if play_sound:
                try:
                    sounds_dict = PROJECT_CONFIG_DICT.get("sounds", None)
                    robot_sound_path = sounds_dict.get("object_moved", None)
                    if not robot_sound_path:
                        print("No sound path found for 'robot_moved'. Using default sound.")
                        pass
                    wave_obj = sa.WaveObject.from_wave_file(robot_sound_path)
                    play_obj = wave_obj.play()
                    # play_obj.wait_done() #todo: don't know if I need this see if it plays to the end without blocking.
                except Exception as e:
                    print(f"Error playing sound: {e}")

        # Save timestamped change
        if model_name not in output_by_timestamp_data:
            output_by_timestamp_data[model_name] = []

        output_by_timestamp_data[model_name].append({
            "timestamp": timestamp,
            "position": current_position,
            "rotation": current_rotation
        })

        try:
            with open(RIGID_BODIES_CURRENT_FILEPATH, "w") as f:
                json.dump(current_rigid_body_locations, f, indent=2)

            with open(RIGID_BODIES_BY_TIMESTAMP_FILEPATH, "w") as f:
                json.dump(output_by_timestamp_data, f, indent=2)

            print(f"[{time.strftime('%H:%M:%S')}] Updated {model_name} location/rotation.")
        except Exception as e:
            print(f"Error writing variation file: {e}")

def position_changed(pos1, pos2, threshold=0.01):
    dx = pos1["x"] - pos2["x"]
    dy = pos1["y"] - pos2["y"]
    dz = pos1["z"] - pos2["z"]
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    return distance > threshold

def rotation_changed(rot1, rot2, angle_threshold_deg=1.0):
    dot = (
        rot1["x"] * rot2["x"] +
        rot1["y"] * rot2["y"] +
        rot1["z"] * rot2["z"] +
        rot1["w"] * rot2["w"]
    )
    dot = max(min(dot, 1.0), -1.0)
    
    angle_rad = 2 * math.acos(abs(dot))
    angle_deg = math.degrees(angle_rad)
    return angle_deg > angle_threshold_deg

#TODO : BELOW IS THE FUNCTIONS FOR CONVERTING TO RHINO FRAME FROM MOTIVE OUTPUT #######################################################################################################################################

# -----------------------------------------------------------------------------------------
# Coordinate Transform: Motive (Z forward, Y up, X left) → Rhino (Z up, X right, Y forward)
# -----------------------------------------------------------------------------------------

MOTIVE_ZFWD_TO_RHINO_ZUP_4x4 = np.array([
    [-1, 0,  0, 0],  # X (left) → -X (right)
    [ 0, 0,  1, 0],  # Z (fwd)  → Y (fwd)
    [ 0, 1,  0, 0],  # Y (up)   → Z (up)
    [ 0, 0,  0, 1]
])

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

def create_rhino_frame_from_motive(point_motive, quat_motive):
    point_rhino = transform_point(point_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
    quat_rhino = transform_quaternion(quat_motive, MOTIVE_ZFWD_TO_RHINO_ZUP_4x4)
    frame_motive = Frame.from_quaternion(quat_motive, point_motive)
    frame_rhino = Frame.from_quaternion(quat_rhino, point_rhino)
    return frame_motive, frame_rhino

def update_robots_localization(robot_name, point_motive, quat_motive):
    global robot_transformer

    # Convert Motive pose to Rhino frame #TODO: Do not need anything right now for moving the motive frame, but just keeping it for now.
    observed_frame_motive, observed_frame_rhino = create_rhino_frame_from_motive(point_motive, quat_motive)

    robot_transformer.update_robot_transformation(robot_name, observed_frame_rhino)
    print(f"Updated {robot_name} localization in Rhino frame: {observed_frame_rhino}")

#TODO : BELOW DOES NOT WORK VERY WELL YET #######################################################################################################################################

def process_full_mocap_frame(mocap_data):
    global last_write_time, output_by_frame_data, OUTPUT_PATH, client

    timestamp = time.time()

    # Get list of rigid bodies from the dict
    rigid_bodies = mocap_data.get("rigid_bodies", [])

    for rb in rigid_bodies:
        rb_id = rb.get("id")
        pos = rb.get("position", [0.0, 0.0, 0.0])
        rot = rb.get("orientation", [0.0, 0.0, 0.0, 1.0])
        error = rb.get("mean_marker_error", 0.0)
        valid = rb.get("tracking_valid", False)

        # Try to find model name from descriptions
        model_name = "Unknown"
        if hasattr(client, "rigidBodyDescriptions") and client.rigidBodyDescriptions:
            for desc in client.rigidBodyDescriptions:
                if desc.ID == rb_id:
                    model_name = desc.name
                    break

        frame_entry = {
            "timestamp": timestamp,
            "id": rb_id,
            "model_name": model_name,
            "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "rotation": {"x": rot[0], "y": rot[1], "z": rot[2], "w": rot[3]},
            "error": error,
            "tracking_valid": valid,
        }

        print ("Frame Entry:", frame_entry)

        output_by_frame_data.append(frame_entry)

    if time.time() - last_write_time > WRITE_INTERVAL:
        try:
            with open(OUTPUT_PATH, "w") as f:
                json.dump(output_by_frame_data, f, indent=2)
            print(f"[{time.strftime('%H:%M:%S')}] Wrote {len(output_by_frame_data)} frames to file.")
            output_by_frame_data.clear()
            last_write_time = time.time()
        except Exception as e:
            print(f"Write error: {e}")

def receive_new_frame_with_data(data_dict):
    order_list = ["frameNumber", "markerSetCount", "unlabeledMarkersCount", #type: ignore  # noqa F841
                  "rigidBodyCount", "skeletonCount", "labeledMarkerCount",
                  "timecode", "timecodeSub", "timestamp", "isRecording",
                  "trackedModelsChanged", "offset", "mocap_data", "Model Name"]
    dump_args = True
    if dump_args is True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "= "
            if key in data_dict:
                out_string += str(data_dict[key]) + " "
            out_string += "/"
        print(out_string)

def receive_new_frame_with_data(data_dict):
    print(f"Frame #{data_dict.get('frame_number', 'N/A')} at {data_dict.get('timestamp', 'N/A'):.3f}s")

    rigid_bodies = data_dict.get("rigid_bodies", [])
    print(f"  → Found {len(rigid_bodies)} rigid bodies")

    for rb in rigid_bodies:
        print(f"    ID {rb['id']}: Pos={rb['position']}, Rot={rb['orientation']}, "
              f"Error={rb.get('mean_marker_error', 0.0):.4f}, Valid={rb.get('tracking_valid', False)}")

#TODO: TESTING  #######################################################################################################################################

if __name__ == "__main__":

    # Robotic Transofrmations class
    transformations_fp = PROJECT_CONFIG_DICT.get("robot_transformations_fp", None)
    fb_config_fp = PROJECT_CONFIG_DICT.get("firebase_config_fp", None)
    project_name = PROJECT_CONFIG_DICT.get("project_name", None)

    if not transformations_fp or not fb_config_fp or not project_name:
        print("Error: Missing required configuration paths in project_config.json.")
        sys.exit(1)
    robot_transformer = RobotTransformationsFromObserved(transformations_fp, fb_config_fp, project_name)

    #Natnet Streaming. #TODO: could be moved to project_config.json
    optionsDict = {
        "clientAddress": "127.0.0.1",
        "serverAddress": "127.0.0.1",
        "use_multicast": True,
        "stream_type": "d"
    }

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])
    streaming_client.set_print_level(0)
    streaming_client.rigid_body_listener = receive_rigid_body_frame_TEST

    print("Starting NatNet Streaming Client...")
    if not streaming_client.run(optionsDict["stream_type"]):
        print("ERROR: Could not start streaming client.")
        sys.exit(1)

    time.sleep(1)
    if not streaming_client.connected():
        print("ERROR: Connection to Motive failed.")
        streaming_client.shutdown()
        sys.exit(2)

    print("Streaming started.")
    print("Press Ctrl+C to stop...\n")

    try:
        while True:
            time.sleep(0.1)  # Keep the loop alive without busy waiting
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down streaming client...")

    streaming_client.shutdown()
    print("Done.")