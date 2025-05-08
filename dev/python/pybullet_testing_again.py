import os
import time
from datetime import datetime

import compas_fab
from compas_fab.backends import PyBulletClient
from compas.data import json_load, json_dump

# --- Paths ---
script_dir = os.path.dirname(os.path.abspath(__file__))

urdf_file_path = os.path.normpath(
    os.path.join(script_dir, "../scripts/urdf/ur_description/urdf/ur20.urdf")
)
frames_file_path = os.path.normpath(
    os.path.join(script_dir, "frames_same.json")
)
output_ik_path = os.path.normpath(
    os.path.join(script_dir, "ik_configurations_pybullet_testing__directaccess.json")
)
output_fk_path = os.path.normpath(
    os.path.join(script_dir, "random_config_frames.json")
)
output_config_path = os.path.normpath(
    os.path.join(script_dir, "configuration_data_testing.json")
)

# --- Load target frames ---
frames = json_load(frames_file_path)

with PyBulletClient(connection_type='gui') as client:
    urdf_file = compas_fab.get(urdf_file_path)
    robot = client.load_robot(urdf_file)
    
    print("ClientDir", dir(client))

    # robot_uid = client.get_robot().uid
    robot_uid = client._robot.uid
    # robot_uid = robot.uid
    pb = robot.client._pybullet_client  # shortcut for convenience

    # Get joint indices for non-fixed joints
    joint_ids = [
        i for i in range(pb.getNumJoints(robot_uid))
        if pb.getJointInfo(robot_uid, i)[2] != pb.JOINT_FIXED
    ]

    ik_information = {}
    ik_configurations = []
    random_config_frames = []
    configuration_data = []

    for i, frame in enumerate(frames):
        start_config = robot.zero_configuration() if i == 0 else ik_configurations[-1]

        # Solve IK
        ik_config = robot.inverse_kinematics(
            frame_WCF=frame,
            start_configuration=start_config
        )

        # Visualize IK result in PyBullet
        joint_values = ik_config.values
        for joint_id, joint_value in zip(joint_ids, joint_values):
            pb.resetJointState(robot_uid, joint_id, joint_value)

        client.step_simulation()
        time.sleep(0.5)

        # Save data
        configuration_data.append(ik_config.__data__)
        fk_frame = robot.forward_kinematics(ik_config)
        random_config_frames.append(fk_frame)
        ik_configurations.append(ik_config)

        ik_information[i] = {
            "target_configuration": ik_config,
            "frame": fk_frame,
            "start_configuration": start_config,
            "time": datetime.now().isoformat()
        }

    # --- Save JSON outputs ---
    json_dump(ik_information, output_ik_path, pretty=True)
    json_dump(random_config_frames, output_fk_path, pretty=True)
    json_dump(configuration_data, output_config_path, pretty=True)

    print("IK results saved and visualized.")
