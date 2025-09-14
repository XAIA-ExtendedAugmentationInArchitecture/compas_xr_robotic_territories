from rtde_control import RTDEControlInterface as RTDEControl
from rtde_io import RTDEIOInterface
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import threading
from compas.geometry import Frame, Transformation, Translation, Vector, Point
from compas_fab.robots.robot import Configuration
from compas import json_load
from compas_fab.robots import to_degrees
import math
from compas_fab.robots import JointTrajectory

def SEND_TO_STATIC_CONFIG_FOR_INFERENCE(speed, accel, ur_c):
    joe_joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    joe_joint_types = [0, 0, 0, 0, 0, 0]
    # joe_start_config_values = [-0.10790457, -0.29844413,  0.06922359, -1.36258329, -1.5687577 , -1.64426868]
    joe_start_config_values = [-0.10790457000000009, -1.26844413, 1.0792235899999998, -2.8625832899999999, -1.5687576999999999, -1.6442686799999999]
    joe_start_configuration = Configuration(joint_values=joe_start_config_values, joint_names=joe_joint_names, joint_types=joe_joint_types)
    move_to_joints_urc(joe_start_configuration, speed, accel, True, ur_c)
    print("SENT TO STATIC CONFIG")

def get_config(ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    robot_joints = ur_r.getActualQ()
    config = Configuration.from_revolute_values(robot_joints)
    return config

def get_config_TEST(ip="127.0.0.1"):
    print (f"RTDE : Getting Configfor First instance. Robot IP : {ip}")
    # ur_r = RTDEReceive(ip)
    # robot_joints = ur_r.getActualQ()
    # config = Configuration.from_revolute_values(robot_joints)
    # return config

def get_tcp_offset(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    tcp = ur_c.getTCPOffset()
    return tcp

def set_tcp_offset(pose, ip = "127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.setTcp(pose)

def normalize_joint_values_to_pi(config):
    for i,v in enumerate(config.joint_values):
        if v>math.pi:
            v-=2*math.pi
        if v<-math.pi:
            v+=2*math.pi
        config.joint_values[i]=v
    return config

def move_to_joints_urc(config, speed, accel, nowait, ur_c):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c.moveJ(config.joint_values, speed, accel, nowait)

def move_to_joints(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool

    ur_c = RTDEControl(ip)
    ur_c.moveJ(config.joint_values, speed, accel, nowait)

def move_to_joints_blend(config, speed, accel, blend, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool

    ur_c = RTDEControl(ip)
    ur_c.moveJ(config.joint_values, speed, accel, nowait)

def move_to_joints_TEST(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    print ("SENDING TO JOINTS", config.joint_values, speed, accel, nowait)
    # ur_c = RTDEControl(ip)
    # ur_c.moveJ(config.joint_values, speed, accel, nowait)

def move_to_joints_urc(config, speed, accel, nowait, ur_c):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c.moveJ(config.joint_values, speed, accel, nowait)

def movel_to_joints(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c = RTDEControl(ip)
    ur_c.moveL_FK(config.joint_values, speed, accel, nowait)

def movel_to_joints_urc(config, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c = RTDEControl(ip)
    ur_c.moveL_FK(config.joint_values, speed, accel, nowait)

def move_to_target(frame, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    pose = frame.point.x, frame.point.y, frame.point.z, *frame.axis_angle_vector
    ur_c = RTDEControl(ip)
    ur_c.moveL(pose ,speed, accel, nowait)
    return pose

def move_to_target_TEST(frame, speed, accel, nowait, ip="127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    pose = frame.point.x, frame.point.y, frame.point.z, *frame.axis_angle_vector
    print("SENDING TO TARGET POSE", pose)

def move_in_z_until_contact(config, speed, accel, nowait, ip):
    ur_r = RTDEReceive(ip)
    ur_c = RTDEControl(ip)
    #tcp_force = ur_r.getActualTCPForce()
    # # ur_c.forceMode(([0, 0, 1, 0, 0, 0], [0.0, 0.0, max_force, 0.0, 0.0, 0.0], [0.01, 0.01, max_speed, 0.01, 0.01, 0.01]))
    # # ur_c.forceModeStop()

    move_to_joints(config, speed, accel, nowait, ur_c)
    ur_c.startContactDetection()
    contact_detected = ur_c.readContactDetection()
    if contact_detected:
        ur_c.stopContactDetection()

    return contact_detected

def pick_and_place_async(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist = 100):
    thread = threading.Thread(target=pick_and_place, args=(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist))
    thread.start()

def pick_and_place(pick_frames, place_frames, speed, accel, ip, vaccum_io, safe_dist = 100):
#move to pick safety plane
    if isinstance(pick_frames,Frame):
        pick_frames = [pick_frames]*len(place_frames)

    for pick, place in zip(pick_frames, place_frames):
        move_to_target(pick.transformed(Translation.from_vector(Vector(0,0,safe_dist))), speed, accel, False, ip = ip)
        #move to pick plane
        move_to_target(pick, speed, accel, False, ip = ip)
        #turn IO on
        set_digital_io(vaccum_io,True,ip=ip)
        #sleep on position to give some time to pick up
        time.sleep(0.5)
        #move to pick safety plane
        move_to_target(pick.transformed(Translation.from_vector(Vector(0,0,safe_dist))), speed, accel, False, ip = ip)
        #move to pre placement frame
        pre_place_frame = place.transformed(Translation.from_vector(Vector(0,0,safe_dist)))
        move_to_target(pre_place_frame, speed, accel, False, ip = ip)
        #move to placement frame
        move_to_target(place, speed, accel, False, ip = ip)
        #turn vaccuum off to place brick
        set_digital_io(vaccum_io,False,ip=ip)
        #sleep robot to make sure it is placed
        time.sleep(0.5)
        #move to post placement frame
        post_place_frame = place.transformed(Translation.from_vector(Vector(0,0,safe_dist)))
        move_to_target(post_place_frame, speed, accel, False, ip = ip)

def create_path(frames, speed, accel, radius):
    # speed rad/s, accel rad/s^2, nowait bool
    path = []
    for f in frames:
        pose = f.point.x/1000, f.point.y/1000, f.point.z/1000, *f.axis_angle_vector
        target = [*pose,speed,accel, radius]
        path.append(target)
    return path

def move_to_path(frames, speed, accel, radius, ip = "127.0.0.1"):
    # speed rad/s, accel rad/s^2, nowait bool
    ur_c = RTDEControl(ip)
    path = create_path(frames, speed, accel, radius)
    ur_c.moveL(path, True)
    return path

def stopL(accel, ip = "127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.stopL(accel)

def get_digital_io(signal, ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    return ur_r.getDigitalOutState(signal)

def set_digital_io(signal, value, ip="127.0.0.1"):
    io = RTDEIOInterface(ip)
    io.setStandardDigitalOut(signal, value)

def set_tool_digital_io(signal, value, ip="127.0.0.1"):
    io = RTDEIOInterface(ip)
    io.setToolDigitalOut(signal, value)

def get_tcp_frame(ip="127.0.0.1"):
    ur_r = RTDEReceive(ip)
    tcp = ur_r.getActualTCPPose()
    frame = Frame.from_axis_angle_vector(tcp[3:], point=tcp[0:3])
    return frame

def start_teach_mode(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.teachMode()

def stop_teach_mode(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_c.endTeachMode()

def measure_frame_from_3_points(ip="127.0.0.1"):
    ur_c = RTDEControl(ip)
    ur_r = RTDEReceive(ip)

    tcp = ur_c.getTCPOffset()
    print("Hello, your current TCP offset is:")
    print(tcp)

    print("The robot is in free drive mode now")
    print()

    print("1. Move the robot tip to the origin of the calibration frame and press Enter")
    ur_c.teachMode()
    input()
    ur_c.endTeachMode()

    frame_origin = ur_r.getActualTCPPose()
    print("Frame origin:")
    print(frame_origin)

    print("2. Move the robot tip to the X-axis of the calibration frame and press Enter")
    ur_c.teachMode()
    input()
    ur_c.endTeachMode()

    frame_point_on_xaxis = ur_r.getActualTCPPose()
    print("Frame on X-axis:")
    print(frame_point_on_xaxis)

    print("3. Move the robot tip to the Y-axis of the calibration frame and press Enter")
    ur_c.teachMode()
    input()
    ur_c.endTeachMode()

    frame_point_on_yaxis = ur_r.getActualTCPPose()
    print("Frame on Y-axis:")
    print(frame_point_on_yaxis)

    frame = Frame.from_points(
        point=frame_origin[0:3], point_xaxis=frame_point_on_xaxis[0:3], point_xyplane=frame_point_on_yaxis[0:3]
    )

    return frame

def send_trajectory_path_test(trajectory, speed, accel, radius, ip):
    ur_c = RTDEControl(ip)
    send_trajectory_path(trajectory, speed, accel, radius,ur_c)
    
def send_trajectory(trajectory_points, speed, accel, ip):

    #Convert points of trajectory to configurations
    for i in range(len(trajectory_points)):

        #Trajectory points
        point = trajectory_points[i]
        print (type(point))
        #Joint values
        # joint = [math.degrees(item) for item in point.values()]

        #move to configuration
        move_to_joints(point, speed, accel, 0 , ip)

def send_trajectory_path(configurations, speed, accel, radius, ur_c):

    print(f"Move trajectory of {len(configurations)} points with speed {speed}, accel {accel} and blend {radius}")
    path = []
   
    for config in configurations:
        path.append(config.joint_values + [speed, accel, radius])

    if len(path):
        ur_c.moveJ(path)

def send_trajectory_path_TEST(configurations, speed, accel, radius, ur_c):

    print(f"Move trajectory of {len(configurations)} points with speed {speed}, accel {accel} and blend {radius}")
    path = []
   
    for config in configurations:
        path.append(config.joint_values + [speed, accel, radius])
    print("PATH TO SEND", path)
    # if len(path):
    #     ur_c.moveJ(path)

def send_trajectory_path_joint_values_only(joint_values, speed, accel, radius, ur_c):

    print(f"Move trajectory of {len(joint_values)} points with speed {speed}, accel {accel} and blend {radius}")
    path = []
   
    for config in joint_values:
        path.append(config + [speed, accel, radius])

    if len(path):
        ur_c.moveJ(path)

def pick_and_place_blocks_trajectories(move_to_pick_trajectory, pick_trajectory, move_trajectory, place_trajectory, speed, accel, radius, ip, vaccum_io):
    
    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    pick_trajectory_reversed = list(reversed(pick_trajectory)) 
    place_trajectory_reversed = list(reversed(place_trajectory)) 
    nowait = True

    try:

        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,True,ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        #Send Exit Trajectory
        #send_trajectory_path(exit_trajectory, speed, accel, radius,ur_c)

        #Send Move to pick_trajectory
        send_trajectory_path(move_to_pick_trajectory, speed, accel, radius,ur_c)
        
        #Send to pick configs list
        send_trajectory_path([pick_trajectory[-2]], speed, accel, 0.0, ur_c)

        #Send to last pick config
        send_trajectory_path([pick_trajectory[-1]], speed/3., accel, 0.0, ur_c)
        
        #Turn off io to grasp new stick
        set_digital_io(vaccum_io, False, ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        # Reverse from pick location to the approach pick plane
        send_trajectory_path([pick_trajectory_reversed[-1]], speed, accel, radius, ur_c)
        
        # Send move trajectory
        send_trajectory_path(move_trajectory, speed*1.5, 0.1, radius*4, ur_c)

        send_trajectory_path([place_trajectory[0]], speed, accel, 0.0, ur_c)

        # Send Place Trajectory
        send_trajectory_path([place_trajectory[-2]], speed, accel, 0.0, ur_c)

        #move_to_joints(place_trajectory[-1], speed, accel, nowait, ip=ur_c)
        send_trajectory_path([place_trajectory[-1]], speed/3., 0.2, 0.0, ur_c)

        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,True,ip=ip)

        send_trajectory_path([place_trajectory[0]], speed, accel, 0.0, ur_c)

        send_trajectory_path(move_to_pick_trajectory, speed, accel, radius,ur_c)
    
    except Exception as e:
        print(e)
        raise

def pick_and_place_jk(pick_trajectory_configs, move_trajectory_configs, place_trajectory_configs, speed, accel, radius, ip, vaccum_io=None):

    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    pick_trajectory_configs_reversed = list(reversed(pick_trajectory_configs)) 
    nowait = True

    try:
        if vaccum_io != None:
            #Turn on io to release stick that is being held
            set_digital_io(vaccum_io,True,ip=ip)
            #sleep on position to give some time for release
            time.sleep(1.0)

        #Send pick trajectoy
        send_trajectory_path(pick_trajectory_configs, speed, accel, radius,ur_c)

        if vaccum_io != None:
            #Turn off io to grasp new stick
            set_digital_io(vaccum_io, False, ip=ip)

        #Send pick_trajectory_configs to reversed
        send_trajectory_path(pick_trajectory_configs_reversed, speed, accel, radius,ur_c)
        
        #Send to move configs list
        send_trajectory_path(move_trajectory_configs, speed, accel, 0.0, ur_c)

        #Send to place trajectory
        send_trajectory_path(place_trajectory_configs, speed/3., accel, 0.0, ur_c)

    except Exception as e:
        print(e)
        raise

def send_pick_and_place_trajectory_RT_inference(trajectory_list, speed, accel, ur_c, radius, ip):

    # ur_c = RTDEControl(ip)
    try:
        complete_trajectory_length = sum([len(t.points) for t in trajectory_list])
        print(f"Sending pick-and-place trajectory with {complete_trajectory_length} points in {len(trajectory_list)} segments")
        for i, trajectory in enumerate(trajectory_list):
            print(f"Sending trajectory {i+1} of {len(trajectory_list)}")

            configs = trajectory.points
            send_trajectory_path(configs, speed, accel, radius, ur_c)

            if i == 2: # After pick trajectory
                time.sleep(0.5)
                set_tool_digital_io(1, True, ip=ip)
                time.sleep(1.0)
            if i == 5: # After place trajectory
                time.sleep(0.5)
                set_tool_digital_io(1, False, ip=ip)
                time.sleep(1.0)
        SEND_TO_STATIC_CONFIG_FOR_INFERENCE(speed, accel, ur_c)

    except Exception as e:
        print(e)
        raise
    print("All trajectories sent successfully.")

#TODO: ADDED RAJJJJJ FUNCTIONS HERE....... #############################################################

def _evenly_spaced_indices(start, end, k):
    """k indices in [start, end] inclusive, evenly spaced."""
    if k <= 0 or end < start:
        return []
    if k == 1:
        return [start + (end - start) // 2]
    span = end - start
    return [start + int(round(t * span / (k - 1))) for t in range(k)]

def _decimate_keep_start_pick_end(configs, pick_idx, max_pts):
    n = len(configs)
    if n == 0:
        return [], None
    pick_idx = max(0, min(int(pick_idx), n - 1))
    if max_pts >= n:
        return configs, pick_idx

    anchors = sorted({0, pick_idx, n - 1})
    remaining = max_pts - len(anchors)
    if remaining <= 0:
        kept_idx = anchors
        return [configs[i] for i in kept_idx], kept_idx.index(pick_idx)

    before_len = max(0, pick_idx - 1)              # 1..pick-1
    after_len  = max(0, (n - 2) - pick_idx)        # pick+1..n-2
    total_between = before_len + after_len

    keep_idx = set(anchors)
    if total_between > 0 and remaining > 0:
        # proportional allocation
        slots_before = int(round(remaining * (before_len / total_between))) if total_between else 0
        slots_before = min(slots_before, before_len)
        slots_after  = remaining - slots_before
        slots_after  = min(slots_after, after_len)
        # fix leftover if any due to min()
        leftover = remaining - (slots_before + slots_after)
        if leftover > 0 and before_len - slots_before > 0:
            add = min(leftover, before_len - slots_before)
            slots_before += add
            leftover -= add
        if leftover > 0 and after_len - slots_after > 0:
            slots_after += min(leftover, after_len - slots_after)

        # sample evenly in each interval (inclusive)
        if slots_before > 0:
            idx_beg = 1
            idx_end = pick_idx - 1
            for i in _evenly_spaced_indices(idx_beg, idx_end, slots_before):
                keep_idx.add(i)
        if slots_after > 0:
            idx_beg = pick_idx + 1
            idx_end = n - 2
            for i in _evenly_spaced_indices(idx_beg, idx_end, slots_after):
                keep_idx.add(i)

    kept_idx = sorted(keep_idx)
    new_pick = kept_idx.index(pick_idx)
    return [configs[i] for i in kept_idx], new_pick

def send_pick_and_place_trajectory_RT_inference_raj(
    trajectory,               # [JointTrajectory] (len=1) or JointTrajectory
    pick_index,               # int index in traj.points where pick happens
    speed, accel, ur_c, radius, ip,
    chunk_size=50,            # 25–75 is a good range
    target_points=100         # decimate to at most this many points
):
    traj = trajectory[0] if isinstance(trajectory, (list, tuple)) else trajectory
    configs = list(traj.points)
    n = len(configs)
    if n == 0:
        print("Empty trajectory.")
        return

    # decimate but keep start/pick/end
    configs, pick_index = _decimate_keep_start_pick_end(configs, pick_index, target_points)
    n = len(configs)
    print(f"Sending chunked pick-and-place: {n} pts after decimation (target={target_points}, chunk={chunk_size})")

    has_pick = isinstance(pick_index, int) and 0 <= pick_index < n
    if not has_pick:
        chunks = [configs[i:i+chunk_size] for i in range(0, n, chunk_size)]
        print(f"  → {len(chunks)} chunks (no pick index)")
        for j, chunk in enumerate(chunks):
            print(f"  chunk {j+1}/{len(chunks)}: {len(chunk)} pts")
            send_trajectory_path(chunk, speed, accel, radius, ur_c)
        SEND_TO_STATIC_CONFIG_FOR_INFERENCE(speed, accel, ur_c)
        print("All chunks sent successfully.")
        return

    pre  = configs[:pick_index+1]
    post = configs[pick_index+1:]

    pre_chunks  = [pre[i:i+chunk_size]   for i in range(0, len(pre),  chunk_size)] if pre  else []
    post_chunks = [post[i:i+chunk_size]  for i in range(0, len(post), chunk_size)] if post else []

    print(f"  pre-pick:  {len(pre)} pts → {len(pre_chunks)} chunks")
    print(f"  post-pick: {len(post)} pts → {len(post_chunks)} chunks")

    # --- send ---
    try:
        for j, chunk in enumerate(pre_chunks):
            print(f"  pre {j+1}/{len(pre_chunks)}: {len(chunk)} pts")
            send_trajectory_path(chunk, speed, accel, radius, ur_c)

        if post_chunks:
            time.sleep(0.5)
            set_tool_digital_io(1, True, ip=ip)   # vacuum ON
            time.sleep(0.5)

        for j, chunk in enumerate(post_chunks):
            print(f"  post {j+1}/{len(post_chunks)}: {len(chunk)} pts")
            send_trajectory_path(chunk, speed, accel, radius, ur_c)

        if post_chunks:
            time.sleep(0.5)
            set_tool_digital_io(1, False, ip=ip)  # vacuum OFF
            time.sleep(0.5)

        SEND_TO_STATIC_CONFIG_FOR_INFERENCE(speed, accel, ur_c)
        print("All chunks sent successfully.")
    except Exception as e:
        print(f"Error while sending chunks: {e}")
        raise

#TODO: ADDED RAJJJJJ FUNCTIONS HERE....... #############################################################

def send_to_single_trajectory(trajectory_configs, speed, accel, radius, nowait, ip, vaccum_io=None):

    ur_c = RTDEControl(ip)

    try:
        if vaccum_io != None:
            #Turn on io to release stick that is being held
            set_digital_io(vaccum_io,True,ip=ip)
            #sleep on position to give some time for release
            time.sleep(1.0)
        #Send pick trajectoy
        send_trajectory_path(trajectory_configs, speed, accel, radius,ur_c)
    
    except Exception as e:
        print(e)
        raise

def send_to_single_trajectory_robotic_territories_TEST(trajectory_configs, speed, accel, radius, ip, io_begining_end_none, vaccum_io=None):
    print(f"URRealtimeMimicHandlerPyB: [{ip}] (Sim) Executing UR trajectory: {trajectory_configs} THIS SHOULD BE A TEST")

def send_to_single_trajectory_robotic_territories(trajectory_configs, speed, accel, radius, ur_c, io_begining_end_none, vaccum_io=None):

    
    time_sleep_delay = 1.5

    #TODO: This means there is no IO connected...
    if io_begining_end_none == 0:
        #Send to trajectoy without any IO controls.
        send_trajectory_path(trajectory_configs, speed, accel, radius,ur_c)
    
    #TODO: This means that the IO needs to be turned on at the begining of the trajectory.
    elif io_begining_end_none == 1:
        try:
            if vaccum_io != None:
                #Turn on io to turn on the vaccum.
                set_tool_digital_io(vaccum_io,True,ip=ip)
                time.sleep(time_sleep_delay)
            send_trajectory_path(trajectory_configs, speed, accel, radius,ur_c)
        
        except Exception as e:
            print(e)
            raise

    #TODO: This means that the IO needs to be turned off at the end of the movement.
    elif io_begining_end_none == 2:
        try:
            send_trajectory_path(trajectory_configs, speed, accel, radius,ur_c)
            if vaccum_io != None:
                #Turn off io to turn off the vaccum.
                set_tool_digital_io(vaccum_io,False,ip=ip)
                time.sleep(time_sleep_delay)
        
        except Exception as e:
            print(e)
            raise

def send_to_single_trajectory_only_joint_values(trajectory_configs, speed, accel, radius, nowait, ip, vaccum_io=None):

    ur_c = RTDEControl(ip)
    nowait = True

    try:
        if vaccum_io != None:
            #Turn on io to release stick that is being held
            set_digital_io(vaccum_io,True,ip=ip)
            #sleep on position to give some time for release
            time.sleep(1.0)
        #Send pick trajectoy
        send_trajectory_path_joint_values_only(trajectory_configs, speed, accel, radius,ur_c)
    
    except Exception as e:
        print(e)
        raise


def exit_pick_and_place_jk(pick_trajectory_configs, move_trajectory_configs, place_trajectory_configs, speed, accel, radius, ip, vaccum_io=None):

    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    pick_trajectory_configs_reversed = list(reversed(pick_trajectory_configs)) 
    nowait = True

    try:
        if vaccum_io != None:
            #Turn on io to release stick that is being held
            set_digital_io(vaccum_io,True,ip=ip)
            #sleep on position to give some time for release
            time.sleep(1.0)

        #Send pick trajectoy
        send_trajectory_path(pick_trajectory_configs, speed, accel, radius,ur_c)

        if vaccum_io != None:
            #Turn off io to grasp new stick
            set_digital_io(vaccum_io, False, ip=ip)

        #Send pick_trajectory_configs to reversed
        send_trajectory_path(pick_trajectory_configs_reversed, speed, accel, radius,ur_c)
        
        #Send to move configs list
        send_trajectory_path(move_trajectory_configs, speed, accel, 0.0, ur_c)

        #Send to place trajectory
        send_trajectory_path(place_trajectory_configs, speed/3., accel, 0.0, ur_c)

    except Exception as e:
        print(e)
        raise


def release_pick_and_place_stick_trajectories(exit_trajectory, move_to_pick_trajectory, pick_trajectory, move_trajectory, place_trajectory, speed, accel, radius, ip, vaccum_io):
    
    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    pick_trajectory_reversed = list(reversed(pick_trajectory))
    nowait = True

    try:

        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,False,ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        ur_c.setPayload(0.1,[0.0,0.0,0.1])

        # Reverse from place location to the approach place plane
        send_trajectory_path([exit_trajectory[-1]], speed/3., 0.2, 0.0, ur_c)

        #Send Move to pick_trajectory
        send_trajectory_path(move_to_pick_trajectory, speed, accel, radius,ur_c)
        
        #Send to pick configs list
        send_trajectory_path([pick_trajectory[-2]], speed, accel, 0.0, ur_c)

        #Send to last pick config
        send_trajectory_path([pick_trajectory[-1]], speed/3., accel, 0.0, ur_c)

        time.sleep(1.0)
        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,True,ip=ip)

        ur_c.setPayload(0.4,[0.0,0.0,0.1])
        #Send to reversed pick configs list
        send_trajectory_path([pick_trajectory_reversed[-1]], speed, accel, 0.0, ur_c)

        # Send move trajectory
        send_trajectory_path(move_trajectory, speed*1.5, accel, radius*1.5, ur_c)

        # Send Place Trajectory
        send_trajectory_path([place_trajectory[-2]], speed, accel, 0.0, ur_c)

        #move_to_joints(place_trajectory[-1], speed, accel, nowait, ip=ur_c)
        send_trajectory_path([place_trajectory[-1]], speed/3., 0.2, 0.0, ur_c)


    except Exception as e:
        print(e)
        raise

def pick_and_place_stick_trajectories(move_to_pick_trajectory, pick_trajectory, move_trajectory, place_trajectory, speed, accel, radius, ip, vaccum_io):
    
    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    pick_trajectory_reversed = list(reversed(pick_trajectory)) 
    nowait = True

    try:

        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,True,ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        #Send Exit Trajectory
        #send_trajectory_path(exit_trajectory, speed, accel, radius,ur_c)

        #Send Move to pick_trajectory
        send_trajectory_path(move_to_pick_trajectory, speed, accel, radius,ur_c)

        #Turn on io to release stick that is being held
        set_digital_io(vaccum_io,False,ip=ip)
        
        #Send to pick configs list

        send_trajectory_path([pick_trajectory[-2]], speed, accel, 0.0, ur_c)

        #Send to last pick config
        send_trajectory_path([pick_trajectory[-1]], speed/3., accel, 0.0, ur_c)

        #Send to reversed pick configs list
        send_trajectory_path([pick_trajectory_reversed[-1]], speed, accel, 0.0, ur_c)

        # Send move trajectory
        send_trajectory_path(move_trajectory, speed, accel, radius*1.5, ur_c)

        # Send Place Trajectory
        send_trajectory_path([place_trajectory[-2]], speed, accel, 0.0, ur_c)

        #move_to_joints(place_trajectory[-1], speed, accel, nowait, ip=ur_c)
        send_trajectory_path([place_trajectory[-1]], speed/3., 0.2, 0.0, ur_c)


    except Exception as e:
        print(e)
        raise

def release_stick_trajectories(place_trajectory, speed, accel, radius, ip, vaccum_io):
    
    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement

    place_trajectory_reversed = list(reversed(place_trajectory)) 
    move_trajectory_reversed = list(reversed(place_trajectory)) 
    nowait = True

    try:
        #Turn off io to open the gripper
        set_digital_io(vaccum_io, False, ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        # Reverse from place location to the approach place plane
        send_trajectory_path(place_trajectory_reversed, speed, accel, 0.0, ur_c)
        
        # Reverse move trajectory
        send_trajectory_path(move_trajectory_reversed, speed, accel, radius, ur_c)
        
   
    except Exception as e:
        print(e)
        raise

def pick_and_place_sticks_configs_trajectories(exit_safe_config, move_to_pick_trajectory, approach_pick_config, pick_config, move_trajectory, place_config, speed, accel, radius, ip, vaccum_io):
    
    ur_c = RTDEControl(ip)
    #reverse pick configs list for safety movement
    
    try:
        #Turn on io to release stick that is being held
        # set_digital_io(vaccum_io,True,ip=ip)
        #sleep on position to give some time for release
        # time.sleep(1.0)

        #move to exit safe config
        move_to_joints_urc(exit_safe_config, speed, accel, True, ur_c)

        #Send Move to pick_trajectory
        send_trajectory_path(move_to_pick_trajectory, speed, accel, radius, ur_c)

        #move to pick config
        move_to_joints_urc(pick_config, speed, accel, True, ur_c)

        #Turn off io to grasp new stick
        set_digital_io(vaccum_io, False,ip=ip)
        #sleep on position to give some time for release
        time.sleep(1.0)

        #move to approach pick config
        move_to_joints_urc(approach_pick_config, speed, accel, True, ur_c)

        # send move to place trajectory
        send_trajectory_path(move_trajectory, speed, accel, radius, ur_c)

        # move to place config
        move_to_joints_urc(place_config,  speed, accel, True, ur_c)
    
    except Exception as e:
        print(e)
        raise








# if __name__ == "__main__":
# #    print(get_config("192.168.10.12"))

#     exit_trajectory= JointTrajectory.from_json(r"X:\mas_t3_working\working_local\week_10\01_json_dump\1.json")
#     move_to_pick_trajectory=  JointTrajectory.from_json(r"X:\mas_t3_working\working_local\week_10\01_json_dump\2.json")
#     pick_trajectory=  JointTrajectory.from_json(r"X:\mas_t3_working\working_local\week_10\01_json_dump\3.json")
#     move_trajectory=  JointTrajectory.from_json(r"X:\mas_t3_working\working_local\week_10\01_json_dump\4.json")
#     place_trajectory=  JointTrajectory.from_json(r"X:\mas_t3_working\working_local\week_10\01_json_dump\5.json")
    
#     speed, accel, radius, ip, vaccum_io = 0.24,0.1, 0.001, "192.168.0.10", 0

#     pick_and_place_sticks(exit_trajectory, move_to_pick_trajectory, pick_trajectory, move_trajectory, place_trajectory, speed, accel, radius, ip, vaccum_io)