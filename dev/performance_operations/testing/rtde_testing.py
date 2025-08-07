import rtde_receive
import time

ROBOT_IP = "192.168.1.10"

# Connect (handshake is implicit)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

try:
    while True:
        q     = rtde_r.getActualQ()         # current joint angles (rad)
        tcp   = rtde_r.getActualTCPPose()   # [x,y,z,rx,ry,rz]
        speed = rtde_r.getActualTCPSpeed()  # [vx,vy,vz,vrx,vry,vrz]
        print(f"q={q}  TCP={tcp}  speed={speed}")
        time.sleep(0.01)
finally:
    # No explicit pause/disconnect needed for this interface
    pass


# import rtde_receive
# import time
# import math

# ROBOT_IP  = "192.168.1.10"
# THRESHOLD = 1e-3    # adjust to taste (rad/s or m/s)

# # connect (uses default recipe)
# rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

# try:
#     while True:
#         # 1) read the instantaneous TCP velocity
#         speed = rtde_r.getActualTCPSpeed()  # [vx, vy, vz, vrx, vry, vrz]

#         # 2) compute its norm
#         speed_norm = math.sqrt(sum(v*v for v in speed))

#         if speed_norm > THRESHOLD:
#             # robot is still moving — only show this
#             print("Moving")
#         else:
#             # robot has stopped — show pose and joints
#             q    = rtde_r.getActualQ()
#             tcp  = rtde_r.getActualTCPPose()
#             print(f"q={q}  TCP={tcp}  speed={speed}")

#         time.sleep(0.01)  # 100 Hz polling
# finally:
#     # nothing special needed to clean up for rtde_receive
#     pass