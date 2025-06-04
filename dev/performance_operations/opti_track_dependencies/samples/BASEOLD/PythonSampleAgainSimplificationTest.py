import time
import json
from NatNetClient import NatNetClient

import os
from datetime import datetime

# Globals
output_data = []
last_write_time = 0
WRITE_INTERVAL = 2  

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_FILE_NAME = f"rigid_bodies_{timestamp}.json"
OUTPUT_PATH = os.path.join(
    r"C:\Users\jk6372\Desktop\00_princeton_projects\00_robotic_territories\00_git\compas_xr_robotic_territories\dev\nat_net_lib_test\samples\out",
    OUT_FILE_NAME
)
# This will be filled once the client connects
rigid_body_names = {}

# Rigid body callback
def receive_rigid_body_frame(id, position, rotation):
    global last_write_time, output_data, rigid_body_names

    now = time.time()

    model_name = rigid_body_names.get(id, "Unknown")

    rb_info = {
        "timestamp": now,
        "id": id,
        "model_name": model_name,
        "position": {"x": position[0], "y": position[1], "z": position[2]},
        "rotation": {"x": rotation[0], "y": rotation[1], "z": rotation[2], "w": rotation[3]},
    }

    output_data.append(rb_info)

    if now - last_write_time > WRITE_INTERVAL:
        with open(OUTPUT_PATH, "w") as f:
            json.dump(output_data, f, indent=2)
        last_write_time = now
        print(f"[{time.strftime('%H:%M:%S')}] Saved {len(output_data)} frames to {OUTPUT_PATH}")
        output_data.clear()


# Main
if __name__ == "__main__":
    optionsDict = {
        "clientAddress": "127.0.0.1",
        "serverAddress": "127.0.0.1",
        "use_multicast": True,
        "stream_type": "d"
    }

    client = NatNetClient()
    client.set_client_address(optionsDict["clientAddress"])
    client.set_server_address(optionsDict["serverAddress"])
    client.set_use_multicast(optionsDict["use_multicast"])
    client.set_print_level(0)

    client.rigid_body_listener = receive_rigid_body_frame

    print("Starting NatNet Streaming Client...")
    if not client.run(optionsDict["stream_type"]):
        print("ERROR: Could not start streaming client.")
        exit(1)

    time.sleep(1)
    if not client.connected():
        print("ERROR: Connection to Motive failed.")
        client.shutdown()
        exit(2)

    # Load model names
    client.get_data_descriptions()
    if client.rigidBodyDescriptions is not None:
        rigid_body_names = {rb.ID: rb.name for rb in client.rigidBodyDescriptions}
    else:
        print("Warning: No rigid body descriptions received.")

    print("Collecting data for 10 seconds...")
    try:
        time.sleep(3)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting early.")

    print("Shutting down client...")
    client.shutdown()

    if output_data:
        print(f"Saving remaining {len(output_data)} frames...")
        with open(OUTPUT_PATH, "w") as f:
            json.dump(output_data, f, indent=2)

    print("Done.")