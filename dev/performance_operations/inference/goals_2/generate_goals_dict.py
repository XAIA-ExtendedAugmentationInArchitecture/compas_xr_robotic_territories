import json
import os

# Get the directory where this script lives
dir_path = os.path.dirname(os.path.realpath(__file__))

GOAL_SHAPES = {}

# Loop through all files in the directory
for filename in os.listdir(dir_path):
    if filename.endswith(".json"):
        file_path = os.path.join(dir_path, filename)

        with open(file_path, "r") as f:
            data = json.load(f)

        cube_locations = data["cube_locations"]
        points = []

        # Sort keys to preserve G0 -> G8 order
        for key in sorted(cube_locations.keys(), key=lambda x: int(x[1:])):
            frame_point = cube_locations[key]["data"]["frame"]["point"]
            x, y = frame_point[0], frame_point[1]
            points.append((round(x, 4), round(y, 4)))  # tuple of (x,y)

        goal_name = data["name"]  # e.g., "Goal00"
        GOAL_SHAPES[goal_name] = points

# Print the final dictionary
print("GOAL_SHAPES = {")
for k, v in GOAL_SHAPES.items():
    print(f'    "{k}": {v},')
print("}")