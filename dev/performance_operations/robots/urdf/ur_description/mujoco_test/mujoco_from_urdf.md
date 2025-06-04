# UR20 URDF to MuJoCo MJCF Conversion Guide

This guide outlines the process of converting a UR20 robot from a URDF description into a valid MuJoCo MJCF (.xml) format. It includes only the successful steps that led to a functional MuJoCo model.

---

## 1. Environment Setup

### 1.1 Create and activate a new Conda environment

```bash
conda create -n mujoco_env python=3.10
conda activate mujoco_env
pip install urdf2mjcf mujoco glfw numpy
```

## 2. File Preparation

### 2.1 Project Directory Structure

- Ensure the following directory structure:

```txt    
    mujoco_test/
    ├── ur20.urdf
    ├── meshes/
    │   └── ur20/
    │       ├── collision/
    │       └── visual/
    └── output/
```
- ur20.urdf should be fully expanded (converted from .xacro).
- Mesh file paths inside the URDF should use relative paths such as:

    ```xml
    <mesh filename="meshes/ur20/visual/base.dae" />
    ```
- Avoid using package:// or ../ references.

## 3. URDF to MJCF Conversion

### 3.1 Run the Converter
- Run the converter from the mujoco_test directory (Running Anaconda Prompt as Administrator):
- Inputput path (Path to .urdf file next to meshes)
- --ouput (path to where you would like the mujoco.xml to be created)

```bash
urdf2mjcf C://.../.../urdf.urdf --output C://.../.../.../output/urdf_mujoco.xml --copy-meshes
```

### 3.2 Ouput Generation

This command should generate two outputs:
- output/ur20_mujoco.xml — the MJCF file
- output/meshes/ — the copied mesh assets

## 4. Fix Mesh Paths in MJCF

### 4.1 Updating paths in urdf_mujoco.xml
- Open output/ur20_mujoco.xml and modify the <asset> section to point to the correct mesh locations. For example:
```xml
<mesh name="base" file="meshes/ur20/collision/base.stl" />
<mesh name="shoulder" file="meshes/ur20/collision/shoulder.stl" />
<mesh name="upperarm" file="meshes/ur20/collision/upperarm.stl" />
<mesh name="forearm" file="meshes/ur20/collision/forearm.stl" />
<mesh name="wrist1" file="meshes/ur20/collision/wrist1.stl" />
<mesh name="wrist2" file="meshes/ur20/collision/wrist2.stl" />
<mesh name="wrist3" file="meshes/ur20/collision/wrist3.stl" />
```
- Ensure that all mesh paths are relative to the ur20_mujoco.xml file.

## 5 Fix the Root Body Position (If required)
This anchors the robot to the ground (i.e., prevent floating):

### 5.1 Remove the freejoint from the root body
- Delete this line from <body name="root">:
```xml
<freejoint name="root" />
```

### 5.2 Set the root body postion to the ground
- Change:
```xml
<body name="root" pos="0 0 0.3141691543396815" ...>
```
- To:
```xml
<body name="root" pos="0 0 0" ...>
```
- This ensures the robot base starts at the origin and remains fixed.

## 6. Run and Test the MJCF Model

### 6.1 Visualize the model
- Run the test script below to visualize the robot model:
- Input : Path to mujoco.xml

```py
import mujoco
import mujoco.viewer
import time

# Load the model
model = mujoco.MjModel.from_xml_path(
    r"C:\..\...\...\...urdf_mujoco.xml"
)

# Create a data instance
data = mujoco.MjData(model)

# Viewer (for manual inspection)
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer started. Close the window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
```
### 6.2 Ensure the following paramaters
- The robot appears correctly in the scene.
- It is fixed to the ground (not falling).
- Meshes load without errors.
- Joints are visible and can be controlled using the UI sliders or programmatically via .ctrl[] in Python.
