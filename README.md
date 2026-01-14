# AV.ONE ROS 2 Workspace (avone_ros2_ws)

ROS 2 Humble workspace for NU Racing Driverless platform **AV.ONE**.  
This repo contains the core autonomy stack used in simulation and on-hardware, including localisation, Nav2, control bridging, CAN interfacing, utilities, and cone detection.

---

## Repo Structure (high level)

- `avone_localisation/`
  - EKF based localisation for AV.ONE (single EKF setup for simpler testing)
  - Typical inputs: IMU, GPS derived odom, wheel odom (when available)
  - Outputs: filtered odometry for Nav2

- `avone_mapping/`
  - Mapping and map utilities for AV.ONE
  - Used for generating and consuming maps during simulation and testing

- `avone_nav/`
  - Main Nav2 package configuration and bringup
  - Includes tools for CAN based goal start and cancel:
    - `nav2_cancel` (start and cancel goals via CAN triggers)
  - Collision detection:
    - Standard LiDAR based detection
    - Sends a CAN message if an object is detected in front of the car

- `avone_utils/`
  - Misc utilities and glue nodes
  - `cmd_filter`: layer between Nav2 and Ackermann controller
    - blocks steering commands until linear velocity is present
  - `drive_state_led`: sends vehicle state colour to ESP32 LED matrix
  - `ros_cmd_heartbeat`: safety heartbeat outputs sent to the LVD

- `avone_can/`
  - CAN to ROS bridge nodes and CAN simulation tools
  - Provides decoded topics derived from DBC signals
  - Also contains simulated wheel speeds and steering angle topics for testing

- `avone_yolo/`
  - Cone detection and development tools (YOLO based)
  - Nodes:
    - `yolo_base`: minimal bare bones node
    - `yolo_node`: more developed node with correct coloured bounding boxes
    - `yolo_dev`: development node with position estimation and markers

---

## Requirements

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- Colcon build tools

Recommended ROS installs:
- `nav2_bringup`, `robot_localization`, `ros2_control` and controllers
- Sensor drivers as required (IMU, GPS, LiDAR, camera)

---

Note: 
- For Gazebo simulation to work, the path's to AV.ONE's meshes must be    hard coded to meshes path. This can be changed in avone/description/robot_core.xacro

- Similar with CAN Programs utilising the AV1.dbc, this dbc path should be changed to your local NUCAN git repository



## Build

```bash

# clone this repo into src
git clone <YOUR_REPO_SSH_OR_HTTPS_URL>

# install dependencies (run from workspace root)
cd ~/avone_ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# build
colcon build --symlink-install

# source
source install/setup.bash



