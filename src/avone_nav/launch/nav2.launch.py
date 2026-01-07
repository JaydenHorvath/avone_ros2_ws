# AV.ONE Nav2 Bringup
# File: nav2.launch.py
# Launch Command:
#           ros2 launch avone_nav nav2.launch.py

# Overview of Nav2:
#   Nav2 (Navigation2) is ROS 2's navigation framework. It takes a goal pose (PoseStamped) and
#   produces velocity commands (cmd_vel) to move the robot to that goal while avoiding obstacles.
#   In a typical stack it uses:
#     - TF frames (map, odom, base_link) to understand robot pose
#     - Costmaps (local/global) built from sensors (eg LiDAR pointcloud) for obstacle avoidance
#     - A planner (ComputePathToPose) to generate a path
#     - A controller (FollowPath) to track that path and output cmd_vel
#     - A behavior tree to sequence planning + control and handle recoveries

# Purpose of this launch file:
#   - Resolve the AV.ONE Nav2 parameter file (nav2_config.yaml) from the avone_nav package
#   - Include Nav2's standard bringup launch (nav2_bringup/navigation_launch.py)
#   - Pass the params_file so Nav2 uses AV.ONE tuning (controllers, costmaps, planner, BT, etc.)

# Notes:
#   - use_sim_time is set to "false" here (real clock). For Gazebo, set this to "true".
#   - This file only includes Nav2 bringup. It assumes localization and TF are already running.


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve path to Nav2 config inside the package
    nav2_config = os.path.join(
        get_package_share_directory("avone_nav"), "config", "nav2_config.yaml"
    )

    return LaunchDescription(
        [
            # Launch the main Nav2 bringup launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("nav2_bringup"),
                        "launch",
                        "navigation_launch.py",
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "params_file": nav2_config,
                }.items(),
            ),
        ]
    )
