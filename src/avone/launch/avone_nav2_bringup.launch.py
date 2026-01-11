# AV.ONE Nav2 Bringup
# File: avone_nav2_bringup.launch.py
# Launch Command: ros2 launch avone avone_nav2_bringup.launch.py

# Purpose:
#   - Start a small node that listens for RC CAN messages to start or cancel nav2 goals
#   - Include the main Nav2 launch file from the avone_nav package (nav2.launch.py)

# Notes:
#   - This launch file assumes avone_nav/launch/nav2.launch.py exists and contains the full Nav2 stack.
#   - nav2_cancel is a convenience node for quickly stopping a running mission or clearing goals.
#   - If Nav2 needs specific parameters (maps, params YAML, use_sim_time, etc), those are handled in nav2.launch.py.


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nav = get_package_share_directory("avone_nav")

    return LaunchDescription(
        [
            # Nav2 CAN message start/cancel
            Node(
                package="avone_nav",
                executable="nav2_cancel",
                name="nav2_cancel",
                output="screen",
            ),
            # Main Nav2 bringup (full stack lives in avone_nav)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav, "launch", "nav2.launch.py")
                )
            ),
        ]
    )
