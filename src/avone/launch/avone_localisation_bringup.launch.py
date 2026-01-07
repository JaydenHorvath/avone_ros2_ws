# AV.ONE Localisation Bringup
# File: avone_localisation_bringup.launch.py
# Launch Command: ros2 launch avone avone_localisation_bringup.launch.py

# Purpose:
#   - Start the local EKF (odom-frame) for smooth short term state estimation
#   - Start the global EKF (map-frame) to fuse GPS based position with local motion
#   - Start navsat_transform to convert GPS fixes into an odometry message usable by the EKFs

# Notes:
#   - This launch file simply includes three launch files from the avone_localisation package.
#   - local_ekf.launch.py should publish a local filtered odometry (example: /odometry/local1).
#   - global_ekf.launch.py should publish a global filtered odometry (example: /odometry/global1).
#   - navsat_transform.launch.py should consume /fix + /imu_gps + filtered odom and publish /odometry/gps1.
#   - If topic names differ, adjust remaps inside the included launch files (not here).


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("avone_localisation")

    return LaunchDescription(
        [
            # Local EKF (odom frame)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "local_ekf.launch.py")
                )
            ),
            # Global EKF (map frame)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "global_ekf.launch.py")
                )
            ),
            # NavSat transform (GPS fix -> odom)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, "launch", "navsat_transform.launch.py")
                )
            ),
        ]
    )
