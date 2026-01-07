# AV.ONE NavSat Transform Bringup
# File: navsat_transform.launch.py
# Package: avone_localisation
# Launch Command: ros2 launch avone_localisation navsat_transform.launch.py
#
# Purpose:
#   - Start Robot Localization's navsat_transform_node to fuse GPS (NavSatFix) with an IMU heading
#     and a reference odometry source, producing GPS-derived odometry in the local/world frame.
#   - Output is used as an input into a global EKF (map-level localisation).
#
# Notes:
#   - Configuration is loaded from: avone_localisation/config/navsat_transform.yaml
#   - Input topics are remapped to match AV.ONE conventions:
#       * gps/fix  -> /fix          (sensor_msgs/msg/NavSatFix)
#       * imu      -> /imu_gps      (sensor_msgs/msg/Imu, yaw is important)
#       * odometry/filtered -> /odometry/global1  (reference odom, adjust if needed)
#   - Output GPS odometry is remapped:
#       * odometry/gps -> /odometry/gps1
#   - A LogInfo warning is printed at launch to remind you to verify the filtered odom remap.
#
# Typical outputs:
#   - /odometry/gps1  (nav_msgs/msg/Odometry)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Path to your YAML config
    config_file = PathJoinSubstitution(
        [FindPackageShare("avone_localisation"), "config", "navsat_transform.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock",
            ),
            # --------------------------------------------------
            # ⚠️ IMPORTANT NOTICE
            # --------------------------------------------------
            LogInfo(
                msg=[
                    "[AV.ONE][navsat_transform] ",
                    "Check odometry remap: odometry/filtered",
                    "(CHANGE THIS IF YOUR PRIMARY ODOM TOPIC IS DIFFERENT)",
                ]
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[config_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("gps/fix", "/fix"),
                    ("imu", "/imu_gps"),
                    # CHANGE THIS TO WHATEVER THE OUTPUT ODOM IS
                    ("odometry/filtered", "/odometry/global1"),
                    ("odometry/gps", "/odometry/gps1"),
                ],
            ),
        ]
    )
