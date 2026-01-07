#!/usr/bin/env python3

# AV.ONE Global EKF Bringup
# File: global_ekf.launch.py
# Package: avone_localisation
# Launch Command: ros2 launch avone_localisation global_ekf.launch.py
#
# Purpose:
#   - Start a "global" Robot Localization EKF (ekf_node) using global_ekf.yaml.
#   - Publish filtered global odometry on /odometry/global1, used for map-level localisation
#     (fusing GPS/UTM + IMU + wheel odom) and feeding Nav2 global planning.
#
# Notes:
#   - global_ekf.yaml should define the global fusion setup ( GPS-derived odometry + GPS Heading),
#     and frame relationships (map -> odom -> base_link) depending on architecture.
#   - This launch file currently only remaps the filtered odometry output topic.
#
# Typical outputs:
#   - /odometry/global1  (nav_msgs/msg/Odometry)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    map_ekf_yaml = PathJoinSubstitution(
        [FindPackageShare("avone_localisation"), "config", "global_ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, map_ekf_yaml],
                remappings=[
                    ("odometry/filtered", "/odometry/global1"),
                ],
            ),
        ]
    )
