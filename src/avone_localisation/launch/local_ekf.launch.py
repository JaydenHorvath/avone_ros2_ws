# AV.ONE Local EKF Bringup
# File: local_ekf.launch.py
# Package: avone_localisation
# Launch Command: ros2 launch avone_localisation local_ekf.launch.py
#
# Purpose:
#   - Start a "local" Robot Localization EKF (ekf_node) using local_ekf.yaml.
#   - Publish filtered local odometry on /odometry/local1 for use by local planners, TF, and debugging.
#
# Notes:
#   - local_ekf.yaml should define the sensor inputs used for local state estimation
#     ( IMU + wheel odometry), and frames (odom/base_link).
#   - use_sim_time is exposed as a launch argument (defaults false).
#   - This launch file only remaps ekf_node's filtered odometry output topic.
#
# Typical outputs:
#   - /odometry/local1  (nav_msgs/msg/Odometry)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    ekf_yaml = PathJoinSubstitution(
        [FindPackageShare("avone_localisation"), "config", "local_ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_yaml],
                remappings=[
                    ("odometry/filtered", "/odometry/local1"),
                ],
            ),
        ]
    )
