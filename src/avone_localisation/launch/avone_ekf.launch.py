# AV.ONE Localisation Single EKF Bringup
# File: avone_ekf.launch.py
# Package: avone_localisation
# Launch Command: ros2 launch avone_localisation avone_ekf.launch.py
#
# Purpose:
#   - Provides a simple accurate position estimation tool in one package, good to use with simulation
#   - Start a single Robot Localization EKF (ekf_node) to fuse onboard sensor data into a filtered odometry estimate.
#   - Publish filtered odometry on /odometry/avone for downstream nodes (Nav2, logging, visualization).
#
# Notes:
#   - Uses a YAML config file: avone_localisation/config/avone_ekf.yaml
#   - use_sim_time is exposed as a launch argument (defaults true for simulation).
#   - This launch file currently only remaps the filtered odometry output.
#   - All sensor inputs (imu, wheel odom, gps, etc.) are expected to be defined inside the YAML.
#
# Typical outputs:
#   - /odometry/avone  (nav_msgs/msg/Odometry)


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    ekf_yaml = PathJoinSubstitution(
        [FindPackageShare("avone_localisation"), "config", "avone_ekf.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}, ekf_yaml],
                remappings=[
                    ("odometry/filtered", "/odometry/avone"),
                ],
            ),
        ]
    )
