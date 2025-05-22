import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Update this to your actual package name
    pkg_my_package = FindPackageShare('avone')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': PathJoinSubstitution([
                    pkg_my_package,
                    'config',
                    'ros_gz_bridge.yaml'  # Your bridge config YAML
                ]),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }],
            output='screen'
        )
    ])
