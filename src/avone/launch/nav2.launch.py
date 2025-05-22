import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Full path to your Nav2 config file
    nav2_config = os.path.expanduser('~/ros2_ws/src/avone/config/Nav2.yaml')

    return LaunchDescription([
        # Launch the main Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_config
            }.items()
        ),

        # Relay /cmd_vel to ackermann topic
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            arguments=['/cmd_vel', '/ackermann_steering_controller/reference_unstamped'],
            output='screen'
        )
    ])
