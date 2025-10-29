from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avone_utils',
            executable='ros_cmd_heartbeat.py',
            name='ros_cmd_heartbeat',
            output='screen',
        ),
        Node(
            package='avone_utils',
            executable='sensor_timeout.py',
            name='sensor_timeout',
            output='screen',
        ),
    ])
