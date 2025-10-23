from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quanergy_client_ros',
            executable='client.launch.py',
            name='quanergy_client',
            output='screen',
            parameters=[{'host': '192.168.2.57'}],
        ),
    ])
