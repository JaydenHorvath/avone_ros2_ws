from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='avone_localisation',
            executable='imu_start',
            name='imu_start',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
