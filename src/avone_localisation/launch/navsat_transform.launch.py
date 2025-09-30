import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'frequency': 30.0, 'publish_filtered_gps': True}
            ],
            remappings=[
                ('gps/fix', '/navsat1'),
                ('odometry/filtered', '/odometry/local'),
                ('odometry/gps', '/odometry/gps'),
                ('imu/data', '/imu'),
            ]
        ),
    ])
