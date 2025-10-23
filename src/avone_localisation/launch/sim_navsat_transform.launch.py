import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to your YAML config (assuming it's inside `config/` of your package)
    config_file = PathJoinSubstitution([
        FindPackageShare('avone_localisation'),  # <-- replace with your package name
        'config',
        'navsat_transform.yaml'
    ])

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
            parameters=[config_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('gps/fix', '/fix'),
                ('imu', '/imu/sim'),   
                ('odometry/filtered', '/odometry/sim'),
                ('odometry/gps', '/odometry/gps_new'),
                
            ]
        ),
    ])
