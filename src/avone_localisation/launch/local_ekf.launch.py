import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ekf_yaml = PathJoinSubstitution([FindPackageShare('avone_localisation'), 'config', 'local_ekf.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, ekf_yaml],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
            ]
        ),
    ])
