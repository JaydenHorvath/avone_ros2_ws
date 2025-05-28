import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Use /clock if running in Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths to your YAMLs in avone/config
    ekf_yaml      = PathJoinSubstitution([FindPackageShare('avone'), 'config', 'ekf.yaml'])
    map_ekf_yaml  = PathJoinSubstitution([FindPackageShare('avone'), 'config', 'map_ekf.yaml'])

    return LaunchDescription([
        # ----------------------------------------------------
        # 1) Argument
        # ----------------------------------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        # ----------------------------------------------------
        # 2) Local EKF: wheel odom + IMU → /odometry/local
        # ----------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                ekf_yaml
            ],
            remappings=[
                # this makes the node’s internal "odometry/filtered"
                # subscribe to "/odometry/local"
                ('odometry/filtered', '/odometry/local'),
            ]
        ),

        # ----------------------------------------------------
        # 3) NavSat Transform: GPS → /odometry/gps
        # ----------------------------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                # tune as needed:
                {'frequency': 30.0, 'publish_filtered_gps': True}
            ],
            remappings=[
                ('gps/fix',            '/navsat1'),
                ('odometry/filtered',  '/odometry/local'),
                ('odometry/gps',       '/odometry/gps'),
            ]
        ),

        # ----------------------------------------------------
        # 4) Global EKF: fuse /odometry/local + /odometry/gps
        # ----------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                map_ekf_yaml
            ],
            remappings=[
                # this makes the node’s internal "odometry/filtered"
                # subscribe to "/odometry/local"
                ('odometry/filtered', '/odometry/global'),
            ]
            # inputs are defined in your map_ekf.yaml (odom0=/odometry/local, odom1=/odometry/gps)
        ),
    ])
