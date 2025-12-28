import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to your YAML config
    config_file = PathJoinSubstitution([
        FindPackageShare('avone_localisation'),
        'config',
        'navsat_transform.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        # --------------------------------------------------
        # ⚠️ IMPORTANT NOTICE
        # --------------------------------------------------
        LogInfo(
            msg=[
                '[AV.ONE][navsat_transform] ',
                'Check odometry remap: odometry/filtered',
                '(CHANGE THIS IF YOUR PRIMARY ODOM TOPIC IS DIFFERENT)'
            ]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('gps/fix', '/fix'),
                ('imu', '/imu_gps'),

                # CHANGE THIS TO WHATEVER THE OUTPUT ODOM IS
                ('odometry/filtered', '/odometry/global1'),

                ('odometry/gps', '/odometry/gps1'),
            ]
        ),
    ])
