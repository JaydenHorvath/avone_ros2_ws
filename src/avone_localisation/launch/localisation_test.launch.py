#!/usr/bin/env python3
#
# AVONE localisation + GPS replay stack (no sim time)
# This replicates your Terminator layout:
#   - RSP (robot_state_publisher)
#   - GPS nodes (gps_linux, gps_topics_vts)
#   - EKF local + global
#   - navsat_transform
#   - rosbag playback (fix, heading, odometry/gps, vel, with /clock)
#
# Author: Jayden Horvath
#

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    # bag_path = LaunchConfiguration('bag_path')
    # bag_topics = LaunchConfiguration('bag_topics')

    # declare_bag_path = DeclareLaunchArgument(
    #     'bag_path',
    #     default_value='/home/jay/ros2_ws/avone_rosbags/lct_carpark_1',
    #     description='Path to the rosbag to replay.'
    # )

    # declare_bag_topics = DeclareLaunchArgument(
    #     'bag_topics',
    #     default_value='/fix /heading /odometry/gps /vel',
    #     description='Topics to replay from rosbag.'
    # )

    # ---------------------------------------------------------------------
    # 1. Robot State Publisher (URDF)
    # ---------------------------------------------------------------------
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('avone'), 'launch', 'rsp.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ---------------------------------------------------------------------
    # 2. GPS Nodes
    # ---------------------------------------------------------------------
    gps_topics_vis_node = Node(
        package='avone_gps',
        executable='gps_topics_vis',
        name='gps_topics_vis',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    # )

    gps_imu_sim_node = Node(
        package='avone_gps',
        executable='gps_imu_sim',
        name='gps_imu_sim',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ---------------------------------------------------------------------
    # 3. EKF Nodes (Local + Global)
    # ---------------------------------------------------------------------
    ekf_local_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('avone_localisation'),
                                  'launch', 'ekf_local.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    ekf_global_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('avone_localisation'),
                                  'launch', 'ekf_global.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ---------------------------------------------------------------------
    # 4. NavSat Transform Node
    # ---------------------------------------------------------------------
    navsat_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('avone_localisation'),
                                  'launch', 'navsat_transform.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # ---------------------------------------------------------------------
    # 5. Rosbag Play Process
    # ---------------------------------------------------------------------
    # bag_play = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'bag', 'play', bag_path,
    #         '--topics', bag_topics,
    #         '--clock'
    #     ],
    #     output='screen'
    # )

    # ---------------------------------------------------------------------
    # Return combined LaunchDescription
    # ---------------------------------------------------------------------
    return LaunchDescription([
        # declare_bag_path,
        # declare_bag_topics,
        rsp_launch,
        gps_imu_sim_node,
        gps_topics_vis_node,
        ekf_local_launch,
        ekf_global_launch,
        navsat_transform_launch,
        # bag_play
    ])
