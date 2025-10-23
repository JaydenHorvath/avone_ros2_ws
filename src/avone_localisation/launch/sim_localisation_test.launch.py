#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------- Launch Arguments -------------------
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package paths
    avone_pkg = FindPackageShare('avone')
    avone_gps_pkg = FindPackageShare('avone_gps')
    avone_localisation_pkg = FindPackageShare('avone_localisation')

    # ------------------- Included Launch Files -------------------
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([avone_pkg, 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    local_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([avone_localisation_pkg, 'launch', 'local_ekf.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_yaml': 'sim_local_ekf.yaml'
        }.items()
    )

    global_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([avone_localisation_pkg, 'launch', 'global_ekf.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ekf_yaml': 'sim_global_ekf.yaml'
        }.items()
    )

    navsat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([avone_localisation_pkg, 'launch', 'sim_navsat_transform.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ------------------- Standalone Nodes -------------------
    gps_imu_sim_node = Node(
        package='avone_gps',
        executable='gps_imu_sim',
        name='gps_imu_sim',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    vel_cov_repub_node = Node(
        package='avone_gps',
        executable='vel_cov_repub',
        name='vel_cov_repub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    heading_to_imu_node = Node(
        package='avone_gps',
        executable='heading_to_imu',
        name='heading_to_imu',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------- Launch Description -------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        # Launch order
        rsp_launch,
        gps_imu_sim_node,
        vel_cov_repub_node,
        heading_to_imu_node,
        local_ekf_launch,
        global_ekf_launch,
        navsat_launch,
    ])
