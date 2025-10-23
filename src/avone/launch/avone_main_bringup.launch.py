#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_imu = LaunchConfiguration('enable_imu')
    enable_gps = LaunchConfiguration('enable_gps')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_utils = LaunchConfiguration('enable_utils')
    enable_localisation = LaunchConfiguration('enable_localisation')
    enable_nav2 = LaunchConfiguration('enable_nav2')

    # Package directories
    avone_pkg = get_package_share_directory('avone')
    localisation_pkg = get_package_share_directory('avone_localisation')
    nav_pkg = get_package_share_directory('avone_nav')
    utils_pkg = get_package_share_directory('avone_utils')
    lidar_pkg = get_package_share_directory('avone_lidar') if os.path.exists(
        os.path.join(get_package_share_directory('avone'), '../avone_lidar')
    ) else avone_pkg  # fallback if lidar in avone
    # Launch file paths
    bringup_launch = os.path.join(avone_pkg, 'launch', 'avone_control_bringup.launch.py')
    imu_launch = os.path.join(localisation_pkg, 'launch', 'avone_imu_bringup.launch.py')
    gps_launch = os.path.join(localisation_pkg, 'launch', 'avone_gps_bringup.launch.py')
    lidar_launch = os.path.join(lidar_pkg, 'launch', 'avone_lidar_bringup.launch.py')
    utils_launch = os.path.join(utils_pkg, 'launch', 'avone_utilities_bringup.launch.py')
    localisation_launch = os.path.join(localisation_pkg, 'launch', 'avone_localisation_bringup.launch.py')
    nav2_launch = os.path.join(nav_pkg, 'launch', 'avone_nav2_bringup.launch.py')

    return LaunchDescription([
        # ---------------- Launch Arguments ----------------
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true'),

        DeclareLaunchArgument('enable_imu', default_value='true', description='Launch IMU subsystem'),
        DeclareLaunchArgument('enable_gps', default_value='true', description='Launch GPS subsystem'),
        DeclareLaunchArgument('enable_lidar', default_value='true', description='Launch LiDAR subsystem'),
        DeclareLaunchArgument('enable_utils', default_value='true', description='Launch utility nodes'),
        DeclareLaunchArgument('enable_localisation', default_value='true', description='Launch localisation stack'),
        DeclareLaunchArgument('enable_nav2', default_value='true', description='Launch navigation stack'),

        # ---------------- Core Bringup (ROS2 Control, URDF, TF, etc.) ----------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ---------------- Subsystems ----------------
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch),
                condition=IfCondition(enable_imu),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gps_launch),
                condition=IfCondition(enable_gps),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch),
                condition=IfCondition(enable_lidar),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(utils_launch),
                condition=IfCondition(enable_utils),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(localisation_launch),
                condition=IfCondition(enable_localisation),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),

        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(enable_nav2),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            ),
        ]),
    ])
