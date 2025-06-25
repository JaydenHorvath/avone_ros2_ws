#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Allow use_sim_time override
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Process robot description from Xacro
    pkg_dir = get_package_share_directory('avone')
    xacro_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Wheel joint names exactly as in URDF
    joint_list = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint'
    ]

    return LaunchDescription([
        # Override sim time if needed
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use real or simulation clock'
        ),

        # Publish robot_description and static TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': robot_description}
            ]
        ),

        # GUI slider to manually drive joint angles
        Node(
              package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
           
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'view_bot.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
