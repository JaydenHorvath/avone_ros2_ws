#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) Point directly at your empty.sdf in the installed share
    world_file = PathJoinSubstitution([
        FindPackageShare('avone'),
        'worlds',
        'empty.sdf'
    ])

    # 2) Launch Ignition Gazebo in “run” mode (-r) so it skips the quick-start
    start_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

    # 3) Generate the robot_description (sim mode)
    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('avone'),
                'description',
                'robot.urdf.xacro'
            ]),
            ' use_sim:=true'
        ])
    }

    # 4) Start the robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5) Spawn the model into Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'avone'
        ]
    )

    return LaunchDescription([
        start_gazebo,
        rsp_node,
        spawn_node,
    ])
