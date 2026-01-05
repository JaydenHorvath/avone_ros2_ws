#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # --------------------------------------------------
    # Gazebo simulation
    # --------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v1 /home/avone/avone_ws/src/avone/worlds/empty.sdf'
        }.items()
    )

    # --------------------------------------------------
    # Robot description (xacro -> URDF)
    # --------------------------------------------------
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution([
                    FindPackageShare('avone'),
                    'description',
                    'robot.urdf.xacro'
                ])
            ]),
            value_type=str
        ),
        'use_sim_time': use_sim_time
    }


    # --------------------------------------------------
    # Spawn robot into Gazebo
    # --------------------------------------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
        parameters=[{'robot_description': robot_description}]
    )

    # --------------------------------------------------
    # Robot State Publisher (TF authority)
    # --------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        gz_sim,
        gz_spawn_entity,
        robot_state_publisher,
    ])
