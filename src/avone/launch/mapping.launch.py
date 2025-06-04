#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1) Include the YOLO launch from the avone package
    # -------------------------------------------------------------------------
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('avone'),
                'launch',
                'yolo.launch.py'
            ])
        ])
    )

    # -------------------------------------------------------------------------
    # 2) Run the ground removal node from cone_mapper
    # -------------------------------------------------------------------------
    ground_removal_node = Node(
        package='cone_mapper',
        executable='groundremoval',
        name='ground_removal',
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 3) Run the cone landmark mapper node from cone_mapper
    # -------------------------------------------------------------------------
    cone_landmark_mapper_node = Node(
        package='cone_mapper',
        executable='conelandmarkmapper',
        name='cone_landmark_mapper',
        output='screen'
    )

    return LaunchDescription([
        yolo_launch,
        ground_removal_node,
        cone_landmark_mapper_node,
    ])
