import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Resolve path to Nav2 config inside the package
    nav2_config = os.path.join(
        get_package_share_directory('avone_nav'),
        'config',
        'nav2_config.yaml'
    )

    return LaunchDescription([
        # Launch the main Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': nav2_config
            }.items()
        ),
        
    ])
