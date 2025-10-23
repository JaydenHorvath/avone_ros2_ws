from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_nav = get_package_share_directory('avone_nav')

    return LaunchDescription([
        Node(
            package='avone_nav',
            executable='nav2_cancel.py',
            name='nav2_cancel',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'nav2.launch.py'))
        ),
    ])
