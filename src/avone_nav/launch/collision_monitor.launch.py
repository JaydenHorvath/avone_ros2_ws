import os

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Path to your collision monitor YAML config
    collision_yaml = os.path.join(
        get_package_share_directory('avone_nav'),
        'config',
        'nav2_collision_monitor.yaml'
    )

    # Collision Monitor Lifecycle Node
    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace='',
        output='screen',
        parameters=[collision_yaml],
    )

    return LaunchDescription([
        collision_monitor_node
    ])
