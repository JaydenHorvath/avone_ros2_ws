from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the YOLO inference node with use_sim_time set to true.
    """
    return LaunchDescription([
        Node(
            package='yolo_ros',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},   # ← enable sim-time here
            ]
        ),
    ])
