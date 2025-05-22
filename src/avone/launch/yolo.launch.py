from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launches the YOLO inference node with the correct topic remapping.
    """
    return LaunchDescription([
        Node(
            package='yolo_ros',
            executable='yolo_node',   # match your installed script name
            name='yolo_node',
            
            
            output='screen'
        ),
    ])
