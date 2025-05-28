from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Launch teleop_twist_joy with parameters
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'stamp': True,
                'axis_linear.x': 1,
                'scale_linear.x': 2.0,
                'axis_angular.yaw': 3,
                'scale_angular.yaw': 0.75,
                'enable_button': 4,
                'repeat_rate': 50.0
            }],
            remappings=[
                ('/cmd_vel', '/ackermann_steering_controller/reference_unstamped')
            ]
        )
    ])
