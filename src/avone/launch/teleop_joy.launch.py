# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # Launch joy_node
#         Node(
#             package='joy',
#             executable='joy_node',
#             name='joy_node',
#             output='screen'
#         ),

#         # Launch teleop_twist_joy with parameters
#         Node(
#             package='teleop_twist_joy',
#             executable='teleop_node',
#             name='teleop_twist_joy_node',
#             output='screen',
#             parameters=[{
#                 'stamp': True,
#                 'axis_linear.x': 1,
#                 'scale_linear.x': 2.0,
#                 'axis_angular.yaw': 3,
#                 'scale_angular.yaw': 0.2,
#                 'enable_button': 4,
#                 'repeat_rate': 50.0
#             }],
#             remappings=[
#                 ('/cmd_vel', '/ackermann_steering_controller/reference_unstamped')
#             ]
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen'),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                'axis_linear.x': 1,
                'scale_linear.x': 2.0,
                'axis_angular.yaw': 3,
                'scale_angular.yaw': 0.2,
                'enable_button': 4,
                'require_enable_button': True
            }],
            # remappings=[('/cmd_vel', '/cmd_vel_raw')]
        ),
        # Node(
        #     package='avone_utils',
        #     executable='cmd_vel_filter',
        #     name='cmd_vel_filter',
        #     output='screen'
        # )
    ])
