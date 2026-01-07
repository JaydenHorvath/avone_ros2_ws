# AV.ONE Joystick Teleop
# File: teleop_joy.launch.py
# Launch:
#   ros2 launch avone teleop_joy.launch.py


# Purpose:
#   - Start the Linux joystick driver (joy_node)
#   - Start teleop_twist_joy to convert joystick axes into /cmd_vel (Twist)

# Control mapping (typical game controller):
#   - Hold enable_button to allow output (deadman switch)
#   - Linear velocity: left stick up/down  (axis_linear.x)
#   - Angular velocity: right stick left/right (axis_angular.yaw)

# Notes:
#   - Axis indices depend on your controller and OS mapping.
#     Use `ros2 topic echo /joy` to verify axes/buttons.
#   - teleop_twist_joy publishes geometry_msgs/Twist on /cmd_vel by default.


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy", executable="joy_node", name="joy_node", output="screen"
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                output="screen",
                parameters=[
                    {
                        # Linear velocity on left stick (usually left stick vertical)
                        "axis_linear.x": 1,
                        "scale_linear.x": 4.0,
                        # Angular velocity on right stick (usually right stick horizontal)
                        "axis_angular.yaw": 3,
                        "scale_angular.yaw": 0.3,
                        # Deadman enable button (must be held)
                        "enable_button": 4,
                        "require_enable_button": True,
                    }
                ],
            ),
        ]
    )
