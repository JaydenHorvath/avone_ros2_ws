#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelToAckermannSimple(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_simple')

        # Subscribe to /cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Publish to Ackermann controller
        self.pub = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10
        )

    def cmd_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'

        # Only allow steering if moving forward
        if msg.linear.x > 0.01:
            stamped.twist = msg
        else:
            # Stop steering when stopped or reversing
            stamped.twist.linear.x = max(0.0, msg.linear.x)
            stamped.twist.angular.z = 0.0

        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannSimple()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# # !/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TwistStamped

# class CmdVelToAckermannBypass(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_to_ackermann_bypass')

#         # Subscribe to /cmd_vel
#         self.sub = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_callback,
#             10
#         )

#         # Publish directly to the controller’s reference topic
#         self.pub = self.create_publisher(
#             TwistStamped,
#             '/ackermann_steering_controller/reference',
#             10
#         )

#     def cmd_callback(self, msg: Twist):
#         stamped = TwistStamped()
#         stamped.header.stamp = self.get_clock().now().to_msg()
#         stamped.header.frame_id = 'base_link'
#         stamped.twist = msg  # Directly pass through the received Twist
#         self.pub.publish(stamped)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CmdVelToAckermannBypass()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
