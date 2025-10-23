#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')

        # Subscribes to unfiltered velocity command
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Publishes filtered, stamped command to the Ackermann controller
        self.pub = self.create_publisher(
            TwistStamped,
            '/ackermann_steering_controller/reference',
            10
        )

        # Parameters
        self.min_linear = 0.1       # m/s deadband to ignore angular commands at low speeds
        self.max_steer_scale = 1.0  # angular scaling factor

    def cmd_callback(self, msg: Twist):
        # Create a stamped message
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'

        # --- Filter linear velocity (no reverse allowed) ---
        linear_x = max(0.0, msg.linear.x)
        stamped.twist.linear.x = linear_x

        # --- Apply angular filtering ---
        if linear_x < self.min_linear:
            # When nearly stopped, suppress steering
            stamped.twist.angular.z = 0.0
        else:
            # Scale angular input proportionally to forward speed
            stamped.twist.angular.z = msg.angular.z * linear_x * self.max_steer_scale

        # Publish the stamped command
        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
