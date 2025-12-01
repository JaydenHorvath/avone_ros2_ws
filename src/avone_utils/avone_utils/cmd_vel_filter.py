#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry


class SimpleSteeringCutoff(Node):
    def __init__(self):
        super().__init__('simple_steering_cutoff')

        # Minimum velocity before allowing steering
        self.min_velocity = 0.2  # m/s

        # State
        self.current_velocity = 0.0
        self.target_linear = 0.0
        self.target_omega = 0.0

        # Subscribers
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/ackermann_steering_controller/odometry', self.odom_callback, 10
        )

        # Publisher
        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )

        # Timer to publish at fixed rate
        self.create_timer(0.02, self.publish_command)  # 50 Hz

    def cmd_callback(self, msg: Twist):
        self.target_linear = msg.linear.x
        self.target_omega = msg.angular.z

    def odom_callback(self, msg: Odometry):
        self.current_velocity = msg.twist.twist.linear.x

    def publish_command(self):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = self.target_linear

        # Only allow steering if velocity is above threshold
        if self.current_velocity >= self.min_velocity:
            out.twist.angular.z = self.target_omega
        else:
            out.twist.angular.z = 0.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSteeringCutoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()