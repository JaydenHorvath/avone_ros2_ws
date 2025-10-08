#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelFilter(Node):
    def __init__(self):
        super().__init__('cmd_vel_filter')

        self.sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.min_linear = 0.05  # m/s deadband
        self.max_steer_scale =  1.9 #angular scaling factor

    def cmd_callback(self, msg: Twist):
        filtered = Twist()
        filtered.linear.x = msg.linear.x

        if abs(msg.linear.x) < self.min_linear:
            # Ignore angular command if car is nearly stopped
            filtered.angular.z = 0.0
        else:
            # Scale angular input based on forward speed
            filtered.angular.z = msg.angular.z * abs(msg.linear.x) * self.max_steer_scale

        self.pub.publish(filtered)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
