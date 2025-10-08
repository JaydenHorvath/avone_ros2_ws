#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped


class TwistConverter(Node):
    def __init__(self):
        super().__init__('twist_converter')
        
        # Enable sim time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', 
                                                       rclpy.Parameter.Type.BOOL, 
                                                       True)])
        
        # Simple parameters
        self.declare_parameter('input_topic', '/vel')
        self.declare_parameter('output_topic', '/vel_cov')
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.flip_x = self.get_parameter('flip_x').value
        self.flip_y = self.get_parameter('flip_y').value
        
        # Subscriber and publisher
        self.sub = self.create_subscription(TwistStamped, input_topic, self.callback, 10)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, output_topic, 10)
        
        self.get_logger().info(f'Converting {input_topic} -> {output_topic}')
        self.get_logger().info(f'Flip: x={self.flip_x}, y={self.flip_y}')
        
    def callback(self, msg):
        out = TwistWithCovarianceStamped()
         # ✅ Set timestamp from input
        out.header.stamp = msg.header.stamp

        # ✅ FIX: explicitly say this twist is in odom frame
        out.header.frame_id = 'odom'
        
        # Copy velocities with optional flipping
        out.twist.twist.linear.x = msg.twist.linear.x * (-1 if self.flip_x else 1)
        out.twist.twist.linear.y = msg.twist.linear.y * (-1 if self.flip_y else 1)
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = msg.twist.angular.z
        
        # Covariance: 6x6 row-major = 36 elements [vx, vy, vz, vroll, vpitch, vyaw]
        # Higher values = less trust, so EKF relies more on GPS position
        covariance = [0.0] * 36
        covariance[0] = 2.0  # vx variance (increased to trust less)
        covariance[7] = 3.0    # vy variance (increased to trust less)
        covariance[14] = 1e9   # vz variance (unused)
        covariance[21] = 1e9  # vroll variance (unused)
        covariance[28] = 1e9   # vpitch variance (unused)
        covariance[35] = 1e9   # vyaw variance (increased to trust less)
        out.twist.covariance = covariance
        
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()