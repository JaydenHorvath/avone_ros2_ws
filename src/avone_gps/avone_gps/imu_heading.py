#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math

def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw (degrees, 0–360)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg


class HeadingDirection(Node):
    def __init__(self):
        super().__init__('heading_direction')

        # Subscribes to IMU orientation
        self.sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        # Publishes combined heading string (optional)
        self.pub = self.create_publisher(String, '/imu/heading_text', 10)

        self.get_logger().info("🧭 HeadingDirection node started — listening on /imu/data")

    def imu_callback(self, msg):
        yaw = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )

        # Determine cardinal direction
        dirs = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        idx = int((yaw + 22.5) // 45) % 8
        direction = dirs[idx]

        # Print nicely
        self.get_logger().info(f'Heading: {yaw:6.2f}°  →  {direction}')

        # Optionally publish as text
        text_msg = String()
        text_msg.data = f"{yaw:.2f}° {direction}"
        self.pub.publish(text_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingDirection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
