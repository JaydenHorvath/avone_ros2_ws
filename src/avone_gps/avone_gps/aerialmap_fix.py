#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class FixLatch(Node):
    def __init__(self):
        super().__init__('fix_latch')

        self.sub = self.create_subscription(NavSatFix, '/fix', self.cb, 10)
        self.pub = self.create_publisher(NavSatFix, '/fix_static', 10)

        self.latched = None
        self.timer = self.create_timer(0.5, self.tick)  # re-publish at 2 Hz

    def cb(self, msg: NavSatFix):
        # Latch only the *first* GPS fix
        if self.latched is None:
            self.latched = msg
            self.latched.header.frame_id = 'odom'  # 🔹 Force the odom frame
            self.get_logger().info(
                f'Latched first /fix at lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={msg.altitude:.2f}'
            )

    def tick(self):
        # Re-publish the latched fix with updated timestamp
        if self.latched is not None:
            msg = NavSatFix()
            msg = self.latched
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'  # 🔹 Keep frame fixed to odom
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = FixLatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
