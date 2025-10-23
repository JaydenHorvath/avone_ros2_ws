#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import csv
import os


class ConeLogger(Node):
    def __init__(self):
        super().__init__('cone_logger')

        # Output CSV file (default in current directory)
        self.csv_path = os.path.join(os.getcwd(), 'cones_log.csv')

        # Subscribe to filtered cone markers
        self.create_subscription(MarkerArray, '/cones/tracks/markers_filtered', self.cb, 10)

        self.get_logger().info(f"Logging cones to {self.csv_path}")

    def cb(self, msg):
        """Callback: save all cone positions to CSV."""
        if not msg.markers:
            return

        try:
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'x', 'y', 'z', 'frame'])
                for m in msg.markers:
                    pos = m.pose.position
                    writer.writerow([
                        m.id,
                        round(pos.x, 4),
                        round(pos.y, 4),
                        round(pos.z, 4),
                        m.header.frame_id
                    ])
            self.get_logger().info(f"Saved {len(msg.markers)} cones → {self.csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")


def main():
    rclpy.init()
    node = ConeLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
