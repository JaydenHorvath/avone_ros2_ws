#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import csv
import os
import math


class ConeLogger(Node):
    def __init__(self):
        super().__init__('cone_logger')

        # Output CSV file (default in current directory)
        self.csv_path = os.path.join(os.getcwd(), 'cones_log.csv')

        # Duplicate filtering threshold (in meters)
        self.merge_threshold = 0.25  # adjust as needed

        # Subscribe to filtered cone markers
        self.create_subscription(MarkerArray, '/cones/tracks/markers_filtered', self.cb, 10)

        self.get_logger().info(f"Logging cones (with colour, deduplicated) to {self.csv_path}")

    def classify_colour(self, r, g, b):
        """Classify cone colour based on RGB values."""
        if r > 0.8 and g > 0.4 and b < 0.2:
            return 'orange'
        elif b > 0.8 and r < 0.2 and g < 0.2:
            return 'blue'
        elif r > 0.8 and g > 0.8 and b < 0.2:
            return 'yellow'
        else:
            return 'unknown'

    def is_duplicate(self, x, y, existing_cones):
        """Return True if (x, y) is within threshold distance of any existing cone."""
        for ex, ey in existing_cones:
            if math.hypot(x - ex, y - ey) < self.merge_threshold:
                return True
        return False

    def cb(self, msg):
        """Callback: save all unique cone positions and colours to CSV."""
        if not msg.markers:
            return

        try:
            seen_positions = []
            filtered_cones = []

            # Filter duplicates
            for m in msg.markers:
                pos = m.pose.position
                if not self.is_duplicate(pos.x, pos.y, seen_positions):
                    seen_positions.append((pos.x, pos.y))
                    filtered_cones.append(m)

            # Write to CSV
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['id', 'x', 'y', 'z', 'frame', 'colour'])
                for m in filtered_cones:
                    pos = m.pose.position
                    color = m.color
                    cone_colour = self.classify_colour(color.r, color.g, color.b)
                    writer.writerow([
                        m.id,
                        round(pos.x, 4),
                        round(pos.y, 4),
                        round(pos.z, 4),
                        m.header.frame_id,
                        cone_colour
                    ])

            self.get_logger().info(
                f"Saved {len(filtered_cones)} unique cones (from {len(msg.markers)}) → {self.csv_path}"
            )

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
