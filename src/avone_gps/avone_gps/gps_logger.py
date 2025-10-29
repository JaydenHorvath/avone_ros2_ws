#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
from datetime import datetime

class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')

        # Declare parameters
        self.declare_parameter('topic', '/fix')
        self.declare_parameter('csv_path', 'gps_log.csv')
        self.declare_parameter('append', True)

        topic = self.get_parameter('topic').value
        csv_path = self.get_parameter('csv_path').value
        append = self.get_parameter('append').value

        # Set file mode (append or overwrite)
        file_mode = 'a' if append else 'w'
        self.csv_path = os.path.abspath(csv_path)
        self.csv_file = open(self.csv_path, file_mode, newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write header if new file
        if os.stat(self.csv_path).st_size == 0 or not append:
            self.csv_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])

        # Subscribe to GPS topic
        self.create_subscription(NavSatFix, topic, self.gps_callback, 10)
        self.get_logger().info(f"✅ Logging GPS data from {topic} → {self.csv_path}")

    def gps_callback(self, msg: NavSatFix):
        """Callback for incoming GPS messages."""
        timestamp = self.get_clock().now().to_msg()
        time_str = f"{timestamp.sec}.{str(timestamp.nanosec).zfill(9)}"
        self.csv_writer.writerow([time_str, msg.latitude, msg.longitude, msg.altitude])
        self.csv_file.flush()  # ensure immediate write to disk
        self.get_logger().info(f"📍 {msg.latitude:.6f}, {msg.longitude:.6f}, alt={msg.altitude:.2f}")

    def destroy_node(self):
        """Close file when node shuts down."""
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Stopping GPS logger.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
