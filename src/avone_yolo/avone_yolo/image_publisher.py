#!/usr/bin/env python3

# Static Image Publisher (ROS 2)
# File: image_publisher.py

# Purpose:
#   - Publish a single image file repeatedly as a sensor_msgs/Image topic at a fixed rate.
#   - Useful for testing vision pipelines (CV nodes, camera calibration, YOLO, overlays, etc.)
#     without needing a live camera connected.

# How to run:
#   If installed as a ROS 2 node (recommended)
#   ros2 run avone_yolo image_publisher /full/path/to/image.png


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse


class ImagePublisher(Node):
    def __init__(self, image_path: str, topic: str, rate_hz: float):
        super().__init__("image_publisher")
        # Create a publisher for sensor_msgs/Image
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        # Load the image from disk
        self.frame = cv2.imread(image_path)
        if self.frame is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            raise FileNotFoundError(image_path)

        # Set up a timer to publish at the desired rate
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(
            f"Publishing '{image_path}' on topic '{topic}' at {rate_hz} Hz"
        )

    def timer_callback(self) -> None:
        # Convert OpenCV image (BGR8) to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description="ROS2 image publisher: publish a static image as a video stream"
    )
    parser.add_argument("image_path", help="Path to the image file to publish")
    parser.add_argument(
        "--topic",
        default="/camera/image_raw",
        help="ROS2 topic name to publish the image to",
    )
    parser.add_argument("--rate", type=float, default=5.0, help="Publish rate in Hz")
    args = parser.parse_args()

    node = ImagePublisher(args.image_path, args.topic, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
