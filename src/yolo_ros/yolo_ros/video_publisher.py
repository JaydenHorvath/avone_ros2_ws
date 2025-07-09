#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import argparse
import os
import sys

class VideoPublisher(Node):
    def __init__(self, video_path: str, topic_name: str, width: int, height: int, speed: float):
        super().__init__('video_publisher')
        self.bridge = CvBridge()
        self.topic = topic_name
        self.width = width
        self.height = height
        self.speed = speed

        # Publisher
        self.pub = self.create_publisher(Image, self.topic, 10)

        # Open video
        if not os.path.isfile(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            sys.exit(1)
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open video: {video_path}")
            sys.exit(1)

        # Get original FPS
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0 or fps != fps:  # check for zero or NaN
            fps = 30.0

        # Compute callback interval: slower (<1) or faster (>1)
        interval = (1.0 / fps) / self.speed
        self.timer = self.create_timer(interval, self.timer_callback)

        self.get_logger().info(
            f"Publishing '{video_path}' at {fps:.1f} FPS "
            f"(speed={self.speed*100:.0f}%), resizing to {self.width}×{self.height} "
            f"on topic '{self.topic}'"
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # loop video
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Resize and publish
        frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        self.pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(
        description='Publish a video file as a ROS 2 image stream with adjustable speed'
    )
    parser.add_argument('video_path', help='Path to the video file')
    parser.add_argument('--topic', default='/camera/image_raw',
                        help='ROS 2 topic to publish')
    parser.add_argument('--width', type=int, default=640,
                        help='Frame width after resize')
    parser.add_argument('--height', type=int, default=360,
                        help='Frame height after resize')
    parser.add_argument('--speed', type=float, default=0.3,
                        help='Playback speed factor: 1.0 real-time, <1 slower, >1 faster')
    args = parser.parse_args()

    node = VideoPublisher(
        video_path=args.video_path,
        topic_name=args.topic,
        width=args.width,
        height=args.height,
        speed=args.speed
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
