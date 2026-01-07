#!/usr/bin/env python3

# Video Publisher (ROS 2)
# File: video_publisher.py

# Purpose:
#   - Publish a video file as a ROS 2 sensor_msgs/Image stream.
#   - Also publishes a matching sensor_msgs/CameraInfo message so downstream vision nodes
#     (image_proc, calibration, projection, overlays, etc.) can run with sensible intrinsics.

# Publishes:
#   - Image topic:      /camera/image_raw
#   - CameraInfo topic: /camera/camera_info
#   - frame_id:         "camera_frame"

# Notes:
#   - The video loops when it reaches the end.
#   - FPS is taken from the video metadata; if invalid, defaults to 30 FPS.
#   - Intrinsics are estimated from an approximate horizontal FOV (hfov_deg=82.0) and the video resolution.
#     This is good enough for many testing tasks, but not a substitute for real calibration.

# How to run:
#   ros2 run avone_yolo video_publisher /full/path/to/video.mp4

#   # Example: publish faster for stress testing with dedicated topic
#   ros2 run avone_yolo video_publisher /home/avone/test.mp4 --topic /camera/image_raw --speed 2.0


import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import cv2
import argparse
import os
import sys
import math


class VideoPublisher(Node):
    def __init__(self, video_path: str, topic_name: str, speed: float):
        super().__init__("video_publisher")

        self.bridge = CvBridge()
        self.topic = topic_name
        self.speed = speed
        self.frame_id = "camera_frame"

        # Publishers
        self.image_pub = self.create_publisher(Image, self.topic, 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        # Open video
        if not os.path.isfile(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            sys.exit(1)

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open video: {video_path}")
            sys.exit(1)

        # Native video properties
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)

        if fps <= 0 or fps != fps:
            fps = 30.0

        # -----------------------------
        # Pixel 9 intrinsics (approx)
        # -----------------------------
        # Main camera HFOV is about 82 degrees
        hfov_deg = 82.0
        hfov_rad = math.radians(hfov_deg)

        self.fx = (self.width / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        interval = (1.0 / fps) / self.speed
        self.timer = self.create_timer(interval, self.timer_callback)

        self.get_logger().info(
            f"Publishing '{video_path}' at {self.width}x{self.height}, "
            f"{fps:.1f} FPS (speed={self.speed*100:.0f}%)"
        )
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f}"
        )

    def make_camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id

        info.width = self.width
        info.height = self.height

        # Intrinsic matrix K
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]

        # No distortion model (acceptable for this use)
        info.d = []
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Projection matrix P
        info.p = [
            self.fx,
            0.0,
            self.cx,
            0.0,
            0.0,
            self.fy,
            self.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]

        return info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # Loop video
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        stamp = self.get_clock().now().to_msg()

        # Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = self.frame_id
        self.image_pub.publish(img_msg)

        # CameraInfo message
        caminfo_msg = self.make_camera_info(stamp)
        self.caminfo_pub.publish(caminfo_msg)


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(
        description="Publish a video file as a ROS 2 image stream with CameraInfo (Pixel 9)"
    )
    parser.add_argument("video_path", help="Path to the video file")
    parser.add_argument(
        "--topic", default="/camera/image_raw", help="ROS 2 topic to publish images"
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed factor: 1.0 real-time, <1 slower, >1 faster",
    )

    args = parser.parse_args()

    node = VideoPublisher(
        video_path=args.video_path, topic_name=args.topic, speed=args.speed
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
