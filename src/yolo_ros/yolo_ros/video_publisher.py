#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys
import time

class VideoPublisher(Node):
    def __init__(self, video_path: str, topic_name: str = '/camera/image_raw'):
        super().__init__('video_publisher')
        self.bridge = CvBridge()
        self.topic   = topic_name

        # Create a publisher on the same topic your YOLO node is listening to:
        self.pub = self.create_publisher(Image, self.topic, 10)

        # OpenCV video capture:
        if not os.path.isfile(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            sys.exit(1)

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Unable to open video: {video_path}")
            sys.exit(1)

        # Get the FPS of the video so we can sleep between frames:
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 30.0
        self.delay = 1.0 / fps

        self.timer = self.create_timer(self.delay, self.timer_callback)
        self.get_logger().info(f"Publishing '{video_path}' → '{self.topic}' @ {fps:.1f} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # End of video: loop back to start (or shut down)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        # Convert BGR OpenCV frame → ROS Image:
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'

        self.pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run <your_package> video_publisher.py /path/to/video.mp4")
        sys.exit(1)

    video_path = sys.argv[1]
    node = VideoPublisher(video_path)
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
