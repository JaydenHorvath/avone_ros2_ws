#!/usr/bin/env python3
"""
Stereo image concatenation & blend node for Formula SAE AV.ONE
-------------------------------------------------------------
* Subscribes to left and right camera image topics:
    /rgb_left/image
    /rgb_right/image
* Synchronizes the two streams approximately in time.
* Horizontally concatenates incoming frames into one wide view,
  cutting out a central offset to account for the 20 mm baseline.
* Applies a smooth blend transition across the cut boundary.
* Publishes the blended wide image on /wide_view/image

Author: ChatGPT (OpenAI) for Jay — 02 Jul 2025
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
try:
    from message_filters import ApproximateTimeSynchronizer, Subscriber
except ImportError:
    raise ImportError("Please install ros-<distro>-message-filters to use ApproximateTimeSynchronizer")

import cv2
import numpy as np

class StereoBlendNode(Node):
    def __init__(self):
        super().__init__('stereo_blend_node')
        self.bridge = CvBridge()

        # Subscribers for left and right image topics
        left_sub = Subscriber(self, Image, '/rgb_left/image')
        right_sub = Subscriber(self, Image, '/rgb_right/image')

        # Approximate time synchronizer
        self.sync = ApproximateTimeSynchronizer(
            [left_sub, right_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=False
        )
        self.sync.registerCallback(self.callback)

        # Publisher for the blended wide view
        self.pub = self.create_publisher(Image, '/wide_view/image', 10)
        self.get_logger().info('StereoBlendNode initialized, publishing /wide_view/image')

        # blending and cropping parameters
        self.blend_width = 50    # pixels for linear alpha transition
        self.cut_offset  = 100   # pixels to cut from inner edges (approx. 20 mm)

    def callback(self, left_msg: Image, right_msg: Image):
        # Convert ROS Image to OpenCV BGR
        try:
            left_img  = self.bridge.imgmsg_to_cv2(left_msg,  'bgr8')
            right_img = self.bridge.imgmsg_to_cv2(right_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Crop to same height
        hL, wL = left_img.shape[:2]
        hR, wR = right_img.shape[:2]
        h = min(hL, hR)
        left_crop  = left_img[:h, :]
        right_crop = right_img[:h, :]

        # ensure cut_offset + blend_width fit
        bw = min(self.blend_width, (wL - self.cut_offset)//2, (wR - self.cut_offset)//2)
        co = min(self.cut_offset, wL//2, wR//2)

        # define regions: left image up to (wL - co - bw), blend region width bw, right image starting at (co + bw)
        left_region  = left_crop[:, :wL - co - bw]
        blend_left   = left_crop[:, wL - co - bw : wL - co]
        blend_right  = right_crop[:, co : co + bw]
        right_region = right_crop[:, co + bw :]

        # create alpha mask for blending bw-wide strip
        alpha = np.linspace(0.0, 1.0, bw, dtype=np.float32)
        alpha = alpha[np.newaxis, :, np.newaxis]            # shape (1, bw, 1)
        alpha = np.repeat(alpha, h, axis=0)                 # shape (h, bw, 1)
        alpha = np.repeat(alpha, 3, axis=2)                 # shape (h, bw, 3)

        # blend the overlapping strips
        blended_strip = (blend_left.astype(np.float32) * (1 - alpha) +
                         blend_right.astype(np.float32) * alpha)
        blended_strip = blended_strip.astype(np.uint8)

        # stitch: left_region, blended_strip, right_region
        wide_img = np.hstack([left_region, blended_strip, right_region])

        # Convert back to ROS Image and publish
        out_msg = self.bridge.cv2_to_imgmsg(wide_img, 'bgr8')
        out_msg.header = left_msg.header
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoBlendNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
