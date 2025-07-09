#!/usr/bin/env python3
"""
YOLO Depth Visualizer with Exponential Smoothing for Formula SAE AV.ONE
------------------------------------------------------------------------
* Subscribes to:
    - /yolo/image         (sensor_msgs/Image)
    - /yolo/detections    (vision_msgs/Detection2DArray)
    - /camera/camera/depth/image_rect_raw (sensor_msgs/Image)
* Synchronizes streams approximately in time.
* For each detection:
    - Finds bounding-box center pixel (u,v).
    - Samples a 5×5 patch around (u,v), takes the median of the valid Z’s.
    - Applies exponential smoothing to Z for steadier output.
    - Prints and overlays the smoothed distance on the RGB image.
* Additionally:
    - Generates a colored depth map and overlays detection points + smoothed values.
* Publishes:
    - /yolo/depth_vis     : RGB image + arrows + depth text
    - /yolo/depth_overlay : colored depth map + detection points + values
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
import math


class YoloDepthVisualizer(Node):
    def __init__(self):
        super().__init__('yolo_depth_visualizer')
        self.bridge = CvBridge()
        self.min_depth = 0.3    # metres
        self.max_depth = 15.0   # metres
        self.alpha = 0.3        # smoothing factor
        self.smoothed = {}      # idx -> smoothed depth
        self.point_dy = 0       # vertical offset for depth-overlay points

        # Subscribers
        rgb_sub   = Subscriber(self, Image, '/yolo/image')
        det_sub   = Subscriber(self, Detection2DArray, '/yolo/detections')
        # depth_sub = Subscriber(self, Image, '/depth/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/camera/depth/image_rect_raw')

        self.sync = ApproximateTimeSynchronizer(
            [rgb_sub, det_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.callback)

        # Publishers
        self.pub_vis     = self.create_publisher(Image, '/yolo/depth_vis', 10)
        self.pub_overlay = self.create_publisher(Image, '/yolo/depth_overlay', 10)
        self.get_logger().info('YOLO Depth Visualizer with overlay initialized.')

    def callback(self, img_msg, dets_msg, depth_msg):
        # 1) Convert inputs
        try:
            cv_img    = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 2) Depth to metres
        if depth_msg.encoding == '16UC1':
            depth = depth_raw.astype(np.float32) * 0.001
        else:
            depth = depth_raw.astype(np.float32)

        h, w = depth.shape
        origin = (w // 2, h - 1)

        # Prepare colored depth map
        depth_clipped = np.clip(depth, self.min_depth, self.max_depth)
        norm          = ((depth_clipped - self.min_depth)
                         / (self.max_depth - self.min_depth) * 255.0).astype(np.uint8)
        depth_color   = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # 3) Annotate both images per detection
        for idx, det in enumerate(dets_msg.detections):
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            # sample median in 5×5 patch
            u0, v0 = max(u-2, 0), max(v-2, 0)
            u1, v1 = min(u+3, w), min(v+3, h)
            patch   = depth[v0:v1, u0:u1].flatten()
            valid   = patch[np.isfinite(patch) & (patch > 0)]
            rawZ    = float(np.median(valid)) if valid.size else None

            # exponential smoothing
            if rawZ is not None:
                prev = self.smoothed.get(idx, rawZ)
                Z    = self.alpha * rawZ + (1 - self.alpha) * prev
                self.smoothed[idx] = Z
            else:
                Z = self.smoothed.get(idx)

            # only draw the arrow/vector if Z is a valid number
            if Z is not None and not math.isnan(Z):
                label  = f"{Z:.2f} m"
                pt_rgb = (u, v)
                mid    = ((origin[0] + u)//2, (origin[1] + v)//2)

                # draw arrow & text on RGB image
                cv2.arrowedLine(cv_img, origin, pt_rgb, (0,255,0), 2, tipLength=0.05)
                cv2.putText(cv_img, label, mid, cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (255,255,255), 2, cv2.LINE_AA)

            # draw circle & text on depth overlay (lowered by point_dy)
            pt_dep = (u, v + self.point_dy)
            if 0 <= pt_dep[0] < w and 0 <= pt_dep[1] < h:
                color = (255,255,255) if Z is not None else (0,0,255)
                cv2.circle(depth_color, pt_dep, 5, color, -1)
                cv2.putText(depth_color, label,
                            (pt_dep[0] + 6, pt_dep[1] - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 4) Publish visuals
        vis_msg     = self.bridge.cv2_to_imgmsg(cv_img,    'bgr8')
        vis_msg.header     = img_msg.header
        overlay_msg = self.bridge.cv2_to_imgmsg(depth_color, 'bgr8')
        overlay_msg.header = depth_msg.header

        self.pub_vis.publish(vis_msg)
        self.pub_overlay.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
