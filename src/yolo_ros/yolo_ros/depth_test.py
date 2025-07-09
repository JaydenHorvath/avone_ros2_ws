#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.sub = self.create_subscription(
            Image, '/depth_anything/depth_raw',
            self.depth_cb, 10)
        self.pub_colored = self.create_publisher(
            Image, '/depth/colored', 10)
        self.pub_points = self.create_publisher(
            Image, '/depth/points', 10)

    def depth_cb(self, msg):
        # 1) grab raw, unconverted depth
        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 2) convert to float32 metres
        if msg.encoding == '16UC1':
            depth = depth_raw.astype(np.float32) * 0.001
        else:
            depth = depth_raw.astype(np.float32)

        # Mask out invalid values for min/max computation
        finite_mask = np.isfinite(depth)
        if not finite_mask.any():
            return  # nothing to display

        # 3) compute dynamic min/max and normalize to [0,255]
        d_min = float(np.min(depth[finite_mask]))
        d_max = float(np.max(depth[finite_mask]))
        # avoid div-zero
        span = d_max - d_min if d_max > d_min else 1.0
        norm = ((depth - d_min) / span * 255.0)
        norm[~finite_mask] = 0
        norm = norm.astype(np.uint8)

        # 4) apply jet colormap
        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        # Annotate dynamic min/max
        h, w = norm.shape
        cv2.putText(color, f"Max: {d_max:.2f} m", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(color, f"Min: {d_min:.2f} m", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)

        # Colorbar
        bar_h, bar_w = h - 100, 20
        gradient = np.linspace(255, 0, bar_h, dtype=np.uint8)[:, None]
        bar = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)
        bar = np.tile(bar, (1, bar_w, 1))
        x0, y0 = w - bar_w - 20, 50
        color[y0:y0+bar_h, x0:x0+bar_w] = bar
        cv2.putText(color, f"{d_max:.2f} m", (x0 - 60, y0 + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
        cv2.putText(color, f"{d_min:.2f} m", (x0 - 60, y0 + bar_h),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)

        # Publish colored depth map
        out_colored = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
        out_colored.header = msg.header
        self.pub_colored.publish(out_colored)

        # 5) generate points overlay image (no longer capped)
        points_img = color.copy()
        step = 50  # sample every 50 pixels
        for v in range(0, h, step):
            for u in range(0, w, step):
                Z = depth[v, u]
                if np.isfinite(Z):
                    cv2.circle(points_img, (u, v), 5, (255,255,255), -1)
                    cv2.putText(points_img, f"{Z:.2f}",
                                (u+5, v-5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.4, (255,255,255), 1)

        # Publish points overlay
        out_points = self.bridge.cv2_to_imgmsg(points_img, encoding='bgr8')
        out_points.header = msg.header
        self.pub_points.publish(out_points)


def main():
    rclpy.init()
    node = DepthVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
