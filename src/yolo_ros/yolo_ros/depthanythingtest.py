#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from transformers import pipeline
from PIL import Image as PILImage
import numpy as np
import cv2

class DepthAnythingLegendNode(Node):
    def __init__(self):
        super().__init__('depth_anything_legend_node')
        self.bridge = CvBridge()

        # Load a metric-scale, outdoor-trained Depth-Anything model (Large)
        self.depth_pipe = pipeline(
            task='depth-estimation',
            model='depth-anything/Depth-Anything-V2-Metric-Outdoor-Large-hf',
            device=0  # set to your CUDA device (or -1 for CPU)
        )

        # Subscribe to the raw camera feed
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for raw depth map (32FC1)
        self.raw_pub = self.create_publisher(
            Image,
            '/depth_anything/depth_raw',
            10
        )

        # Publisher for heatmap + dynamic legend (BGR8)
        self.viz_pub = self.create_publisher(
            Image,
            '/depth_anything/depth_viz',
            10
        )

        self.get_logger().info('DepthAnythingLegendNode initialized.')

    def colorize_with_legend(self, min_d, max_d, height=300, width=50):
        # Create a vertical gradient from 255 (max depth) → 0 (min depth)
        grad = np.linspace(255, 0, height, dtype=np.uint8)[:, None]
        grad = np.repeat(grad, width, axis=1)
        legend = cv2.applyColorMap(grad, cv2.COLORMAP_JET)

        # Annotate ticks: top = max_d, mid = mid, bottom = min_d
        ticks = [ (0.0, max_d), (0.5, (min_d + max_d) / 2), (1.0, min_d) ]
        for frac, val in ticks:
            y = int(frac * (height - 1))
            cv2.putText(
                legend,
                f"{val:.2f}m",
                (5, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )
        return legend

    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # BGR → RGB PIL
        pil_image = PILImage.fromarray(cv_image[..., ::-1])

        # Run depth estimation
        result = self.depth_pipe(pil_image)
        depth_map = np.array(result['depth'], dtype=np.float32)

        # Publish raw depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
        depth_msg.header = msg.header
        self.raw_pub.publish(depth_msg)

        # Compute dynamic min/max for this frame
        min_d = float(np.nanmin(depth_map))
        max_d = float(np.nanmax(depth_map))
        if max_d - min_d < 1e-3:
            # avoid div by zero
            max_d = min_d + 1e-3

        # Normalize & colormap for visualization
        norm = np.clip((depth_map - min_d) / (max_d - min_d), 0, 1)
        heat = (norm * 255).astype(np.uint8)
        heat_color = cv2.applyColorMap(heat, cv2.COLORMAP_JET)

        # Build dynamic legend and combine
        legend = self.colorize_with_legend(min_d, max_d)
        h, w = heat_color.shape[:2]
        legend_resized = cv2.resize(legend, (legend.shape[1], h))
        combined = np.hstack([heat_color, legend_resized])

        # Publish visualization
        viz_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthAnythingLegendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
