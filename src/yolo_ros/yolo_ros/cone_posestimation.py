#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from visualization_msgs.msg import Marker



class MultiConeLocalizer(Node):
    def __init__(self):
        super().__init__('cone_posestimation')

        # Camera intrinsics (filled on first CameraInfo)
        self.fx = self.fy = self.cx = self.cy = None

        # Real cone radii (m) by YOLO class ID
        self.radius_map = {
            '0': 0.14,  # small blue
            '4': 0.14,  # small yellow
            '1': 0.20,  # large orange (alt ID)
            '2': 0.20   # large orange
        }

        # Real cone heights (m) by YOLO class ID
        self.height_map = {
            '0': 0.325,
            '4': 0.325,
            '1': 0.505,
            '2': 0.505
        }

        # Blend width‐ and height‐based Z estimates
        self.alpha = 0.5

        # Colors (RGBA) by YOLO class ID
        self.color_map = {
            '0': (0.0, 0.0, 1.0, 0.8),  # blue
            '4': (1.0, 1.0, 0.0, 0.8),  # yellow
            '1': (1.0, 0.5, 0.0, 0.8),  # orange
            '2': (1.0, 0.5, 0.0, 0.8)
        }

        # Subscribers
        self.create_subscription(CameraInfo,
                                 '/yolo/camera_info',
                                 self.camera_info_cb, 10)
        self.create_subscription(Detection2DArray,
                                 '/yolo/detections',
                                 self.detections_cb, 10)

        # Publishers
        self.pose_pub   = self.create_publisher(PoseStamped,   '/cone_pose',    10)
        self.marker_pub = self.create_publisher(Marker,        '/cone_markers', 10)

    def camera_info_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def detections_cb(self, msg: Detection2DArray):
        if self.fx is None:
            return

        for idx, det in enumerate(msg.detections):
            hyp = det.results[0].hypothesis
            if hyp.score < 0.6:
                continue

            label = hyp.class_id
            R     = self.radius_map.get(label)
            H     = self.height_map.get(label)
            color = self.color_map.get(label)
            if R is None or H is None or color is None:
                self.get_logger().warn(f'Unknown label "{label}", skipping')
                continue

            # width‐based Z
            w_px = det.bbox.size_x
            Z_w  = (2.0 * self.fx * R) / w_px

            # height‐based Z
            h_px = det.bbox.size_y
            Z_h  = (self.fx * H) / h_px

            # fused depth
            Z = self.alpha * Z_w + (1.0 - self.alpha) * Z_h

            # back‐project
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            # skip far‐away cones
            max_range = 15.0  # meters
            r = math.sqrt(X*X + Y*Y + Z*Z)
            if r > max_range:
                continue

            # publish PoseStamped ps as before…
            ps = PoseStamped()
            ps.header = det.header
            ps.pose.position.x = X
            ps.pose.position.y = Y
            ps.pose.position.z = Z
            ps.pose.orientation.w = 1.0
            self.pose_pub.publish(ps)

            # now build the Marker from ps.pose.position
            m = Marker()
            m.header = ps.header
            m.ns     = f'cone_{label}'
            m.id     = idx
            m.type   = Marker.CUBE
            m.action = Marker.ADD

            # assign the marker pose from ps
            m.pose.position = ps.pose.position
            m.pose.orientation = ps.pose.orientation

            # size and color as before
            m.scale.x = 2.0 * R
            m.scale.y = 2.0 * R
            m.scale.z = H
            r,g,b,a = color
            m.color.r, m.color.g, m.color.b, m.color.a = r,g,b,a

            self.marker_pub.publish(m)

    def destroy_node(self):
        self.get_logger().info('Shutting down MultiConeLocalizer')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiConeLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
