#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from builtin_interfaces.msg import Time


class ConeLineOverlayNode(Node):
    def __init__(self):
        super().__init__('cone_line_overlay_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.world_frame = 'map'
        self.camera_frame = 'camera_link_optical'

        self.blue_points = []
        self.yellow_points = []
        self.midpoints = []

        self.create_subscription(CameraInfo, '/camera/camera_info', self.cam_info_cb, 1)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 1)
        self.create_subscription(MarkerArray, '/cone_landmarks_blue', self.blue_cb, 10)
        self.create_subscription(MarkerArray, '/cone_landmarks_yellow', self.yellow_cb, 10)
        self.create_subscription(Marker, '/delaunay_midpoints', self.mid_cb, 10)

        self.marker_pub = self.create_publisher(Marker, '/cone_line_marker', 2)
        self.image_pub = self.create_publisher(Image, '/cone_overlay/image', 1)

        self.create_timer(0.2, self.publish_marker)

    def cam_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

    def blue_cb(self, msg):
        self.blue_points = [Point(x=m.pose.position.x, y=m.pose.position.y, z=0.0) for m in msg.markers]

    def yellow_cb(self, msg):
        self.yellow_points = [Point(x=m.pose.position.x, y=m.pose.position.y, z=0.0) for m in msg.markers]

    def mid_cb(self, msg):
        self.midpoints = [Point(x=p.x, y=p.y, z=0.0) for p in msg.points]

    def transform_point(self, point):
        ps = PointStamped()
        ps.header.frame_id = self.world_frame
        ps.header.stamp = Time()
        ps.point = point
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.world_frame,
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            return do_transform_point(ps, tf).point
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            return None

    def project_to_image(self, pt):
        if pt is None:
            return None
        z = max(pt.z, 0.001)
        pt_np = np.array([[pt.x, pt.y, z]], dtype=np.float32)
        uv, _ = cv2.projectPoints(pt_np, (0, 0, 0), (0, 0, 0), self.camera_matrix, self.dist_coeffs)
        u, v = uv[0][0]
        return (int(u), int(v))

    def euclidean_distance(self, p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return np.sqrt(dx * dx + dy * dy)

    def nearest_neighbor_lines(self, points):
        if len(points) < 2:
            return []

        used = [False] * len(points)
        idx = 0
        used[idx] = True
        lines = []

        while any(not u for u in used):
            min_dist = float('inf')
            closest_idx = -1
            for i, pt in enumerate(points):
                if not used[i]:
                    d = self.euclidean_distance(points[idx], pt)
                    if d < min_dist:
                        min_dist = d
                        closest_idx = i
            if closest_idx != -1:
                lines.append(points[idx])
                lines.append(points[closest_idx])
                used[closest_idx] = True
                idx = closest_idx

        return lines  # [p1, p2, p3, p4, ...]

    def publish_marker(self):
        def make_line_marker(lines, color, marker_id):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'cone_lines'
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            marker.points = lines
            return marker

        blue_lines = self.nearest_neighbor_lines(self.blue_points)
        yellow_lines = self.nearest_neighbor_lines(self.yellow_points)
        mid_lines = self.nearest_neighbor_lines(self.midpoints)

        if blue_lines:
            self.marker_pub.publish(make_line_marker(blue_lines, (0.0, 0.0, 1.0), 0))
        if yellow_lines:
            self.marker_pub.publish(make_line_marker(yellow_lines, (1.0, 1.0, 0.0), 1))
        if mid_lines:
            self.marker_pub.publish(make_line_marker(mid_lines, (0.0, 1.0, 0.0), 2))

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        def draw_lines(lines, color_bgr):
            for i in range(0, len(lines) - 1, 2):
                pt1 = self.transform_point(lines[i])
                pt2 = self.transform_point(lines[i + 1])
                uv1 = self.project_to_image(pt1)
                uv2 = self.project_to_image(pt2)
                if uv1 and uv2:
                    cv2.line(image, uv1, uv2, color_bgr, 2)

        draw_lines(self.nearest_neighbor_lines(self.blue_points), (255, 0, 0))     # Blue
        draw_lines(self.nearest_neighbor_lines(self.yellow_points), (0, 255, 255)) # Yellow
        draw_lines(self.nearest_neighbor_lines(self.midpoints), (0, 255, 0))       # Green

        overlay_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        overlay_msg.header = msg.header
        self.image_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeLineOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
