#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import numpy as np
import pcl
import time

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


def pos_key(arr, precision=0.15):
    # For map persistence; bins positions to 15cm cubes for robustness to noise
    return tuple((np.array(arr[:3]) / precision).round().astype(int))


class LidarConeMapper(Node):
    def __init__(self):
        super().__init__('lidar_cone_mapper')

        # -------------------- Filtering / clustering params --------------------
        # Simple Z height filter in the incoming cloud frame (msg.header.frame_id)
        self.declare_parameter('z_min', -0.9)   # meters
        self.declare_parameter('z_max', 1.20)   # meters
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.cluster_tol     = 0.5
        self.min_cluster_pts = 3
        self.max_cluster_pts = 2000
        self.xy_merge_tol    = 0.20
        self.tracking_tol    = 0.5

        # Time to keep markers visible after last detection (seconds)
        self.marker_timeout = 0.1 # Adjust as needed

        self.CLASS_ID_TO_COLOR = {
            0: (0.0, 0.0, 1.0),   # blue
            1: (1.0, 0.5, 0.0),   # orange
            2: (1.0, 0.5, 0.0),   # orange
            4: (1.0, 1.0, 0.0),   # yellow
        }
        self.default_color = (1.0, 0.0, 0.0)   # red if no class match

        self.world_frame = 'quanergy'
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_caminfo    = None
        self.camera_frame      = None
        self.latest_detections = []

        # Time-based marker storage
        self.timed_cone_markers = {}

        # Track marker IDs for cleanup (by category)
        self.current_marker_ids = {'blue': [], 'yellow': [], 'orange': [], 'unidentified': []}

        # Subscriptions
        # self.create_subscription(PointCloud2, '/lidar/points', self.cloud_cb, 5)
        self.create_subscription(PointCloud2, '/quanergy/points', self.cloud_cb, 5)
        # self.create_subscription(CameraInfo, '/camera/camera_info', self.caminfo_cb, 10)
        self.create_subscription(CameraInfo, '/yolo/camera_info', self.caminfo_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.yolo_cb, 10)

        # Publishers
        self.filtered_pub            = self.create_publisher(PointCloud2,   '/filtered_cloud',               10)
        self.blue_marker_pub         = self.create_publisher(MarkerArray,   '/cone_landmarks_blue',          10)
        self.yellow_marker_pub       = self.create_publisher(MarkerArray,   '/cone_landmarks_yellow',        10)
        self.orange_marker_pub       = self.create_publisher(MarkerArray,   '/cone_landmarks_orange',        10)
        self.unidentified_marker_pub = self.create_publisher(MarkerArray,   '/cone_landmarks_unidentified',  10)

        self.get_logger().info(f'[LidarConeMapper] ready. Z filter: [{self.z_min:.2f}, {self.z_max:.2f}] m; Marker timeout: {self.marker_timeout}s')

    # -------------------- Callbacks --------------------
    def caminfo_cb(self, msg: CameraInfo):
        self.latest_caminfo = msg
        self.camera_frame   = msg.header.frame_id

    def yolo_cb(self, msg: Detection2DArray):
        boxes = []
        for det in msg.detections:
            bbox = det.bbox
            u_center = v_center = None
            c = getattr(bbox, 'center', None)
            if c:
                if hasattr(c, 'x') and hasattr(c, 'y'):
                    u_center, v_center = float(c.x), float(c.y)
                elif hasattr(c, 'position'):
                    u_center, v_center = float(c.position.x), float(c.position.y)
            if u_center is None or v_center is None:
                if all(hasattr(bbox, a) for a in ('xmin', 'xmax', 'ymin', 'ymax')):
                    u_center = 0.5 * (bbox.xmin + bbox.xmax)
                    v_center = 0.5 * (bbox.ymin + bbox.ymax)
                else:
                    continue
            if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                w, h = float(bbox.size_x), float(bbox.size_y)
            else:
                w, h = float(bbox.xmax - bbox.xmin), float(bbox.ymax - bbox.ymin)
            cid = -1
            score = 0.0
            if det.results:
                hyp = det.results[0].hypothesis
                try:
                    cid = int(hyp.class_id)
                    score = float(hyp.score)
                except Exception:
                    pass
            boxes.append({
                'u0': u_center - w/2.0, 'u1': u_center + w/2.0,
                'v0': v_center - h/2.0, 'v1': v_center + h/2.0,
                'class_id': cid,
                'score': score
            })
        self.latest_detections = boxes

    # -------------------- Marker utils --------------------
    def cleanup_expired_markers(self, current_time):
        expired_keys = []
        for key, marker_data in self.timed_cone_markers.items():
            if current_time - marker_data['last_seen'] > self.marker_timeout:
                expired_keys.append(key)
        for key in expired_keys:
            del self.timed_cone_markers[key]

    def get_color_category(self, color):
        if color == self.CLASS_ID_TO_COLOR[0]:
            return 'blue'
        elif color == self.CLASS_ID_TO_COLOR[4]:
            return 'yellow'
        elif color == self.CLASS_ID_TO_COLOR[1] or color == self.CLASS_ID_TO_COLOR[2]:
            return 'orange'
        else:
            return 'unidentified'

    def publish_markers_by_category(self, t_now):
        categories = {'blue': [], 'yellow': [], 'orange': [], 'unidentified': []}
        for marker_data in self.timed_cone_markers.values():
            category = self.get_color_category(marker_data['color'])
            categories[category].append(marker_data)

        for cat in self.current_marker_ids:
            self.current_marker_ids[cat] = []

        def publish_category_markers(positions_data, color, ns, publisher, base_id=0):
            ma = MarkerArray()
            marker_ids = []
            for i, marker_data in enumerate(positions_data):
                marker_id = base_id + i
                marker_ids.append(marker_id)
                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp = t_now
                m.ns = ns
                m.id = marker_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(marker_data['position'][0])
                m.pose.position.y = float(marker_data['position'][1])
                m.pose.position.z = float(marker_data['position'][2])

                age = time.time() - marker_data['last_seen']
                age_factor = max(0.7, 1.0 - (age / self.marker_timeout) * 0.3)
                s = 0.30 * age_factor
                m.scale.x = s; m.scale.y = s; m.scale.z = s

                alpha = max(0.5, 1.0 - (age / self.marker_timeout) * 0.3)
                r, g, b = color
                m.color.r, m.color.g, m.color.b, m.color.a = (r, g, b, alpha)
                ma.markers.append(m)

            publisher.publish(ma)
            return marker_ids

        self.current_marker_ids['blue'] = publish_category_markers(
            categories['blue'], self.CLASS_ID_TO_COLOR[0], 'cones_blue', self.blue_marker_pub, 0)
        self.current_marker_ids['yellow'] = publish_category_markers(
            categories['yellow'], self.CLASS_ID_TO_COLOR[4], 'cones_yellow', self.yellow_marker_pub, 0)
        self.current_marker_ids['orange'] = publish_category_markers(
            categories['orange'], self.CLASS_ID_TO_COLOR[1], 'cones_orange', self.orange_marker_pub, 0)
        self.current_marker_ids['unidentified'] = publish_category_markers(
            categories['unidentified'], (1.0, 0.0, 0.0), 'cones_unidentified', self.unidentified_marker_pub, 10000)

    # -------------------- Main cloud callback --------------------
    def cloud_cb(self, msg: PointCloud2):
        current_time = time.time()

        # Read cloud to Nx3
        all_pts = np.array([
            (x, y, z) for (x, y, z) in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=False)
        ], dtype=np.float32)

        if all_pts.size == 0:
            self.cleanup_expired_markers(current_time)
            t_now = self.get_clock().now().to_msg()
            self.publish_markers_by_category(t_now)
            return

        # Drop NaN/Inf rows
        pts = all_pts[np.isfinite(all_pts).all(axis=1)]
        if len(pts) < self.min_cluster_pts:
            self.cleanup_expired_markers(current_time)
            t_now = self.get_clock().now().to_msg()
            self.publish_markers_by_category(t_now)
            return

        # -------------------- Z HEIGHT FILTER (replaces RANSAC) --------------------
        # Keep points where z_min <= z <= z_max in the incoming cloud frame
        z = pts[:, 2]
        z_mask = (z >= self.z_min) & (z <= self.z_max)
        pts_nog = pts[z_mask]

        if len(pts_nog) < self.min_cluster_pts:
            # Still update persistence and republish any surviving markers
            self.cleanup_expired_markers(current_time)
            t_now = self.get_clock().now().to_msg()
            self.publish_markers_by_category(t_now)
            return

        # Publish filtered cloud
        filtered = point_cloud2.create_cloud_xyz32(msg.header, pts_nog.tolist())
        self.filtered_pub.publish(filtered)

        # -------------------- Clustering --------------------
        tree = pcl.PointCloud(pts_nog).make_kdtree()
        ec   = pcl.PointCloud(pts_nog).make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tol)
        ec.set_MinClusterSize(self.min_cluster_pts)
        ec.set_MaxClusterSize(self.max_cluster_pts)
        ec.set_SearchMethod(tree)
        clusters = ec.Extract()

        if not clusters:
            self.cleanup_expired_markers(current_time)
            t_now = self.get_clock().now().to_msg()
            self.publish_markers_by_category(t_now)
            return

        # Centroids
        raw_cents = [pts_nog[idxs].mean(axis=0) for idxs in clusters]

        # Merge near-duplicate centroids by XY distance
        unique_cents = []
        for c in raw_cents:
            if not any(np.linalg.norm(c[:2] - uc[:2]) < self.xy_merge_tol for uc in unique_cents):
                unique_cents.append(c)

        if not unique_cents:
            self.cleanup_expired_markers(current_time)
            t_now = self.get_clock().now().to_msg()
            self.publish_markers_by_category(t_now)
            return

        t_now = self.get_clock().now().to_msg()
        try:
            tf_lidar_map = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id,
                Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lidar→map failed: {e}")
            self.cleanup_expired_markers(current_time)
            self.publish_markers_by_category(t_now)
            return

        # Camera projection (optional; for coloring only)
        cam_tf = None
        if self.latest_caminfo and self.camera_frame:
            try:
                cam_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame, msg.header.frame_id,
                    Time(), timeout=Duration(seconds=0.1)
                )
                K = self.latest_caminfo.k
                fx, fy, cx, cy = K[0], K[4], K[2], K[5]
            except TransformException:
                cam_tf = None

        # Clean up expired markers first
        self.cleanup_expired_markers(current_time)

        # Determine override colors from YOLO projections (no filtering, only color)
        override_colors = [None] * len(unique_cents)
        if cam_tf:
            for i, c in enumerate(unique_cents):
                p_cam = PointStamped()
                p_cam.header.frame_id = msg.header.frame_id
                p_cam.header.stamp    = t_now
                p_cam.point.x, p_cam.point.y, p_cam.point.z = c.tolist()
                try:
                    p_proj = do_transform_point(p_cam, cam_tf)
                except TransformException:
                    continue
                if p_proj.point.z <= 0:
                    continue
                u = fx * p_proj.point.x / p_proj.point.z + cx
                v = fy * p_proj.point.y / p_proj.point.z + cy
                best = max(
                    (d for d in self.latest_detections
                     if d['u0'] <= u <= d['u1'] and d['v0'] <= v <= d['v1']),
                    default=None, key=lambda d: d['score']
                )
                if best and best['class_id'] in self.CLASS_ID_TO_COLOR:
                    override_colors[i] = self.CLASS_ID_TO_COLOR[best['class_id']]

        # Update timed markers in map/world frame
        for i, c in enumerate(unique_cents):
            col = override_colors[i] or self.default_color

            p_map = PointStamped()
            p_map.header.frame_id = msg.header.frame_id
            p_map.header.stamp    = t_now
            p_map.point.x, p_map.point.y, p_map.point.z = c.tolist()
            try:
                p_m = do_transform_point(p_map, tf_lidar_map)
            except TransformException:
                continue

            pos_map = np.array([p_m.point.x, p_m.point.y, p_m.point.z])
            key = pos_key(pos_map)
            self.timed_cone_markers[key] = {
                'position': pos_map,
                'color': col,
                'last_seen': current_time
            }

        # Publish markers
        self.publish_markers_by_category(t_now)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
