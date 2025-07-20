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

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException

def pos_key(arr, precision=0.15):
    # For map persistence; bins positions to 15cm cubes for robustness to noise
    return tuple((np.array(arr[:3]) / precision).round().astype(int))

class LidarConeMapper(Node):
    def __init__(self):
        super().__init__('lidar_cone_mapper')

        self.ground_distance_threshold = 0.1
        self.cluster_tol     = 0.5
        self.min_cluster_pts = 3
        self.max_cluster_pts = 2000
        self.xy_merge_tol    = 0.20
        self.tracking_tol    = 0.5

        self.CLASS_ID_TO_COLOR = {
            0: (0.0, 0.0, 1.0),   # blue
            1: (1.0, 0.5, 0.0),   # orange
            2: (1.0, 0.5, 0.0),   # orange
            4: (1.0, 1.0, 0.0),   # yellow
        }
        self.default_color = (1.0, 0.0, 0.0)   # red if no class match

        self.world_frame = 'map'
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_caminfo    = None
        self.camera_frame      = None
        self.latest_detections = []

        # Persistent map: key = (x_bin, y_bin), value = np.array([x,y,z])
        self.blue_cone_map   = dict()
        self.yellow_cone_map = dict()
        self.orange_cone_map = dict()

        self.create_subscription(PointCloud2, '/lidar/points', self.cloud_cb, 5)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.caminfo_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections', self.yolo_cb, 10)

        self.filtered_pub        = self.create_publisher(PointCloud2,   '/filtered_cloud',         10)
        self.blue_marker_pub     = self.create_publisher(MarkerArray,   '/cone_landmarks_blue',    10)
        self.yellow_marker_pub   = self.create_publisher(MarkerArray,   '/cone_landmarks_yellow',  10)
        self.orange_marker_pub   = self.create_publisher(MarkerArray,   '/cone_landmarks_orange',  10)
        self.unidentified_marker_pub = self.create_publisher(MarkerArray, '/cone_landmarks_unidentified', 10)

        self.get_logger().info('[LidarConeMapper] ready.')

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
                if all(hasattr(bbox, a) for a in ('xmin','xmax','ymin','ymax')):
                    u_center = 0.5 * (bbox.xmin + bbox.xmax)
                    v_center = 0.5 * (bbox.ymin + bbox.ymax)
                else:
                    continue
            if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                w, h = float(bbox.size_x), float(bbox.size_y)
            else:
                w, h = float(bbox.xmax - bbox.xmin), float(bbox.ymax - bbox.ymin)
            cid = -1; score = 0.0
            if det.results:
                hyp = det.results[0].hypothesis
                try:
                    cid = int(hyp.class_id)
                    score = float(hyp.score)
                except:
                    pass
            boxes.append({
                'u0': u_center - w/2.0, 'u1': u_center + w/2.0,
                'v0': v_center - h/2.0, 'v1': v_center + h/2.0,
                'class_id': cid,
                'score': score
            })
        self.latest_detections = boxes

    def cloud_cb(self, msg: PointCloud2):
        all_pts = np.array([
            (x, y, z) for (x, y, z) in point_cloud2.read_points(
                msg, field_names=('x','y','z'), skip_nans=False)
        ], dtype=np.float32)
        if all_pts.size == 0:
            return
        pts = all_pts[np.isfinite(all_pts).all(axis=1)]
        if len(pts) < self.min_cluster_pts:
            return

        cloud = pcl.PointCloud(pts)
        seg   = cloud.make_segmenter_normals(ksearch=50)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ground_distance_threshold)
        inliers, _ = seg.segment()
        if not inliers:
            return
        mask = np.ones(len(pts), bool)
        mask[inliers] = False
        pts_nog = pts[mask]
        if len(pts_nog) < self.min_cluster_pts:
            return

        filtered = point_cloud2.create_cloud_xyz32(msg.header, pts_nog.tolist())
        self.filtered_pub.publish(filtered)

        tree = pcl.PointCloud(pts_nog).make_kdtree()
        ec   = pcl.PointCloud(pts_nog).make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tol)
        ec.set_MinClusterSize(self.min_cluster_pts)
        ec.set_MaxClusterSize(self.max_cluster_pts)
        ec.set_SearchMethod(tree)
        clusters = ec.Extract()
        if not clusters:
            return

        raw_cents = [pts_nog[idxs].mean(axis=0) for idxs in clusters]
        unique_cents = []
        for c in raw_cents:
            if not any(np.linalg.norm(c[:2] - uc[:2]) < self.xy_merge_tol for uc in unique_cents):
                unique_cents.append(c)
        if not unique_cents:
            return

        t_now = self.get_clock().now().to_msg()
        try:
            tf_lidar_map = self.tf_buffer.lookup_transform(
                self.world_frame, msg.header.frame_id,
                Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lidar→map failed: {e}")
            return

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

        blue_pts, yellow_pts, orange_pts = [], [], []
        override_colors = [None]*len(unique_cents)

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

        # --- MAP ACCUMULATION (persistent storage in map frame) ---
        for i, c in enumerate(unique_cents):
            col = override_colors[i] or self.default_color

            # Transform each to the map frame (persistent world frame)
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
            if col == self.CLASS_ID_TO_COLOR[0]:
                # Blue
                if key not in self.blue_cone_map:
                    self.blue_cone_map[key] = pos_map
            elif col == self.CLASS_ID_TO_COLOR[4]:
                # Yellow
                if key not in self.yellow_cone_map:
                    self.yellow_cone_map[key] = pos_map
            elif col == self.CLASS_ID_TO_COLOR[1] or col == self.CLASS_ID_TO_COLOR[2]:
                if key not in self.orange_cone_map:
                    self.orange_cone_map[key] = pos_map

        # --- MARKER PUB from accumulated map ---
        def map_to_markerarray(cone_map, color, ns, base_id=0):
            ma = MarkerArray()
            for i, pos in enumerate(cone_map.values()):
                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp    = t_now
                m.ns              = ns
                m.id              = base_id + i
                m.type            = Marker.SPHERE
                m.action          = Marker.ADD
                m.pose.position.x = float(pos[0])
                m.pose.position.y = float(pos[1])
                m.pose.position.z = float(pos[2])
                s = 0.30
                m.scale.x = s; m.scale.y = s; m.scale.z = s
                r,g,b = color
                m.color.r, m.color.g, m.color.b, m.color.a = (r,g,b,0.8)
                ma.markers.append(m)
            return ma

        self.blue_marker_pub.publish(map_to_markerarray(self.blue_cone_map,   self.CLASS_ID_TO_COLOR[0], 'cones_blue'))
        self.yellow_marker_pub.publish(map_to_markerarray(self.yellow_cone_map, self.CLASS_ID_TO_COLOR[4], 'cones_yellow'))
        self.orange_marker_pub.publish(map_to_markerarray(self.orange_cone_map, self.CLASS_ID_TO_COLOR[1], 'cones_orange'))

        # --- unidentified clusters (transient, non-persistent, as before) ---
        unidentified_ma = MarkerArray()
        for i, c in enumerate(unique_cents):
            col = override_colors[i]
            if col is None:
                # Transform to map
                p_map = PointStamped()
                p_map.header.frame_id = msg.header.frame_id
                p_map.header.stamp    = t_now
                p_map.point.x, p_map.point.y, p_map.point.z = c.tolist()
                try:
                    p_m = do_transform_point(p_map, tf_lidar_map)
                except TransformException:
                    continue
                pos = np.array([p_m.point.x, p_m.point.y, p_m.point.z])
                m = Marker()
                m.header.frame_id = self.world_frame
                m.header.stamp    = t_now
                m.ns              = 'cones_unidentified'
                m.id              = 10000 + i  # large offset to avoid cone ID conflict
                m.type            = Marker.SPHERE
                m.action          = Marker.ADD
                m.pose.position.x = float(pos[0])
                m.pose.position.y = float(pos[1])
                m.pose.position.z = float(pos[2])
                s = 0.33
                m.scale.x = s; m.scale.y = s; m.scale.z = s
                m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.0, 0.0, 0.7)
                unidentified_ma.markers.append(m)
        self.unidentified_marker_pub.publish(unidentified_ma)

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
