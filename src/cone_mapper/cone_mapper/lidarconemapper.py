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


class LidarConeMapper(Node):
    def __init__(self):
        super().__init__('lidar_cone_mapper')

        # ——— Parameters ———
        self.ground_distance_threshold = 0.02  # RANSAC plane threshold (m)
        self.cluster_tol     = 0.20           # Euclidean clustering tolerance (m)
        self.min_cluster_pts = 5             # Min points per cluster
        self.max_cluster_pts = 2000           # Max points per cluster
        self.xy_merge_tol    = 0.20           # Merge centroids within XY tol (m)

        # YOLO class → marker color
        self.CLASS_ID_TO_COLOR = {
            0: (0.0, 0.0, 1.0),   # blue
            1: (1.0, 0.5, 0.0),   # orange
            2: (1.0, 1.0, 0.0),   # yellow
            4: (1.0, 1.0, 0.0),   # yellow
        }
        self.default_color = (1.0, 0.0, 0.0)   # red if no match

        # TF setup
        self.world_frame = 'map'
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # storage for latest camera info & detections
        self.latest_caminfo    = None
        self.camera_frame      = None
        self.latest_detections = []  # list of dicts: u0,u1,v0,v1,class_id,score

        # ——— Subscriptions ———
        self.create_subscription(PointCloud2,
                                 '/lidar/points',
                                 self.cloud_cb,
                                 5)
        self.create_subscription(CameraInfo,
                                 '/camera/camera_info',
                                 self.caminfo_cb,
                                 10)
        self.create_subscription(Detection2DArray,
                                 '/yolo/detections',
                                 self.yolo_cb,
                                 10)

        # ——— Publishers ———
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            '/filtered_cloud',
            10)
        self.marker_pub   = self.create_publisher(
            MarkerArray,
            '/cone_landmarks',
            10)

        self.get_logger().info('[LidarConeMapper] ready.')

    def caminfo_cb(self, msg: CameraInfo):
        self.latest_caminfo = msg
        self.camera_frame   = msg.header.frame_id

    def yolo_cb(self, msg: Detection2DArray):
        boxes = []
        for det in msg.detections:
            bbox = det.bbox

            # 1) Extract center
            u_center = v_center = None
            c = getattr(bbox, 'center', None)
            if c is not None:
                if hasattr(c, 'x') and hasattr(c, 'y'):
                    u_center = float(c.x)
                    v_center = float(c.y)
                elif hasattr(c, 'position'):
                    u_center = float(c.position.x)
                    v_center = float(c.position.y)

            # 2) Fallback to xmin/xmax if needed
            if u_center is None or v_center is None:
                if all(hasattr(bbox, a) for a in ('xmin','xmax','ymin','ymax')):
                    u_center = 0.5 * (float(bbox.xmin) + float(bbox.xmax))
                    v_center = 0.5 * (float(bbox.ymin) + float(bbox.ymax))
                else:
                    continue  # skip if no valid center

            # 3) Extract width/height
            if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                width  = float(bbox.size_x)
                height = float(bbox.size_y)
            else:
                width  = float(bbox.xmax - bbox.xmin)
                height = float(bbox.ymax - bbox.ymin)

            u0 = u_center - width/2.0
            u1 = u_center + width/2.0
            v0 = v_center - height/2.0
            v1 = v_center + height/2.0

            # 4) class_id & score
            cid = None
            score = 0.0
            if det.results:
                hyp = det.results[0].hypothesis
                raw = getattr(hyp, 'class_id', None)
                try:
                    cid = int(raw)
                except:
                    cid = None
                try:
                    score = float(hyp.score)
                except:
                    score = 0.0

            boxes.append({
                'u0': u0, 'u1': u1,
                'v0': v0, 'v1': v1,
                'class_id': cid,
                'score': score
            })

        self.latest_detections = boxes

    def cloud_cb(self, msg: PointCloud2):
        # 1) Read all points (including NaNs), then drop invalid
        all_pts = np.array([
            (x, y, z)
            for (x, y, z) in point_cloud2.read_points(
                msg,
                field_names=('x','y','z'),
                skip_nans=False
            )
        ], dtype=np.float32)
        if all_pts.size == 0:
            return
        finite_mask = np.isfinite(all_pts).all(axis=1)
        pts = all_pts[finite_mask]
        if pts.shape[0] < self.min_cluster_pts:
            return

        # 2) RANSAC plane → remove ground
        cloud = pcl.PointCloud(pts)
        seg   = cloud.make_segmenter_normals(ksearch=50)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self.ground_distance_threshold)
        inliers, _ = seg.segment()
        if not inliers:
            return
        mask_ng = np.ones(pts.shape[0], dtype=bool)
        mask_ng[inliers] = False
        pts_nog = pts[mask_ng]
        if pts_nog.shape[0] < self.min_cluster_pts:
            return

        # 3) Publish the filtered (no-ground) cloud
        filtered_msg = point_cloud2.create_cloud_xyz32(
            msg.header,
            pts_nog.tolist()
        )
        self.filtered_pub.publish(filtered_msg)

        # 4) Euclidean clustering
        tree = pcl.PointCloud(pts_nog).make_kdtree()
        ec   = pcl.PointCloud(pts_nog).make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(self.cluster_tol)
        ec.set_MinClusterSize(self.min_cluster_pts)
        ec.set_MaxClusterSize(self.max_cluster_pts)
        ec.set_SearchMethod(tree)
        clusters = ec.Extract()
        if not clusters:
            return

        # 5) Compute centroids & vertical dedupe by XY
        raw_cents = [pts_nog[idxs].mean(axis=0) for idxs in clusters]
        unique_cents = []
        for c in raw_cents:
            xy = c[:2]
            if not any(np.linalg.norm(xy - uc[:2]) < self.xy_merge_tol
                       for uc in unique_cents):
                unique_cents.append(c)
        if not unique_cents:
            return

        # 6) Prepare timestamps & look up TFs
        now = self.get_clock().now()
        t_msg = now.to_msg()

        # LiDAR → map for marker placement
        try:
            tf_lidar_map = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lidar→map failed: {e}")
            return

        # LiDAR → camera for projection
        cam_tf = None
        if self.latest_caminfo and self.camera_frame:
            try:
                cam_tf = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.1)
                )
                K = self.latest_caminfo.k
                fx, fy, cx, cy = K[0], K[4], K[2], K[5]
            except TransformException as e:
                self.get_logger().warn(f"TF lidar→cam failed: {e}")
                cam_tf = None

        # 7) Project & match each centroid → assign override colors
        override_color = [None] * len(unique_cents)
        if cam_tf and self.latest_detections:
            for i, c in enumerate(unique_cents):
                p_l = PointStamped()
                p_l.header.frame_id = msg.header.frame_id
                p_l.header.stamp    = t_msg
                p_l.point.x, p_l.point.y, p_l.point.z = c.tolist()

                try:
                    p_c = do_transform_point(p_l, cam_tf)
                except TransformException:
                    continue

                Z = p_c.point.z
                if Z <= 0.0:
                    continue
                u = (fx * p_c.point.x / Z) + cx
                v = (fy * p_c.point.y / Z) + cy

                best_det = None
                best_score = -1.0
                for det in self.latest_detections:
                    if det['u0'] <= u <= det['u1'] and det['v0'] <= v <= det['v1']:
                        if det['score'] > best_score:
                            best_score = det['score']
                            best_det   = det
                if best_det and best_det['class_id'] in self.CLASS_ID_TO_COLOR:
                    override_color[i] = self.CLASS_ID_TO_COLOR[best_det['class_id']]

        # 8) Build & publish markers
        ma = MarkerArray()
        for idx, cent in enumerate(unique_cents):
            p_l = PointStamped()
            p_l.header.frame_id = msg.header.frame_id
            p_l.header.stamp    = t_msg
            p_l.point.x, p_l.point.y, p_l.point.z = cent.tolist()

            try:
                p_m = do_transform_point(p_l, tf_lidar_map)
            except TransformException:
                continue

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp    = t_msg
            m.ns              = 'cones'
            m.id              = idx
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position   = p_m.point
            s = 0.30
            m.scale.x = s; m.scale.y = s; m.scale.z = s

            color = override_color[idx] or self.default_color
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.8

            ma.markers.append(m)

        self.marker_pub.publish(ma)


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
