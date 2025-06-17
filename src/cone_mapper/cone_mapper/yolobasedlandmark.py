#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class HybridConeLocalizer(Node):
    def __init__(self):
        super().__init__('hybrid_cone_localizer')

        # Parameters
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('alpha', 0.5)
        self.declare_parameter('cluster_search_radius', 0.2)
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.alpha       = self.get_parameter('alpha').get_parameter_value().double_value
        self.search_radius = self.get_parameter('cluster_search_radius').get_parameter_value().double_value

        # Cone dimensions & colors by class_id
        self.radius_map = {'0': 0.14, '4': 0.14, '1': 0.20, '2': 0.20}
        self.height_map = {'0': 0.325, '4': 0.325, '1': 0.505, '2': 0.505}
        self.color_map  = {'0': (0,0,1,0.8), '4': (1,1,0,0.8),
                           '1': (1,0.5,0,0.8), '2': (1,0.5,0,0.8)}

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.caminfo_cb, 10)

        # Subscribers: YOLO + ground-removed pointcloud
        self.pc_sub   = Subscriber(self, PointCloud2,      '/cloud_no_ground_ransac')
        self.yolo_sub = Subscriber(self, Detection2DArray, '/yolo/detections')
        self.sync     = ApproximateTimeSynchronizer(
                            [self.pc_sub, self.yolo_sub],
                            queue_size=10, slop=0.1)
        self.sync.registerCallback(self.callback)

        # Publishers
        self.pose_pub   = self.create_publisher(PoseStamped,  '/cone_pose',    10)
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_markers', 10)

        self.get_logger().info('[HybridConeLocalizer] initialized')

    def caminfo_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics fx={self.fx}, fy={self.fy}, '
                f'cx={self.cx}, cy={self.cy}'
            )

    def callback(self, pc_msg: PointCloud2, det_msg: Detection2DArray):
        if self.fx is None:
            return

        # — FIXED conversion from structured dtype to float32 Nx3 array —
        pts = list(point_cloud2.read_points(
            pc_msg, field_names=('x','y','z'), skip_nans=True))
        # each pt is a tuple (x,y,z); stack into a regular float array:
        if not pts:
            return
        cloud = np.array([(p[0], p[1], p[2]) for p in pts],
                         dtype=np.float32)

        markers = MarkerArray()

        for idx, det in enumerate(det_msg.detections):
            if not det.results:
                continue
            cid   = str(det.results[0].hypothesis.class_id)
            score = det.results[0].hypothesis.score
            if score < 0.6:
                continue

            R = self.radius_map.get(cid)
            H = self.height_map.get(cid)
            col = self.color_map.get(cid, (0.5,0.5,0.5,0.8))

            # pixel box size
            w_px = getattr(det.bbox, 'size_x',
                           det.bbox.xmax - det.bbox.xmin)
            h_px = getattr(det.bbox, 'size_y',
                           det.bbox.ymax - det.bbox.ymin)

            # depth estimates & blend
            Zw = 2 * self.fx * R / w_px
            Zh =     self.fx * H / h_px
            Z  = self.alpha * Zw + (1 - self.alpha) * Zh

            # pixel center
            cen = det.bbox.center
            u = getattr(cen, 'x', cen.position.x)
            v = getattr(cen, 'y', cen.position.y)

            # back-project to camera frame
            X_cam = (u - self.cx) * Z / self.fx
            Y_cam = (v - self.cy) * Z / self.fy

            p_cam = PointStamped()
            p_cam.header = pc_msg.header
            p_cam.point.x = X_cam
            p_cam.point.y = -Y_cam
            p_cam.point.z = Z

            # transform into world_frame
            try:
                tf = self.tf_buffer.lookup_transform(
                        self.world_frame,
                        pc_msg.header.frame_id,
                        Time())
                p_map = do_transform_point(p_cam, tf)
            except Exception as e:
                self.get_logger().warn(f'TF error: {e}')
                continue

            xw, yw, zw = (p_map.point.x, p_map.point.y, p_map.point.z)

            # refine via nearest cluster in cloud XY
            dists = np.linalg.norm(
                        cloud[:,:2] - np.array([xw, yw]), axis=1)
            mask = dists < self.search_radius
            if np.any(mask):
                centroid = cloud[mask].mean(axis=0)
                xw, yw, zw = centroid.tolist()

            # publish PoseStamped
            ps = PoseStamped()
            ps.header = det.header
            ps.pose.position.x = xw
            ps.pose.position.y = yw
            ps.pose.position.z = zw
            ps.pose.orientation.w = 1.0
            self.pose_pub.publish(ps)

            # add Marker
            m = Marker()
            m.header = ps.header
            m.ns     = f'cone_{cid}'
            m.id     = idx
            m.type   = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose   = ps.pose
            m.scale.x = m.scale.y = 2 * R
            m.scale.z = H
            r, g, b, a = col
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = HybridConeLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
