#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from std_msgs.msg import ColorRGBA
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from time import time
from dataclasses import dataclass, field
import tf2_ros

try:
    import open3d as o3d
except ImportError:
    o3d = None


def quat_to_rotmat(x, y, z, w):
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz), 2.0*(xz + wy)],
        [2.0*(xy + wz), 1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy), 2.0*(yz + wx), 1.0 - 2.0*(xx + yy)]
    ])


@dataclass
class TrackedCone:
    position: np.ndarray
    last_seen: float
    times_seen: int = 1
    last_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cone_id: int = 0
    color_state: str = "unknown"


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')

        # Parameters
        self.declare_parameter("input_topic", "/points_no_ground")
        self.declare_parameter("marker_topic", "/cones/tracks/markers")
        self.declare_parameter("poses_topic", "/cones/tracks/poses")
        self.declare_parameter("cluster_eps", 1.5)
        self.declare_parameter("min_points", 10)
        self.declare_parameter("max_points", 100)
        self.declare_parameter("voxel_leaf", 0.03)
        self.declare_parameter("height_min", 0.02)
        self.declare_parameter("height_max", 0.50)
        self.declare_parameter("radius_min", 0.02)
        self.declare_parameter("radius_max", 0.3)
        self.declare_parameter("association_radius", 0.5)
        self.declare_parameter("association_z_thresh", 0.1)
        self.declare_parameter("max_missed_sec", 0.75)
        self.declare_parameter("max_move_per_sec", 6.0)
        self.declare_parameter("tf_timeout_sec", 0.10)
        self.declare_parameter("tf_use_latest", False)
        self.declare_parameter("body_frame", "gps")  # Treat gps as base_link
        self.declare_parameter("target_frame", "odom")

        self.get_parameters()

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_points = self.create_subscription(PointCloud2, self.input_topic, self.points_callback, qos)
        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.pub_poses = self.create_publisher(PoseArray, self.poses_topic, 10)

        self.tracked_cones = {}
        self.next_cone_id = 0
        self.get_logger().info(f"Cone detection node initialised | visual_frame='{self.target_frame}' body_frame='{self.body_frame}'")

    def get_parameters(self):
        self.input_topic = self.get_parameter("input_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.poses_topic = self.get_parameter("poses_topic").value
        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.voxel_leaf = float(self.get_parameter("voxel_leaf").value)
        self.height_min = float(self.get_parameter("height_min").value)
        self.height_max = float(self.get_parameter("height_max").value)
        self.radius_min = float(self.get_parameter("radius_min").value)
        self.radius_max = float(self.get_parameter("radius_max").value)
        self.association_radius = float(self.get_parameter("association_radius").value)
        self.association_z_thresh = float(self.get_parameter("association_z_thresh").value)
        self.max_missed_sec = float(self.get_parameter("max_missed_sec").value)
        self.max_move_per_sec = float(self.get_parameter("max_move_per_sec").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.tf_use_latest = bool(self.get_parameter("tf_use_latest").value)
        self.body_frame = self.get_parameter("body_frame").value
        self.target_frame = self.get_parameter("target_frame").value

    # ---------- TF ----------
    def _lookup_transform(self, target_frame, source_frame, stamp):
        timeout = rclpy.duration.Duration(seconds=self.tf_timeout_sec)
        try:
            ts = Time.from_msg(stamp)
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, ts, timeout=timeout)
        except Exception:
            if not self.tf_use_latest:
                raise
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=timeout)
        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def transform_points_from_to(self, xyz, source, target, stamp):
        try:
            T = self._lookup_transform(target, source, stamp)
        except Exception as e:
            self.get_logger().warn(f"TF {source}->{target} failed: {e}")
            return np.empty((0, 3))
        pts_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
        return (T @ pts_h.T).T[:, :3]

    # ---------- Main ----------
    def points_callback(self, msg):
        now = time()

        # 1) Read incoming point cloud
        pts = list(pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')))
        if not pts:
            return
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=float)

        # 2) Transform LiDAR → odom for clustering
        xyz_odom = self.transform_points_from_to(xyz, msg.header.frame_id, "odom", msg.header.stamp)
        if xyz_odom.shape[0] == 0:
            return

        xyz_odom = self.voxel_downsample(xyz_odom)
        clusters = self.cluster_points(xyz_odom)

        # 3) Extract cones from clusters
        detections_odom = []
        for _, cpts in clusters.items():
            cone = self.extract_cone_from_cluster(cpts)
            if cone is not None:
                detections_odom.append(cone)
        if not detections_odom:
            return

        # 4) Classify by side in body frame (gps): y>0 left, y<0 right
        detections_classified = []
        for cone_odom in detections_odom:
            cone_body = self.transform_points_from_to(
                np.array([cone_odom]), "odom", self.body_frame, msg.header.stamp
            )
            if cone_body.shape[0] == 0:
                continue
            y_val = float(cone_body[0][1])  # side in body frame
            detections_classified.append((cone_odom, y_val))

        # 5) Track + Publish
        self.update_tracking_with_y(detections_classified, now)
        self.publish_markers_and_poses(msg.header, now)

    # ---------- Downsampling ----------
    def voxel_downsample(self, points):
        if self.voxel_leaf <= 0 or o3d is None:
            return points
        if points.shape[0] > self.max_points:
            points = points[np.random.choice(points.shape[0], self.max_points, replace=False)]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        return np.asarray(pcd.voxel_down_sample(self.voxel_leaf).points)

    # ---------- Clustering ----------
    def cluster_points(self, points):
        if o3d is None or points.shape[0] < self.min_points:
            return {}
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        labels = np.array(pcd.cluster_dbscan(eps=self.cluster_eps, min_points=self.min_points))
        clusters = {}
        for label in set(labels):
            if label == -1:
                continue
            cluster = points[labels == label]
            if self.min_points <= cluster.shape[0] <= self.max_points:
                clusters[int(label)] = cluster
        return clusters

    def extract_cone_from_cluster(self, cluster):
        if cluster.shape[0] < 1:
            return None
        z_min, z_max = cluster[:, 2].min(), cluster[:, 2].max()
        height = z_max - z_min
        if not (self.height_min <= height <= self.height_max):
            return None
        xy_r = np.linalg.norm(cluster[:, :2], axis=1)
        radius = np.std(xy_r)
        if not (self.radius_min <= radius <= self.radius_max):
            return None
        centroid_xy = cluster[:, :2].mean(axis=0)
        low_mask = np.isclose(cluster[:, 2], z_min, atol=0.02)
        base_candidates = cluster[low_mask] if np.any(low_mask) else cluster
        base_pt = base_candidates[np.argmin(np.linalg.norm(base_candidates[:, :2] - centroid_xy, axis=1))]
        return base_pt

    # ---------- Tracking (first colour wins, by body-frame side) ----------
    def update_tracking_with_y(self, detections, now):
        associated = set()
        z_thresh = getattr(self, "association_z_thresh", 0.1)

        for cone_id, cone in list(self.tracked_cones.items()):
            if not detections:
                continue

            positions = np.array([d[0] for d in detections])
            distances_xy = np.linalg.norm(positions[:, :2] - cone.position[:2], axis=1)
            z_diffs = np.abs(positions[:, 2] - cone.position[2])

            mask_valid = (distances_xy <= self.association_radius) & (z_diffs <= z_thresh)
            if not np.any(mask_valid):
                continue

            i = int(np.argmin(distances_xy + z_diffs))
            dt = now - cone.last_seen
            speed = distances_xy[i] / dt if dt > 0 else 0

            if speed <= self.max_move_per_sec:
                cone.last_position = cone.position.copy()
                cone.position = 0.7 * cone.position + 0.3 * positions[i]
                cone.last_seen = now
                cone.times_seen += 1

                y_val = detections[i][1]
                # body frame rule: y>0 left, y<0 right
                if cone.color_state == "unknown":
                    if y_val > 0.0:
                        cone.color_state = "left"
                    elif y_val < 0.0:
                        cone.color_state = "right"

                associated.add(i)

        for i, (pos, y_val) in enumerate(detections):
            if i not in associated:
                new_cone = TrackedCone(pos, now, cone_id=self.next_cone_id)
                if y_val > 0.0:
                    new_cone.color_state = "left"
                elif y_val < 0.0:
                    new_cone.color_state = "right"
                else:
                    new_cone.color_state = "unknown"
                self.tracked_cones[self.next_cone_id] = new_cone
                self.next_cone_id += 1

    # ---------- Publish ----------
    def publish_markers_and_poses(self, header, now):
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = self.target_frame

        # De-duplicate overlapping cones before drawing
        unique_cones = {}
        for cid, cone in self.tracked_cones.items():
            duplicate = False
            for ucid, ucone in unique_cones.items():
                if np.linalg.norm(cone.position[:2] - ucone.position[:2]) < 0.1 and abs(cone.position[2] - ucone.position[2]) < 0.05:
                    duplicate = True
                    break
            if not duplicate:
                unique_cones[cid] = cone
        self.tracked_cones = unique_cones

        for cone_id, cone in self.tracked_cones.items():
            if cone.color_state == "left":
                color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # orange (right)
            elif cone.color_state == "right":
                color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)  # blue (left)
            else:
                color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)  # unknown

            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.target_frame
            marker.id = int(cone_id * 2)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone.position[0])
            marker.pose.position.y = float(cone.position[1])
            marker.pose.position.z = float(cone.position[2] + 0.4)
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = float(2 * self.radius_max)
            marker.scale.z = float(self.height_max)
            marker.color = color
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

            txt = Marker()
            txt.header = header
            txt.header.frame_id = self.target_frame
            txt.id = int(cone_id * 2 + 1)
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(cone.position[0])
            txt.pose.position.y = float(cone.position[1])
            txt.pose.position.z = float(cone.position[2] + 1.0)
            txt.scale.z = 0.3
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = f"{cone_id} ({cone.color_state})"
            txt.lifetime.sec = 1
            marker_array.markers.append(txt)

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = cone.position
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.pub_markers.publish(marker_array)
        self.pub_poses.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
