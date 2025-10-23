

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from std_msgs.msg import ColorRGBA
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from time import time
from dataclasses import dataclass, field

# TF2
import tf2_ros

try:
    import open3d as o3d
except ImportError:
    o3d = None


def quat_to_rotmat(x, y, z, w):
    """Convert quaternion to 3x3 rotation matrix (ROS xyzw)."""
    # Normalise to avoid scaling issues
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1.0 - 2.0*(yy + zz),     2.0*(xy - wz),         2.0*(xz + wy)],
        [    2.0*(xy + wz),   1.0 - 2.0*(xx + zz),       2.0*(yz - wx)],
        [    2.0*(xz - wy),       2.0*(yz + wx),     1.0 - 2.0*(xx + yy)],
    ], dtype=float)


@dataclass
class TrackedCone:
    """Represents a tracked cone with temporal information."""
    position: np.ndarray
    last_seen: float
    times_seen: int = 1
    last_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cone_id: int = 0


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')

        # Declare parameters
        self.declare_parameter("input_topic", "/points_no_ground")
        self.declare_parameter("marker_topic", "/cones/tracks/markers")
        self.declare_parameter("poses_topic", "/cones/tracks/poses")

        # Clustering parameters
        self.declare_parameter("approx_layout", True)
        self.declare_parameter("cluster_eps", 1.5)
        self.declare_parameter("min_points", 2)
        self.declare_parameter("max_points", 2000)
        self.declare_parameter("voxel_leaf", 0.03)

        # Cone geometry parameters
        self.declare_parameter("height_min", 0.05)
        self.declare_parameter("height_max", 0.50)
        self.declare_parameter("radius_min", 0.02)
        self.declare_parameter("radius_max", 0.3)

        # Tracking / frames
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("association_radius", 1.0)
        self.declare_parameter("max_missed_sec", 0.75)
        self.declare_parameter("max_move_per_sec", 6.0)

        # TF behaviour
        self.declare_parameter("tf_timeout_sec", 0.10)     # 100 ms lookup timeout
        self.declare_parameter("tf_use_latest", False)     # fallback to latest transform if timestamped lookup fails

        self.get_parameters()

        # TF2 buffer/listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS profile for sensors
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to point cloud
        self.sub_points = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.points_callback,
            qos
        )

        # Publishers
        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.pub_poses = self.create_publisher(PoseArray, self.poses_topic, 10)

        # Tracking data
        self.tracked_cones = {}  # id -> TrackedCone
        self.next_cone_id = 0

        self.get_logger().info(f"Cone detection node initialized | target_frame='{self.target_frame}'")

    def get_parameters(self):
        """Update parameters from ROS parameter server."""
        self.input_topic = self.get_parameter("input_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.poses_topic = self.get_parameter("poses_topic").value

        self.approx_layout = self.get_parameter("approx_layout").value
        self.cluster_eps = float(self.get_parameter("cluster_eps").value)
        self.min_points = int(self.get_parameter("min_points").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.voxel_leaf = float(self.get_parameter("voxel_leaf").value)

        self.height_min = float(self.get_parameter("height_min").value)
        self.height_max = float(self.get_parameter("height_max").value)
        self.radius_min = float(self.get_parameter("radius_min").value)
        self.radius_max = float(self.get_parameter("radius_max").value)

        self.target_frame = self.get_parameter("target_frame").value
        self.association_radius = float(self.get_parameter("association_radius").value)
        self.max_missed_sec = float(self.get_parameter("max_missed_sec").value)
        self.max_move_per_sec = float(self.get_parameter("max_move_per_sec").value)

        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.tf_use_latest = bool(self.get_parameter("tf_use_latest").value)

    # -------- TF utilities --------
    def _lookup_transform(self, target_frame: str, source_frame: str, stamp) -> np.ndarray:
        """
        Get 4x4 homogeneous transform from source_frame -> target_frame.
        Uses stamped lookup, with optional latest fallback.
        """
        timeout = rclpy.duration.Duration(seconds=self.tf_timeout_sec)
        try:
            # Stamped lookup at message time
            ts = Time.from_msg(stamp)
            tf: tf2_ros.TransformStamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, ts, timeout=timeout
            )
        except Exception as e_stamped:
            if not self.tf_use_latest:
                raise
            # Fallback to latest available
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time(), timeout=timeout
                )
                self.get_logger().warn(
                    f"Stamped TF lookup failed ({e_stamped}); using latest transform "
                    f"{source_frame}->{target_frame}"
                )
            except Exception as e_latest:
                raise e_latest

        t = tf.transform.translation
        q = tf.transform.rotation
        R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = np.array([t.x, t.y, t.z], dtype=float)
        return T

    def transform_points_to_target(self, xyz: np.ndarray, header) -> np.ndarray:
        """
        Transform Nx3 points from header.frame_id into self.target_frame using TF.
        """
        if header.frame_id == self.target_frame:
            return xyz

        try:
            T = self._lookup_transform(self.target_frame, header.frame_id, header.stamp)
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed {header.frame_id}->{self.target_frame}: {e}. "
                f"Skipping frame."
            )
            return np.empty((0, 3))

        # Homogeneous transform
        ones = np.ones((xyz.shape[0], 1), dtype=float)
        pts_h = np.hstack([xyz, ones])            # (N,4)
        xyz_t = (T @ pts_h.T).T[:, :3]            # (N,3)
        return xyz_t

    # -------- Pipeline --------
    def points_callback(self, msg: PointCloud2):
        """Process incoming point cloud and detect cones."""
        current_time = time()

        # Convert ROS point cloud to numpy array
        # NOTE: we only need x,y,z in the LiDAR frame
        pts = list(pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z')))
        if len(pts) == 0:
            return

                # Structured array (with named fields) → regular Nx3 float array
        if isinstance(pts, np.ndarray) and pts.dtype.names:
            xyz = np.vstack([pts['x'], pts['y'], pts['z']]).T.astype(float)
        else:
            xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=float)


        # ---- NEW: transform into target_frame (e.g., odom) BEFORE any processing ----
        xyz = self.transform_points_to_target(xyz, msg.header)
        if xyz.shape[0] == 0:
            return

        # Voxel downsampling
        xyz = self.voxel_downsample(xyz)
        if xyz.shape[0] < self.min_points:
            return

        # DBSCAN clustering
        clusters = self.cluster_points(xyz)

        # Extract cone positions from clusters
        detections = []
        for _, cluster_points in clusters.items():
            cone_pos = self.extract_cone_from_cluster(cluster_points)
            if cone_pos is not None:
                detections.append(cone_pos)

        # Data association and tracking (already in target frame)
        self.update_tracking(detections, current_time)

        # Publish markers and poses
        self.publish_markers_and_poses(msg.header, current_time)

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """Downsample points using voxel grid with Open3D."""
        if self.voxel_leaf <= 0:
            return points

        # Limit maximum points
        if points.shape[0] > self.max_points:
            idx = np.random.choice(points.shape[0], self.max_points, replace=False)
            points = points[idx]

        if o3d is None:
            # Optional: warn only once
            # self.get_logger().warn("Open3D not available, skipping voxel downsampling")
            return points

        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Voxel downsampling
        pcd_downsampled = pcd.voxel_down_sample(voxel_size=self.voxel_leaf)
        return np.asarray(pcd_downsampled.points)

    def cluster_points(self, points: np.ndarray) -> dict:
        """Cluster points using Euclidean Clustering with Open3D."""
        if points.shape[0] < self.min_points:
            return {}

        if o3d is None:
            self.get_logger().error("Open3D not available for clustering")
            return {}

        # Convert to Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Euclidean clustering
        labels = np.array(
            pcd.cluster_dbscan(eps=self.cluster_eps, min_points=self.min_points)
        )

        clusters = {}
        valid_labels = set(labels)
        if -1 in valid_labels:
            valid_labels.remove(-1)  # Skip noise

        for label in valid_labels:
            cluster_points = points[labels == label]
            if self.min_points <= cluster_points.shape[0] <= self.max_points:
                clusters[int(label)] = cluster_points

        return clusters

    def extract_cone_from_cluster(self, cluster_points: np.ndarray):
        """Extract cone position from a cluster of points (already in target frame)."""
        if cluster_points.shape[0] < 1:
            return None

        # Height in target frame
        z_min = cluster_points[:, 2].min()
        z_max = cluster_points[:, 2].max()
        height = z_max - z_min
        if not (self.height_min <= height <= self.height_max):
            return None

        # Radial spread in XY (target frame)
        xy_r = np.linalg.norm(cluster_points[:, :2], axis=1)
        radius = np.std(xy_r) if xy_r.shape[0] > 1 else 0.0
        if not (self.radius_min <= radius <= self.radius_max):
            return None

        # Use base of cone (lowest Z) near centroid in XY
        centroid_xy = cluster_points[:, :2].mean(axis=0)
        # Find point closest to centroid in XY among the lowest Z slice
        low_mask = np.isclose(cluster_points[:, 2], z_min, atol=0.02)
        base_candidates = cluster_points[low_mask] if np.any(low_mask) else cluster_points
        d_xy = np.linalg.norm(base_candidates[:, :2] - centroid_xy, axis=1)
        base_pt = base_candidates[np.argmin(d_xy)]

        return base_pt.copy()  # (x,y,z) in target frame

    def update_tracking(self, detections: list, current_time: float):
        """Update tracked cones with new detections (positions in target frame)."""
        detections = np.array(detections) if detections else np.empty((0, 3))

        # Associate detections to existing tracks
        associated = set()
        for cone_id, cone in list(self.tracked_cones.items()):
            if detections.size == 0:
                continue

            distances = np.linalg.norm(detections - cone.position, axis=1)
            closest_idx = int(np.argmin(distances))
            closest_dist = float(distances[closest_idx])

            # Check association constraints
            if closest_dist <= self.association_radius:
                time_diff = current_time - cone.last_seen
                movement_speed = (closest_dist / time_diff) if time_diff > 0 else 0.0

                if movement_speed <= self.max_move_per_sec:
                    cone.last_position = cone.position.copy()
                    # Smoothly update (helps with jitter)
                    cone.position = 0.7 * cone.position + 0.3 * detections[closest_idx]
                    cone.last_seen = current_time
                    cone.times_seen += 1
                    associated.add(closest_idx)

        # Create new tracks for unassociated detections
        for i, det in enumerate(detections):
            if i not in associated:
                new_cone = TrackedCone(
                    position=det,
                    last_seen=current_time,
                    cone_id=self.next_cone_id
                )
                self.tracked_cones[self.next_cone_id] = new_cone
                self.next_cone_id += 1

        # ✅ Permanence: do NOT delete old cones


    def publish_markers_and_poses(self, header, current_time: float):
        """Publish cone markers and poses (already in target frame)."""
        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = self.target_frame  # ensure target frame

        for cone_id, cone in self.tracked_cones.items():
            # ---- Cone Marker ----
            marker = Marker()
            marker.header = header
            marker.header.frame_id = self.target_frame
            marker.id = int(cone_id * 2)  # unique ID for geometry
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(cone.position[0])
            marker.pose.position.y = float(cone.position[1])
            marker.pose.position.z = float(cone.position[2] + 0.4)
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            marker.scale.x = float(2 * self.radius_max)
            marker.scale.y = float(2 * self.radius_max)
            marker.scale.z = float(self.height_max)
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
            marker.lifetime.sec = 1
            marker_array.markers.append(marker)

            # ---- Text Marker (ID Label) ----
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = self.target_frame
            text_marker.id = int(cone_id * 2 + 1)  # unique ID for text
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(cone.position[0])
            text_marker.pose.position.y = float(cone.position[1])
            text_marker.pose.position.z = float(cone.position[2] + 1.0)  # slightly above the cone
            text_marker.scale.z = 0.4  # text height (m)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = str(cone_id)
            text_marker.lifetime.sec = 1
            marker_array.markers.append(text_marker)

            # ---- Pose Array ----
            pose = Pose()
            pose.position.x = float(cone.position[0])
            pose.position.y = float(cone.position[1])
            pose.position.z = float(cone.position[2])
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
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