#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo

import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

import numpy as np
from typing import List, Dict, Tuple, Optional
import time


def pos_key_xyz(xyz: np.ndarray, precision: float = 0.15) -> Tuple[int, int, int]:
    return tuple((xyz / precision).round().astype(int))


class VoxelCones(Node):
    """
    Turn Nav2 voxel_grid markers into cone positions by clustering occupied voxels.
    Supports both visualization_msgs/Marker and MarkerArray inputs.
    Adds YOLO-based classification (blue/yellow/orange) via camera projection.
    """

    def __init__(self):
        super().__init__('voxel_cones')

        # ---- Params ----
        self.declare_parameter('world_frame', 'map')                    # output frame
        self.declare_parameter('voxel_marker_topic', '/voxel_markers')  # Marker
        self.declare_parameter('voxel_marker_array_topic', '/voxel_markers_array')  # MarkerArray
        self.declare_parameter('z_min', 0.05)        # m
        self.declare_parameter('z_max', 0.60)        # m
        self.declare_parameter('cluster_eps', 0.40)  # m
        self.declare_parameter('min_pts', 3)
        self.declare_parameter('marker_scale', 0.30) # m
        self.declare_parameter('publish_ns', 'cones_voxel')

        # Classification params
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('caminfo_topic', '/yolo/camera_info')
        self.declare_parameter('camera_frame_override', '')             # optional manual override
        self.declare_parameter('min_score', 0.30)
        self.declare_parameter('bin_size', 0.15)     # voting bin size (m)
        self.declare_parameter('vote_decay', 0.95)   # 0..1, larger = more inertia

        self.world_frame = self.get_parameter('world_frame').value

        # QoS: match voxel marker publisher (RELIABLE)
        qos_rel = QoSProfile(depth=10)
        qos_rel.reliability = QoSReliabilityPolicy.RELIABLE
        qos_rel.history = QoSHistoryPolicy.KEEP_LAST

        # Subscriptions to BOTH marker and marker array
        topic_marker = self.get_parameter('voxel_marker_topic').value
        topic_array  = self.get_parameter('voxel_marker_array_topic').value

        self.sub_marker = self.create_subscription(Marker,      topic_marker, self.marker_cb, qos_rel)
        self.sub_array  = self.create_subscription(MarkerArray, topic_array,  self.marker_array_cb, qos_rel)

        # YOLO + Camera
        self.dets_topic = self.get_parameter('detections_topic').value
        self.caminfo_topic = self.get_parameter('caminfo_topic').value
        self.create_subscription(Detection2DArray, self.dets_topic, self.yolo_cb, 10)
        self.create_subscription(CameraInfo, self.caminfo_topic, self.caminfo_cb, 10)

        # Publishers (combined)
        self.pub_markers   = self.create_publisher(MarkerArray, '/cone_landmarks_voxel', 10)
        self.pub_posearray = self.create_publisher(PoseArray,   '/cone_positions', 10)
        self.pub_count     = self.create_publisher(Int32,       '/cone_count', 10)

        # Publishers (per-class)
        self.pub_cls_markers = {
            'blue':   self.create_publisher(MarkerArray, '/cones_classified/blue',   10),
            'yellow': self.create_publisher(MarkerArray, '/cones_classified/yellow', 10),
            'orange': self.create_publisher(MarkerArray, '/cones_classified/orange', 10),
            'unknown':self.create_publisher(MarkerArray, '/cones_classified/unknown',10),
        }
        self.pub_cls_poses = {
            'blue':   self.create_publisher(PoseArray, '/cone_positions_blue',   10),
            'yellow': self.create_publisher(PoseArray, '/cone_positions_yellow', 10),
            'orange': self.create_publisher(PoseArray, '/cone_positions_orange', 10),
            'unknown':self.create_publisher(PoseArray, '/cone_positions_unknown',10),
        }

        # TF + camera intrinsics state
        self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.latest_caminfo: Optional[CameraInfo] = None
        self.camera_frame: Optional[str] = None
        self.fx = self.fy = self.cx = self.cy = None

        # Latest YOLO detections (list of dict)
        self.latest_detections: List[dict] = []

        # Class ID mapping (adjust if your YOLO IDs differ)
        self.CLASS_MAP = {
            0: ('blue',   (0.0, 0.0, 1.0)),
            4: ('yellow', (1.0, 1.0, 0.0)),
            1: ('orange', (1.0, 0.5, 0.0)),
            2: ('orange', (1.0, 0.5, 0.0)),
        }

        # Persistence memory per spatial bin
        # memory[key] = {'blue':w,'yellow':w,'orange':w,'unknown':w,'pos':np.array([x,y,z])}
        self.memory: Dict[Tuple[int,int,int], dict] = {}

        # tiny logger throttle helper (optional)
        self._last_warn: Dict[str, float] = {}

        self.get_logger().info(
            f'[VoxelCones] Subscribed to: {topic_marker} (Marker) and {topic_array} (MarkerArray). '
            f'Publishing cone positions in frame: {self.world_frame}'
        )

    # --- Input handlers ---
    def marker_cb(self, m: Marker):
        self.process_markers([m])

    def marker_array_cb(self, ma: MarkerArray):
        self.process_markers(list(ma.markers))

    def caminfo_cb(self, msg: CameraInfo):
        self.latest_caminfo = msg
        cam_override = self.get_parameter('camera_frame_override').value or ''
        self.camera_frame = cam_override if cam_override else (msg.header.frame_id or self.camera_frame)
        K = msg.k
        self.fx, self.fy, self.cx, self.cy = K[0], K[4], K[2], K[5]

    def yolo_cb(self, msg: Detection2DArray):
        boxes = []
        min_score = float(self.get_parameter('min_score').value)
        for det in msg.detections:
            bbox = det.bbox
            # prefer center + size
            if hasattr(bbox, 'center') and hasattr(bbox.center, 'x'):
                u = float(bbox.center.x); v = float(bbox.center.y)
                if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                    w = float(bbox.size_x); h = float(bbox.size_y)
                else:
                    w = getattr(bbox, 'xmax', 0.0) - getattr(bbox, 'xmin', 0.0)
                    h = getattr(bbox, 'ymax', 0.0) - getattr(bbox, 'ymin', 0.0)
                u0, u1 = u - w/2.0, u + w/2.0
                v0, v1 = v - h/2.0, v + h/2.0
            else:
                if all(hasattr(bbox, a) for a in ('xmin','xmax','ymin','ymax')):
                    u0, u1 = float(bbox.xmin), float(bbox.xmax)
                    v0, v1 = float(bbox.ymin), float(bbox.ymax)
                else:
                    continue
            cid, score = -1, 0.0
            if det.results:
                hyp = det.results[0].hypothesis
                try:
                    cid = int(hyp.class_id)
                    score = float(hyp.score)
                except Exception:
                    pass
            if score >= min_score:
                boxes.append({'u0':u0,'u1':u1,'v0':v0,'v1':v1,'class_id':cid,'score':score})
        self.latest_detections = boxes

    # --- Core processing ---
    def process_markers(self, markers: List[Marker]):
        points = []
        stamp = None
        kept = 0
        for m in markers:
            # keep list-type markers with embedded points
            if m.type in (Marker.CUBE_LIST, Marker.SPHERE_LIST, Marker.POINTS):
                kept += 1
                if stamp is None:
                    stamp = m.header.stamp
                for p in m.points:
                    points.append((p.x, p.y, p.z))

        if stamp is None:
            stamp = self.get_clock().now().to_msg()

        if not points:
            self._publish_deleteall()
            self._publish_class_deleteall()
            self.pub_count.publish(Int32(data=0))
            return

        pts = np.asarray(points, dtype=np.float32)

        # Z bandpass → roughly cone height
        z_min = float(self.get_parameter('z_min').value)
        z_max = float(self.get_parameter('z_max').value)
        mask = (pts[:,2] >= z_min) & (pts[:,2] <= z_max)
        pts = pts[mask]
        if len(pts) == 0:
            self._publish_deleteall()
            self._publish_class_deleteall()
            self.pub_count.publish(Int32(data=0))
            return

        # Cluster in XY (lightweight DBSCAN-ish)
        eps = float(self.get_parameter('cluster_eps').value)
        min_pts = int(self.get_parameter('min_pts').value)
        clusters = self._cluster_xy(pts[:, :2], eps, min_pts)

        # Centroids
        centroids = []
        for cl in clusters:
            if len(cl) == 0:
                continue
            centroids.append(pts[cl].mean(axis=0))  # x,y,z
        centroids = np.asarray(centroids, dtype=np.float32)

        # Publish combined outputs (original behavior)
        self._publish_centroids(centroids, stamp)
        self.pub_count.publish(Int32(data=len(centroids)))

        # Classify + publish per-class
        self._classify_and_publish(centroids, stamp)

        self.get_logger().info(f'[VoxelCones] list_markers={kept} points_in_band={len(pts)} clusters={len(centroids)}')

    # --- Simple clustering ---
    def _cluster_xy(self, pts_xy: np.ndarray, eps: float, min_pts: int):
        N = len(pts_xy)
        if N == 0:
            return []
        visited = np.zeros(N, dtype=bool)
        clustered = np.full(N, -1, dtype=int)
        clusters = []
        cid = 0

        def neighbors(idx):
            d = np.linalg.norm(pts_xy - pts_xy[idx], axis=1)
            return np.where(d <= eps)[0]

        for i in range(N):
            if visited[i]:
                continue
            visited[i] = True
            nbrs = neighbors(i)
            if len(nbrs) < min_pts:
                continue
            clusters.append([])
            clustered[i] = cid
            clusters[cid].append(i)
            seeds = set(nbrs.tolist())
            seeds.discard(i)
            while seeds:
                j = seeds.pop()
                if not visited[j]:
                    visited[j] = True
                    nj = neighbors(j)
                    if len(nj) >= min_pts:
                        seeds.update(nj.tolist())
                if clustered[j] == -1:
                    clustered[j] = cid
                    clusters[cid].append(j)
            cid += 1
        return clusters

    # --- Classification + publishing ---
    def _classify_and_publish(self, centroids: np.ndarray, stamp):
        # If camera/detections not ready, publish all as unknown
        if self.latest_caminfo is None or self.camera_frame is None or self.fx is None:
            self._maybe_warn('no_caminfo', f'[VoxelCones] No CameraInfo/camera_frame yet; publishing UNKNOWN only')
            per_class = {'unknown': centroids, 'blue': np.zeros((0,3),dtype=np.float32),
                         'yellow': np.zeros((0,3),dtype=np.float32), 'orange': np.zeros((0,3),dtype=np.float32)}
            self._publish_per_class(per_class, stamp)
            return

        # Decay memory
        vote_decay = float(self.get_parameter('vote_decay').value)
        for key, rec in list(self.memory.items()):
            for k in ['blue','yellow','orange','unknown']:
                rec[k] *= vote_decay
            if max(rec['blue'],rec['yellow'],rec['orange'],rec['unknown']) < 0.05:
                self.memory.pop(key, None)

        # TF: map -> camera with backoff
        if stamp.sec != 0 or stamp.nanosec != 0:
            t_lookup = Time.from_msg(stamp)
        else:
            t_lookup = Time()

        try:
            tf_map_cam = self._lookup_tf_with_backoff(self.camera_frame, self.world_frame, t_lookup, 0.2)
        except TransformException as e:
            # Graceful fallback: publish unknown, no crash
            self._maybe_warn('tf_fail', f'TF {self.world_frame}->{self.camera_frame} failed: {e}')
            per_class = {'unknown': centroids, 'blue': np.zeros((0,3),dtype=np.float32),
                         'yellow': np.zeros((0,3),dtype=np.float32), 'orange': np.zeros((0,3),dtype=np.float32)}
            self._publish_per_class(per_class, stamp)
            return

        bin_size = float(self.get_parameter('bin_size').value)

        # Classify each centroid
        for c in centroids:
            # project to camera
            pw = PointStamped()
            pw.header.frame_id = self.world_frame
            pw.header.stamp    = stamp
            pw.point.x, pw.point.y, pw.point.z = float(c[0]), float(c[1]), float(c[2])
            try:
                pc = do_transform_point(pw, tf_map_cam)
            except TransformException:
                continue
            if pc.point.z <= 0.0:
                continue

            u = self.fx * pc.point.x / pc.point.z + self.cx
            v = self.fy * pc.point.y / pc.point.z + self.cy

            # best detection containing (u,v)
            best = None; best_score = -1.0
            for d in self.latest_detections:
                if d['u0'] <= u <= d['u1'] and d['v0'] <= v <= d['v1']:
                    if d['score'] > best_score:
                        best = d; best_score = d['score']

            key = pos_key_xyz(np.array([c[0], c[1], c[2]]), bin_size)
            rec = self.memory.get(key, {'blue':0.0,'yellow':0.0,'orange':0.0,'unknown':0.0,'pos':np.array(c)})

            if best and best['class_id'] in self.CLASS_MAP:
                name, _ = self.CLASS_MAP[best['class_id']]
                rec[name] += 1.0
            else:
                rec['unknown'] += 0.5

            # keep pos fresh
            rec['pos'] = 0.5*rec['pos'] + 0.5*np.array(c)
            self.memory[key] = rec

        # Build per-class arrays
        per_class_lists: Dict[str, List[np.ndarray]] = {'blue':[], 'yellow':[], 'orange':[], 'unknown':[]}
        for rec in self.memory.values():
            cls = max(['blue','yellow','orange','unknown'], key=lambda k: rec[k])
            per_class_lists[cls].append(rec['pos'])

        per_class = {}
        for k, lst in per_class_lists.items():
            per_class[k] = np.vstack(lst) if len(lst) else np.zeros((0,3), dtype=np.float32)

        self._publish_per_class(per_class, stamp)

    def _lookup_tf_with_backoff(self, target: str, source: str, at_time: Time, timeout_sec=0.2):
        attempts = [at_time]
        for i in range(5):
            attempts.append(at_time - Duration(seconds=0.1*(i+1)))
        attempts.append(Time())  # latest
        for t in attempts:
            if self.tf_buffer.can_transform(target, source, t, timeout=Duration(seconds=timeout_sec)):
                return self.tf_buffer.lookup_transform(target, source, t, timeout=Duration(seconds=timeout_sec))
        raise TransformException(f'No transform {source}->{target} at/before requested time, and latest unavailable')

    # --- Publishing helpers (combined) ---
    def _publish_centroids(self, centroids: np.ndarray, stamp):
        ns = self.get_parameter('publish_ns').value
        s  = float(self.get_parameter('marker_scale').value)

        # 1) Clear previous
        self._publish_deleteall()

        # 2) Markers (combined)
        ma = MarkerArray()
        for i, c in enumerate(centroids):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp    = stamp
            m.ns              = ns
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.scale.x = s; m.scale.y = s; m.scale.z = s
            m.color.r, m.color.g, m.color.b, m.color.a = (1.0, 0.5, 0.0, 0.9)
            ma.markers.append(m)
        self.pub_markers.publish(ma)

        # 3) PoseArray (combined)
        pa = PoseArray()
        pa.header.frame_id = self.world_frame
        pa.header.stamp    = stamp
        for c in centroids:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), float(c[2])
            pa.poses.append(p)
        self.pub_posearray.publish(pa)

    def _publish_deleteall(self):
        ma = MarkerArray()
        for ns, base_id in [('cones_voxel', 0), ('cones_voxel_labels', 10000)]:
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = base_id
            m.action = Marker.DELETEALL
            ma.markers.append(m)
        self.pub_markers.publish(ma)

    # --- Publishing helpers (per-class) ---
    def _publish_per_class(self, per_class: Dict[str, np.ndarray], stamp):
        # Delete then publish for each class
        for name in ['blue','yellow','orange','unknown']:
            self._publish_class_deleteall_ns(name)
            self._publish_class_markers(name, per_class.get(name, np.zeros((0,3))), stamp)
            self._publish_class_posearray(name, per_class.get(name, np.zeros((0,3))), stamp)

    def _publish_class_deleteall_ns(self, name: str):
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = f'cones_{name}'
        m.id = 0
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.pub_cls_markers[name].publish(ma)

    def _publish_class_markers(self, name: str, pts: np.ndarray, stamp):
        color = (1.0,0.0,0.0)
        if name == 'blue':   color = (0.0,0.0,1.0)
        if name == 'yellow': color = (1.0,1.0,0.0)
        if name == 'orange': color = (1.0,0.5,0.0)

        s  = float(self.get_parameter('marker_scale').value)
        ma = MarkerArray()
        for i, p in enumerate(pts):
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp    = stamp
            m.ns              = f'cones_{name}'
            m.id              = i
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD
            m.pose.position.x = float(p[0])
            m.pose.position.y = float(p[1])
            m.pose.position.z = float(p[2])
            m.scale.x = s; m.scale.y = s; m.scale.z = s
            m.color.r, m.color.g, m.color.b, m.color.a = (color[0], color[1], color[2], 0.9)
            ma.markers.append(m)
        self.pub_cls_markers[name].publish(ma)

    def _publish_class_posearray(self, name: str, pts: np.ndarray, stamp):
        pa = PoseArray()
        pa.header.frame_id = self.world_frame
        pa.header.stamp    = stamp
        for p in pts:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(p[0]), float(p[1]), float(p[2])
            pa.poses.append(pose)
        self.pub_cls_poses[name].publish(pa)

    # --- tiny logger throttle (optional) ---
    def _maybe_warn(self, key: str, msg: str, period_s: float = 2.0):
        now = time.monotonic()
        last = self._last_warn.get(key, 0.0)
        if now - last >= period_s:
            self._last_warn[key] = now
            self.get_logger().warn(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoxelCones()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
