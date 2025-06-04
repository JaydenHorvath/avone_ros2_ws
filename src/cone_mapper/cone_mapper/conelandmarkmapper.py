#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

import numpy as np
import pcl

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


class ConeLandmarkMapperNode(Node):
    """
    Clusters a ground‐removed PointCloud2, computes each cluster’s 3D centroid,
    permutes from camera_link → optical frame, projects centroids to the image,
    compares with YOLO bounding boxes (only logs when matched), and then publishes
    a CYLINDER marker for each validated cone, sized according to the real cone specs:

      class 0 → big orange cone (0.285 m × 0.285 m base, 0.505 m tall)
      class 1 → small orange cone (0.228 m × 0.228 m base, 0.325 m tall)
      class 2 → small yellow cone (0.228 m × 0.228 m base, 0.325 m tall)
      class 4 → small blue cone   (0.228 m × 0.228 m base, 0.325 m tall)

    Validated centroids are then transformed back to camera_link, then to world_frame,
    stored as (world_pt, class_id, color), and finally published as a MarkerArray
    (cylinders) plus a PointCloud2 of all stored centroids.
    """

    def __init__(self):
        super().__init__('cone_landmark_mapper')

        # ─── Clustering parameters ──────────────────────────────────────────────
        self.declare_parameter('cluster_tolerance', 0.07)    # meters
        self.declare_parameter('min_cluster_size', 40)       # min points per cluster
        self.declare_parameter('max_cluster_size', 2000)     # max points per cluster

        self.cluster_tolerance = (
            self.get_parameter('cluster_tolerance')
                .get_parameter_value().double_value
        )
        self.min_cluster_size = (
            self.get_parameter('min_cluster_size')
                .get_parameter_value().integer_value
        )
        self.max_cluster_size = (
            self.get_parameter('max_cluster_size')
                .get_parameter_value().integer_value
        )

        # ─── World‐frame parameter ──────────────────────────────────────────────
        self.declare_parameter('world_frame', 'odom')
        self.world_frame = (
            self.get_parameter('world_frame')
                .get_parameter_value().string_value
        )

        # ─── Storage for validated cone centroids (world), class_id, and color ──
        # Each entry = ( [x_w, y_w, z_w], class_id, (r, g, b) )
        self.known_cones = []
        self.next_id = 0

        # ─── TF2 setup ─────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ─── Store latest CameraInfo & YOLO detections ─────────────────────────
        self.latest_caminfo = None        # sensor_msgs/CameraInfo
        self.latest_detections = []       # list of dicts {u_center, v_center, width, height, class_id, det_id}

        # ─── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(
            PointCloud2,
            '/cloud_no_ground_ransac',
            self.cloud_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.yolo_callback,
            10
        )

        # ─── Publishers ─────────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/cone_landmarks_colored',
            10
        )
        self.centroid_pub = self.create_publisher(
            PointCloud2,
            '/cone_landmark_centroids',
            10
        )

        self.get_logger().info(
            "[ConeLandmarkMapper] Subscribed to /cloud_no_ground_ransac, "
            "/camera/camera_info, /yolo/detections.\n"
            f"Publishing cylinder markers in world frame '{self.world_frame}'."
        )

        # ─── Real‐world cone sizes (meters) for each YOLO class ID ─────────────
        #    Format: class_id → (diameter, diameter, height)
        self.CLASS_ID_TO_SIZE = {
            0: (0.285, 0.285, 0.505),   # big orange cone: 285×285 mm base, 505 mm tall
            1: (0.228, 0.228, 0.325),   # small orange cone: 228×228 mm base, 325 mm tall
            2: (0.228, 0.228, 0.325),   # small yellow cone: same dims as small orange
            4: (0.228, 0.228, 0.325),   # small blue cone: same dims as small orange
        }

        # ─── Colors for each class (r,g,b) in [0,1] ─────────────────────────────
        #    (Note: you requested “big orange → blue,” “small orange → orange,”
        #     “small yellow → yellow,” “small blue → yellow”)
        self.CLASS_ID_TO_COLOR = {
            0: (0.0, 0.0, 1.0),    # Big‐orange detected → color blue
            1: (1.0, 0.5, 0.0),    # Small‐orange detected → color orange
            2: (1.0, 1.0, 0.0),    # Small‐yellow detected → color yellow
            4: (1.0, 1.0, 0.0),    # Small‐blue detected → color yellow
        }
        self.DEFAULT_COLOR = (0.5, 0.5, 0.5)  # gray if class_id is missing

    # ────────────────────────────────────────────────────────────────────────────
    def camera_info_callback(self, msg: CameraInfo):
        """
        Store the latest CameraInfo so we can project 3D→2D.
        """
        self.latest_caminfo = msg

    # ────────────────────────────────────────────────────────────────────────────
    def yolo_callback(self, msg: Detection2DArray):
        """
        Convert each Detection2D into a dictionary:
          {
            'u_center': float,
            'v_center': float,
            'width':    float,
            'height':   float,
            'class_id': int or None,
            'det_id':   string identifier (could be empty)
          }
        """
        boxes = []
        for det in msg.detections:
            bbox = det.bbox

            # 1) Extract (u_center, v_center)
            center = bbox.center
            if hasattr(center, 'x') and hasattr(center, 'y'):
                u_center = float(center.x)
                v_center = float(center.y)
            elif hasattr(center, 'position'):
                u_center = float(center.position.x)
                v_center = float(center.position.y)
            else:
                continue

            # 2) Extract (width, height)
            if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                width = float(bbox.size_x)
                height = float(bbox.size_y)
            elif all(hasattr(bbox, attr) for attr in ('xmin','xmax','ymin','ymax')):
                width  = float(bbox.xmax - bbox.xmin)
                height = float(bbox.ymax - bbox.ymin)
                u_center = float((bbox.xmin + bbox.xmax) / 2.0)
                v_center = float((bbox.ymin + bbox.ymax) / 2.0)
            else:
                continue

            # 3) Extract class_id (string→int if needed)
            class_id = None
            if hasattr(det, 'results') and len(det.results) > 0:
                try:
                    cid = det.results[0].hypothesis.class_id
                    if isinstance(cid, str) and cid.isdigit():
                        class_id = int(cid)
                    elif isinstance(cid, (int, float)):
                        class_id = int(cid)
                except:
                    class_id = None

            # 4) Extract det_id (string), if provided
            det_id = ''
            if hasattr(det, 'id'):
                det_id = det.id if isinstance(det.id, str) else ''

            boxes.append({
                'u_center': u_center,
                'v_center': v_center,
                'width':    width,
                'height':   height,
                'class_id': class_id,
                'det_id':   det_id
            })

        self.latest_detections = boxes

    # ────────────────────────────────────────────────────────────────────────────
    def cloud_callback(self, msg: PointCloud2):
        """
        1) Convert PointCloud2 → Nx3 NumPy array (camera_link frame).
        2) Filter NaN/Inf.
        3) Permute (x_cam,y_cam,z_cam) → (x_opt,y_opt,z_opt) for optical frame.
        4) Euclidean clustering → cluster_indices on optical cloud.
        5) Compute cluster centroids (x_opt,y_opt,z_opt).
        6) Project each optical centroid → (u,v) using CameraInfo.
        7) Compare (u,v) to YOLO boxes; only log & keep those that match (with det_id).
        8) Convert matched optical centroid back to camera_link (x_link,y_link,z_link).
        9) Transform (x_link,y_link,z_link) → world_frame; store new cones as (pt,class_id,color).
       10) Publish all known cones as CYLINDER markers (correct real‐world scale) + a PointCloud2.
        """

        # 0) We need CameraInfo before proceeding
        if self.latest_caminfo is None:
            self.get_logger().warn(
                "[ConeLandmarkMapper] No CameraInfo yet; skipping clustering."
            )
            self._publish_all_markers(msg.header)
            return

        # 1) Read (x_cam, y_cam, z_cam) tuples from PointCloud2
        xyz_list = [
            (pt[0], pt[1], pt[2])
            for pt in point_cloud2.read_points(
                msg,
                field_names=('x','y','z'),
                skip_nans=False
            )
        ]
        if not xyz_list:
            self.get_logger().warn(
                "[ConeLandmarkMapper] Received empty/invalid PointCloud2."
            )
            self._publish_all_markers(msg.header)
            return

        # 2) Convert to Nx3 array and filter out any rows containing NaN/Inf
        cloud_arr = np.asarray(xyz_list, dtype=np.float32)  # in camera_link frame
        valid_mask = np.isfinite(cloud_arr).all(axis=1)
        cloud_arr = cloud_arr[valid_mask]
        if cloud_arr.size == 0:
            self.get_logger().warn(
                "[ConeLandmarkMapper] All points were NaN/Inf after filtering."
            )
            self._publish_all_markers(msg.header)
            return

        # 3) Permute each (x_cam,y_cam,z_cam) → (x_opt,y_opt,z_opt) so that +Z_opt is forward
        #    camera_link: x_cam=forward, y_cam=left,  z_cam=up
        #    optical:     x_opt=right,   y_opt=down,  z_opt=forward
        #
        #    x_opt = -y_cam
        #    y_opt = -z_cam
        #    z_opt =  x_cam
        cloud_arr_opt = np.zeros_like(cloud_arr)
        cloud_arr_opt[:,0] = -cloud_arr[:,1]   # x_opt = -y_cam
        cloud_arr_opt[:,1] = -cloud_arr[:,2]   # y_opt = -z_cam
        cloud_arr_opt[:,2] =  cloud_arr[:,0]   # z_opt =  x_cam

        # 4) Build a PCL PointCloud from optical coords and run Euclidean clustering
        pcl_cloud = pcl.PointCloud(cloud_arr_opt)
        tree = pcl_cloud.make_kdtree()

        ec = pcl_cloud.make_EuclideanClusterExtraction()
        try:
            ec.set_cluster_tolerance(self.cluster_tolerance)
        except AttributeError:
            ec.set_ClusterTolerance(self.cluster_tolerance)
        try:
            ec.set_min_cluster_size(self.min_cluster_size)
        except AttributeError:
            ec.set_MinClusterSize(self.min_cluster_size)
        try:
            ec.set_max_cluster_size(self.max_cluster_size)
        except AttributeError:
            ec.set_MaxClusterSize(self.max_cluster_size)
        try:
            ec.set_search_method(tree)
        except AttributeError:
            ec.set_SearchMethod(tree)

        try:
            cluster_indices = ec.Extract()
        except Exception as e:
            self.get_logger().error(f"[ConeLandmarkMapper] Clustering failed: {e}")
            self._publish_all_markers(msg.header)
            return

        if len(cluster_indices) == 0:
            self._publish_all_markers(msg.header)
            return

        # 5) Compute centroids in optical coords
        pcl_array_opt = np.asarray(pcl_cloud, dtype=np.float32)
        centroids_opt = []
        for indices in cluster_indices:
            pts_opt = pcl_array_opt[indices]
            centroid_opt = np.mean(pts_opt, axis=0)  # (x_opt, y_opt, z_opt)
            centroids_opt.append(centroid_opt.tolist())

        # 6) Project each optical centroid → pixel (u,v)
        fx = self.latest_caminfo.k[0]
        fy = self.latest_caminfo.k[4]
        cx = self.latest_caminfo.k[2]
        cy = self.latest_caminfo.k[5]

        projected = []
        for idx_opt, (x_opt, y_opt, z_opt) in enumerate(centroids_opt):
            # In optical frame, “forward” means z_opt > 0
            if z_opt <= 0:
                continue
            u = (fx * x_opt / z_opt) + cx
            v = (fy * y_opt / z_opt) + cy
            projected.append((u, v, idx_opt))

        if not projected:
            self._publish_all_markers(msg.header)
            return

        # 7) Compare (u,v) to YOLO boxes; only keep those that match, logging the match
        validated = []  # will hold tuples (idx_opt, class_id, color)
        for (u, v, idx_opt) in projected:
            matched_color = self.DEFAULT_COLOR
            matched_class = None
            matched_det_id = None

            for box in self.latest_detections:
                u_min = box['u_center'] - (box['width'] / 2.0)
                u_max = box['u_center'] + (box['width'] / 2.0)
                v_min = box['v_center'] - (box['height'] / 2.0)
                v_max = box['v_center'] + (box['height'] / 2.0)

                if (u_min <= u <= u_max) and (v_min <= v <= v_max):
                    cls_id = box.get('class_id', None)
                    det_id = box.get('det_id', '') or f"idx_{self.latest_detections.index(box)}"

                    if cls_id in self.CLASS_ID_TO_COLOR:
                        matched_color = self.CLASS_ID_TO_COLOR[cls_id]
                        matched_class = cls_id
                        matched_det_id = det_id
                    break

            if matched_class is not None:
                self.get_logger().info(
                    f"[ConeLandmarkMapper] Centroid {idx_opt} matched YOLO ID '{matched_det_id}' "
                    f"(class {matched_class}); assigning color={matched_color}."
                )
                validated.append((idx_opt, matched_class, matched_color))

        if not validated:
            self._publish_all_markers(msg.header)
            return

        # 8) Transform each validated optical centroid back to camera_link, then to world_frame
        camera_frame = msg.header.frame_id  # typically "rgbdcamera_link"
        stamp = Time(
            seconds=msg.header.stamp.sec,
            nanoseconds=msg.header.stamp.nanosec
        )

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                camera_frame,
                stamp
            )
        except TransformException:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    camera_frame,
                    Time()  # fallback to latest
                )
            except TransformException as ex2:
                self.get_logger().error(f"[ConeLandmarkMapper] TF lookup failed: {ex2}")
                self._publish_all_markers(msg.header)
                return

        eps = 0.10  # 10 cm threshold for “same cone”
        for (idx_opt, cls_id, color) in validated:
            x_opt, y_opt, z_opt = centroids_opt[idx_opt]

            # Convert optical → camera_link before TF:
            #   x_link =  z_opt
            #   y_link = -x_opt
            #   z_link = -y_opt
            x_link =  z_opt
            y_link = -x_opt
            z_link = -y_opt

            p_cam = PointStamped()
            p_cam.header.frame_id = camera_frame
            p_cam.header.stamp = msg.header.stamp
            p_cam.point.x = float(x_link)
            p_cam.point.y = float(y_link)
            p_cam.point.z = float(z_link)

            try:
                p_world = do_transform_point(p_cam, tf_msg)
            except TransformException as e:
                self.get_logger().error(f"[ConeLandmarkMapper] Point transform failed: {e}")
                continue

            world_pt = [p_world.point.x, p_world.point.y, p_world.point.z]
            # Only add if new (within eps)
            if not any(
                np.linalg.norm(np.array(world_pt) - np.array(old_pt)) < eps
                for old_pt, _, _ in self.known_cones
            ):
                # Store (world_pt, class_id, color)
                self.known_cones.append((world_pt, cls_id, color))

        # 9) Publish all known cones as CYLINDER markers + a small PointCloud2
        self._publish_all_markers(msg.header)

    # ────────────────────────────────────────────────────────────────────────────
    def _publish_all_markers(self, header):
        """
        Publish a MarkerArray (frame = world_frame) of cylinders whose scale matches
        each cone’s real base‐diameter and height, colored according to class_id.
        Also publish a small PointCloud2 of all stored centroids.
        """
        marker_array = MarkerArray()
        centroids_world = []

        for idx, (pt, cls_id, color) in enumerate(self.known_cones):
            xw, yw, zw = pt

            # Look up the real‐world size for this class:
            if cls_id in self.CLASS_ID_TO_SIZE:
                diameter, _, height = self.CLASS_ID_TO_SIZE[cls_id]
            else:
                # If missing, fallback to a 0.2m×0.2m base, 0.2m tall cylinder
                diameter, height = 0.2, 0.2

            r, g, b = color

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = header.stamp
            m.ns = "cone_landmarks"
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            # Position the cylinder so that its base sits on zw (ground). A CYLINDER’s origin is at its midpoint along Z:
            m.pose.position.x = float(xw)
            m.pose.position.y = float(yw)
            m.pose.position.z = float(zw + (height / 2.0))
            m.pose.orientation.w = 1.0

            # Set scale to (diameter, diameter, height)
            m.scale.x = float(diameter)
            m.scale.y = float(diameter)
            m.scale.z = float(height)

            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 0.8

            marker_array.markers.append(m)
            centroids_world.append((float(xw), float(yw), float(zw)))

        # Publish MarkerArray of cylinders
        self.marker_pub.publish(marker_array)

        # Also publish a PointCloud2 of all centroids in world frame
        header_pc = Header()
        header_pc.stamp = header.stamp
        header_pc.frame_id = self.world_frame
        centroid_cloud = point_cloud2.create_cloud_xyz32(header_pc, centroids_world)
        self.centroid_pub.publish(centroid_cloud)

    # ────────────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = ConeLandmarkMapperNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
