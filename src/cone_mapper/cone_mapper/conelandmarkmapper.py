#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, CameraInfo
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Point

import numpy as np
import pcl

import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException


class ConeLandmarkMapperNode(Node):
    """
    - Clusters a ground‐removed PointCloud2, finds cluster centroids.
    - Maintains a single list self.known_cones, where each entry is:
        ([x_w, y_w, z_w], class_id_or_None, (r,g,b), is_validated_bool)
    - Publishes ALL of them on /cone_landmarks_colored as a single MarkerArray.
    - Additionally publishes validated blue cones on /cone_landmarks_blue and validated yellow cones on /cone_landmarks_yellow, retaining their IDs and text markers.
    """

    def __init__(self):
        super().__init__('cone_landmark_mapper')

        # clustering parameters
        self.declare_parameter('cluster_tolerance', 0.07)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 2000)
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').get_parameter_value().double_value
        self.min_cluster_size  = self.get_parameter('min_cluster_size').get_parameter_value().integer_value
        self.max_cluster_size  = self.get_parameter('max_cluster_size').get_parameter_value().integer_value

        # world frame
        self.declare_parameter('world_frame', 'map')
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        # storage for cones
        self.known_cones = []

        # TF2 setup
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # latest camera info & detections
        self.latest_caminfo   = None
        self.latest_detections = []

        # subscribers
        self.create_subscription(PointCloud2, '/cloud_no_ground_ransac', self.cloud_callback, 10)
        self.create_subscription(CameraInfo,  '/camera/camera_info',       self.camera_info_callback, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections',     self.yolo_callback,      10)

        # publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_landmarks_colored', 10)
        self.blue_pub   = self.create_publisher(MarkerArray, '/cone_landmarks_blue',    10)
        self.yellow_pub = self.create_publisher(MarkerArray, '/cone_landmarks_yellow',  10)

        self.get_logger().info('[ConeLandmarkMapper] Node started.')

        # cone sizes & colors
        self.CLASS_ID_TO_SIZE = {
            0: (0.285, 0.285, 0.505),  # big‐orange → blue
            1: (0.228, 0.228, 0.325),  # small‐orange
            2: (0.228, 0.228, 0.325),  # small‐yellow → yellow
            4: (0.228, 0.228, 0.325),  # small‐blue   → yellow
        }
        self.CLASS_ID_TO_COLOR = {
            0: (0.0, 0.0, 1.0),  # blue
            1: (1.0, 0.5, 0.0),  # orange
            2: (1.0, 1.0, 0.0),  # yellow
            4: (1.0, 1.0, 0.0),  # yellow
        }
        self.DEFAULT_COLOR = (0.5, 0.5, 0.5)

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_caminfo = msg

    def yolo_callback(self, msg: Detection2DArray):
        """
        Convert each Detection2D into a simple dict. We use these to validate centroids.
        """
        boxes = []
        for det in msg.detections:
            bbox = det.bbox

            # Extract center (u_center, v_center)
            center = bbox.center
            if hasattr(center, 'x') and hasattr(center, 'y'):
                u_center = float(center.x)
                v_center = float(center.y)
            elif hasattr(center, 'position'):
                u_center = float(center.position.x)
                v_center = float(center.position.y)
            else:
                continue

            # Extract width, height
            if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
                width  = float(bbox.size_x)
                height = float(bbox.size_y)
            elif all(hasattr(bbox, attr) for attr in ('xmin','xmax','ymin','ymax')):
                width  = float(bbox.xmax - bbox.xmin)
                height = float(bbox.ymax - bbox.ymin)
                u_center = float((bbox.xmin + bbox.xmax) / 2.0)
                v_center = float((bbox.ymin + bbox.ymax) / 2.0)
            else:
                continue

            # Extract class_id
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

            # Extract det_id (string)
            det_id = ''
            if hasattr(det, 'id'):
                det_id = det.id if isinstance(det.id, str) else ''

            # Extract confidence score
            score = 0.0
            if hasattr(det, 'results') and len(det.results) > 0:
                try:
                    score = float(det.results[0].hypothesis.score)
                except:
                    score = 0.0

            boxes.append({
                'u_center': u_center,
                'v_center': v_center,
                'width':    width,
                'height':   height,
                'class_id': class_id,
                'det_id':   det_id,
                'score':    score
            })

        self.latest_detections = boxes


    # ────────────────────────────────────────────────────────────────────────────
    def cloud_callback(self, msg: PointCloud2):
        """
        1) Convert incoming PointCloud2 → (N×3) NumPy array in camera_link frame.
        2) Drop NaN/Inf.
        3) Permute (x_cam,y_cam,z_cam) → (x_opt,y_opt,z_opt).
        4) Euclidean clustering → a list of cluster_indices.
        5) Compute each cluster’s centroid in optical coords → transform to world_frame.
           For each new world_pt:
             - Append ([x,y,z], None, (0.5,0.5,0.5), is_validated=False) to self.known_cones.
             - Also record it in self.known_clusters so you don’t re‐insert the same cluster twice.
        6) Publish a single MarkerArray (spheres & cylinders) via _publish_all_markers().
        7) Then try to “validate” each visible optical centroid against YOLO boxes:
           • If a centroid projects into a YOLO box, pick the box with highest confidence.
           • Transform that centroid back to world_frame, find its index in self.known_cones,
             and set that entry’s (class_id, color, is_validated=True).
        8) Publish the same MarkerArray again via _publish_all_markers() so those IDs switch
           from sphere → cylinder.
        """

        # 0) Ensure we have CameraInfo (for projection)
        if self.latest_caminfo is None:
            self.get_logger().warn("[ConeLandmarkMapper] No CameraInfo; skipping.")
            self._publish_all_markers(msg.header)
            return

        # 1) Read (x_cam, y_cam, z_cam) from PointCloud2
        xyz_list = [
            (pt[0], pt[1], pt[2])
            for pt in point_cloud2.read_points(
                msg, field_names=('x','y','z'), skip_nans=False
            )
        ]
        if not xyz_list:
            self.get_logger().warn("[ConeLandmarkMapper] Empty / invalid cloud.")
            self._publish_all_markers(msg.header)
            return

        # 2) Convert → float32 array, drop any NaN or Inf
        cloud_arr = np.asarray(xyz_list, dtype=np.float32)  # shape=(K,3)
        valid_mask = np.isfinite(cloud_arr).all(axis=1)
        cloud_arr = cloud_arr[valid_mask]
        if cloud_arr.size == 0:
            self.get_logger().warn("[ConeLandmarkMapper] All points were NaN/Inf.")
            self._publish_all_markers(msg.header)
            return

        # 3) Permute from camera_link → optical
        #    camera_link: x_cam=forward, y_cam=left, z_cam=up
        #    optical:     x_opt = –y_cam, y_opt = –z_cam, z_opt = x_cam
        cloud_opt = np.zeros_like(cloud_arr)
        cloud_opt[:, 0] = -cloud_arr[:, 1]
        cloud_opt[:, 1] = -cloud_arr[:, 2]
        cloud_opt[:, 2] =  cloud_arr[:, 0]

        # 4) Build PCL cloud & cluster
        pcl_cloud = pcl.PointCloud(cloud_opt)
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
            # no clusters → just re‐publish whatever we have
            self._publish_all_markers(msg.header)
            return

        # 5) Compute ALL raw optical centroids
        pcl_array_opt = np.asarray(pcl_cloud, dtype=np.float32)
        centroids_opt = []
        for indices in cluster_indices:
            pts = pcl_array_opt[indices]
            centroid = np.mean(pts, axis=0)
            centroids_opt.append(centroid)
        centroids_opt = np.stack(centroids_opt, axis=0)  # shape=(N,3)

        # 5a) Transform each raw_opt centroid (optical → camera_link → world_frame):
        camera_frame = msg.header.frame_id  # e.g. "rgbdcamera_link"
        stamp = Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
        try:
            tf_cam_to_world = self.tf_buffer.lookup_transform(
                self.world_frame, camera_frame, stamp
            )
        except TransformException:
            try:
                tf_cam_to_world = self.tf_buffer.lookup_transform(
                    self.world_frame, camera_frame, Time()
                )
            except TransformException as ex2:
                self.get_logger().error(f"[ConeLandmarkMapper] TF lookup failed: {ex2}")
                self._publish_all_markers(msg.header)
                return

        eps = 0.10  # 10 cm duplicate threshold
        for (x_opt, y_opt, z_opt) in centroids_opt:
            # optical → camera_link
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
                p_world = do_transform_point(p_cam, tf_cam_to_world)
            except TransformException as e:
                self.get_logger().error(f"[ConeLandmarkMapper] TF transform failed: {e}")
                continue

            world_pt = [p_world.point.x, p_world.point.y, p_world.point.z]

            # If this world_pt is not already in known_cones (within eps), register it:
            duplicate = any(
                np.linalg.norm(np.array(world_pt) - np.array(existing_pt)) < eps
                for (existing_pt, _, _, _) in self.known_cones
            )
            if not duplicate:
                # Append a new “unvalidated” entry:
                # [position, class_id=None, color=gray, is_validated=False]
                self.known_cones.append((world_pt, None, self.DEFAULT_COLOR, False))

        # 5b) Immediately publish all markers (all spheres + any existing cylinders)
        #      so that new clusters show up as gray spheres at once.
        self._publish_all_markers(msg.header)

        # 6) Now attempt to validate centroids via YOLO:
        fx = self.latest_caminfo.k[0]
        fy = self.latest_caminfo.k[4]
        cx = self.latest_caminfo.k[2]
        cy = self.latest_caminfo.k[5]

        # Project only centroids that are in front (z_opt > 0):
        z_opt_arr = centroids_opt[:, 2]
        forward_mask = (z_opt_arr > 0.0)
        if not np.any(forward_mask):
            # no centroids in front → no validation → done
            return

        valid_centroids = centroids_opt[forward_mask]
        idx_map = np.nonzero(forward_mask)[0]

        x_opt = valid_centroids[:, 0]
        y_opt = valid_centroids[:, 1]
        z_opt = valid_centroids[:, 2]
        u_proj = (fx * x_opt / z_opt) + cx
        v_proj = (fy * y_opt / z_opt) + cy

        # Build YOLO boxes & confidences
        M = len(self.latest_detections)
        if M == 0:
            return

        BOXES = np.zeros((M, 4), dtype=np.float32)
        CONFIDENCES = np.zeros((M,), dtype=np.float32)
        DET_IDS = ["" for _ in range(M)]
        CLASS_IDS = np.full((M,), -1, dtype=np.int32)

        for i, box in enumerate(self.latest_detections):
            u0 = box['u_center'] - (box['width']  / 2.0)
            u1 = box['u_center'] + (box['width']  / 2.0)
            v0 = box['v_center'] - (box['height'] / 2.0)
            v1 = box['v_center'] + (box['height'] / 2.0)

            BOXES[i, 0] = u0
            BOXES[i, 1] = u1
            BOXES[i, 2] = v0
            BOXES[i, 3] = v1

            CLASS_IDS[i] = -1 if box.get('class_id') is None else int(box['class_id'])
            DET_IDS[i] = box.get('det_id', '') or f"idx_{i:d}"
            CONFIDENCES[i] = float(box.get('score', 0.0))

        # Check “inside box?” for each centroid (vectorized)
        u_col = u_proj[:, np.newaxis]
        v_col = v_proj[:, np.newaxis]
        b_u0 = BOXES[np.newaxis, :, 0]
        b_u1 = BOXES[np.newaxis, :, 1]
        b_v0 = BOXES[np.newaxis, :, 2]
        b_v1 = BOXES[np.newaxis, :, 3]

        inside_u = np.logical_and(u_col >= b_u0, u_col <= b_u1)
        inside_v = np.logical_and(v_col >= b_v0, v_col <= b_v1)
        inside_any = np.logical_and(inside_u, inside_v)

        matched_indices = []
        for row_idx in range(inside_any.shape[0]):
            box_idxs_for_this_centroid = np.nonzero(inside_any[row_idx, :])[0]
            if box_idxs_for_this_centroid.size == 0:
                continue
            best_box_idx = box_idxs_for_this_centroid[
                np.argmax(CONFIDENCES[box_idxs_for_this_centroid])
            ]
            matched_indices.append((row_idx, int(best_box_idx)))

        if len(matched_indices) == 0:
            return

        # 7) For each matched (opt_idx, box_idx), validate that centroid:
        for (local_idx, box_idx) in matched_indices:
            cid = int(CLASS_IDS[box_idx])
            det_id = DET_IDS[box_idx]
            color = self.CLASS_ID_TO_COLOR.get(cid, self.DEFAULT_COLOR)
            opt_idx = idx_map[local_idx]

            # Centroid coordinates in optical
            x_opt, y_opt, z_opt = centroids_opt[opt_idx]

            # optical → camera_link
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
                p_world = do_transform_point(p_cam, tf_cam_to_world)
            except TransformException as e:
                self.get_logger().error(f"[ConeLandmarkMapper] TF transform failed: {e}")
                continue

            # Find which entry in self.known_cones matches this world_pt (within eps)
            world_pt = [p_world.point.x, p_world.point.y, p_world.point.z]
            for idx, (existing_pt, existing_cid, existing_color, is_validated) in enumerate(self.known_cones):
                if np.linalg.norm(np.array(world_pt) - np.array(existing_pt)) < eps:
                    # Validate it:
                    if not is_validated:
                        self.known_cones[idx] = (
                            existing_pt,        # same position
                            cid,                # new class_id
                            color,              # new color
                            True                # now validated
                        )
                    break

        # 8) Publish again so spheres turn into cylinders
        self._publish_all_markers(msg.header)

    def _publish_all_markers(self, header: Header):
        marker_array   = MarkerArray()
        TEXT_ID_OFFSET = 1000

        # build unified marker array
        for idx, (pt, cls_id, color, is_validated) in enumerate(self.known_cones):
            # shape marker
            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp    = header.stamp
            m.ns              = 'cone_landmarks'
            m.id              = idx
            m.action          = Marker.ADD
            if not is_validated:
                m.type = Marker.SPHERE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = pt
                m.scale.x = m.scale.y = m.scale.z = 0.2
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 0.8
            else:
                diameter, _, height = self.CLASS_ID_TO_SIZE.get(cls_id, (0.2,0.2,0.2))
                m.type = Marker.CYLINDER
                m.pose.position.x, m.pose.position.y = pt[0], pt[1]
                m.pose.position.z = pt[2] + height/2.0
                m.scale.x = m.scale.y = diameter; m.scale.z = height
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 0.8
            marker_array.markers.append(m)

            # text marker
            t = Marker()
            t.header.frame_id = self.world_frame
            t.header.stamp    = header.stamp
            t.ns              = 'cone_id_text'
            t.id              = idx + TEXT_ID_OFFSET
            t.type            = Marker.TEXT_VIEW_FACING
            t.action          = Marker.ADD
            if not is_validated:
                t.pose.position.z = pt[2] + 0.25
            else:
                _, _, height = self.CLASS_ID_TO_SIZE.get(cls_id, (0.2,0.2,0.2))
                t.pose.position.z = pt[2] + height + 0.05
            t.pose.position.x, t.pose.position.y = pt[0], pt[1]
            t.scale.z = 0.2
            t.color.r = t.color.g = t.color.b = 1.0; t.color.a = 0.9
            t.text = str(idx)
            marker_array.markers.append(t)

        # publish unified
        self.marker_pub.publish(marker_array)

        # publish blue & yellow subsets
        blue_ids   = {i for i,(pt,cid,col,val) in enumerate(self.known_cones) if val and cid == 0}
        yellow_ids = {i for i,(pt,cid,col,val) in enumerate(self.known_cones) if val and cid in (2,4)}

        blue_ma   = MarkerArray()
        yellow_ma = MarkerArray()
        for m in marker_array.markers:
            base_id = m.id if m.id < TEXT_ID_OFFSET else m.id - TEXT_ID_OFFSET
            if base_id in blue_ids:
                blue_ma.markers.append(m)
            if base_id in yellow_ids:
                yellow_ma.markers.append(m)
        self.blue_pub.publish(blue_ma)
        self.yellow_pub.publish(yellow_ma)


def main(args=None):
    rclpy.init(args=args)
    node = ConeLandmarkMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
