#!/usr/bin/env python3
"""
AVONE YOLO Node (Dev Version with Cone Position Estimation + Detection2DArray)
File: yolo_dev.py

Purpose:
  - Run Ultralytics YOLO on a camera stream and estimate approximate 3D cone positions
    in the CAMERA frame using a pinhole camera model + known cone height.
  - Publish:
      1) /yolo/image          Annotated image with coloured boxes + estimated range text (z meters)
      2) /yolo/camera_info    Forwarded CameraInfo (also used to extract intrinsics fx/fy/cx/cy)
      3) /yolo/detections     Detection2DArray populated with bbox + class_id + score + pose estimate
      4) /yolo/cone_poses     PoseArray of estimated cone poses (camera frame)
      5) /yolo/cone_markers   MarkerArray of cylinders for RViz visualization (camera frame)

How position estimation works:
  - Uses CameraInfo intrinsics (fx, fy, cx, cy).
  - Uses known real cone heights per class (height_map).
  - Uses bounding box pixel height h_pix to estimate depth (range) via:
        z = (fy * real_height) / h_pix
  - Estimates lateral/vertical offsets in camera coordinates:
        x_cam = ((u - cx) / fx) * z
        y_cam = ((v - cy) / fy) * z
    where (u, v) is the bbox center pixel coordinate.

Coordinate mapping used in this file:
  - The estimated pose is written with a custom axis mapping:
        pose.x = z
        pose.y = -x_cam
        pose.z = -y_cam

Filtering and heuristics:
  - Rejects boxes touching image borders (likely partial detections).
  - Performs basic duplicate suppression using IoU threshold (iou_thresh).
  - Applies a heuristic "large cone override" if y_cam is below a threshold
    (large_cone_y_thresh), forcing class_id to '1' and re-estimating depth.

How to run:
  ros2 run yolo_ros yolo_dev

  # Override weights via parameter
  ros2 run yolo_ros yolo_dev --ros-args -p weights_path:=/path/to/best.pt

Notes:
  - Detection2DArray is now populated. Each Detection2D contains:
      - bbox in image coordinates
      - results[0].hypothesis.class_id and score
      - results[0].pose (PoseWithCovariance) filled with the estimated cone pose
"""

import rclpy
from rclpy.node import Node
import os

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


def compute_iou(box_a, box_b):
    x1_a, y1_a, x2_a, y2_a = box_a
    x1_b, y1_b, x2_b, y2_b = box_b
    xi1, yi1 = max(x1_a, x1_b), max(y1_a, y1_b)
    xi2, yi2 = min(x2_a, x2_b), min(y2_a, y2_b)
    inter_w, inter_h = max(0, xi2 - xi1), max(0, yi2 - yi1)
    inter_area = inter_w * inter_h
    area_a = (x2_a - x1_a) * (y2_a - y1_a)
    area_b = (x2_b - x1_b) * (y2_b - y1_b)
    union = area_a + area_b - inter_area
    return inter_area / union if union > 0 else 0


class YoloRosNode(Node):
    def __init__(self):
        super().__init__("yolo_ros_node")
        self.bridge = CvBridge()

        # Camera intrinsics (set once from CameraInfo)
        self.fx = self.fy = self.cx = self.cy = None

        # Real cone heights (m)
        self.height_map = {
            "0": 0.335,  # blue
            "4": 0.335,  # yellow
            "2": 0.335,  # small orange
            "1": 0.475,  # large orange
        }

        # Cone colours (BGR for OpenCV)
        self.color_map = {
            "0": (255, 0, 0),  # blue
            "4": (0, 255, 255),  # yellow
            "2": (0, 165, 255),  # small orange
            "1": (0, 0, 255),  # large orange (red)
        }

        # Same colours converted to RGB for RViz markers
        self.marker_color_map = {
            "0": (0.0, 0.0, 1.0),  # blue
            "4": (1.0, 1.0, 0.0),  # yellow
            "2": (1.0, 0.5, 0.0),  # small orange
            "1": (1.0, 0.0, 0.0),  # large orange
        }

        # Duplicate suppression threshold (IoU)
        self.iou_thresh = 0.3

        # Heuristic threshold used to force large cone classification
        self.large_cone_y_thresh = 0.65

        # Subscribers (hard-coded topics)
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.caminfo_callback, 10
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, "/yolo/image", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/yolo/camera_info", 10)
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)
        self.pose_pub = self.create_publisher(PoseArray, "/yolo/cone_poses", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/yolo/cone_markers", 10)

        # ---------------------------
        # YOLO model weights (PACKAGE)
        # ---------------------------
        default_weights = os.path.join(
            get_package_share_directory("avone_yolo"),
            "weights",
            "best.pt",
        )
        self.declare_parameter("weights_path", default_weights)
        weights_path = (
            self.get_parameter("weights_path").get_parameter_value().string_value
        )

        if not os.path.isfile(weights_path):
            raise FileNotFoundError(
                f"YOLO weights not found: {weights_path}. "
                f"Expected something like: {default_weights}"
            )

        self.get_logger().info(f"Loading YOLO weights from: {weights_path}")
        self.model = YOLO(weights_path)

        self.get_logger().info(
            "YOLO dev node up and running (detections + cone pose estimation)!"
        )

    def caminfo_callback(self, info_msg: CameraInfo):
        # Extract intrinsics once (K matrix)
        if self.fx is None:
            self.fx, self.fy = info_msg.k[0], info_msg.k[4]
            self.cx, self.cy = info_msg.k[2], info_msg.k[5]
            self.get_logger().info(
                f"Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
            )

        # Forward CameraInfo for other nodes / debugging
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # Require intrinsics before doing any work
        if None in (self.fx, self.fy, self.cx, self.cy):
            return

        # Convert ROS Image -> OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        vis = cv_img.copy()
        frame_h, frame_w = cv_img.shape[:2]

        # Run YOLO
        results = self.model(cv_img)

        # Outputs
        det_arr = Detection2DArray()
        det_arr.header = img_msg.header

        pose_arr = PoseArray()
        pose_arr.header = img_msg.header

        marker_arr = MarkerArray()

        # Track accepted boxes to reject duplicates using IoU
        accepted = []
        marker_id = 0

        for box in results[0].boxes:
            # Box corners in pixels
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            # Reject boxes touching image border (often partial / unstable)
            if x1 <= 0 or y1 <= 0 or x2 >= frame_w or y2 >= frame_h:
                continue

            # Reject duplicates if IoU with any accepted box is too high
            if any(
                compute_iou((x1, y1, x2, y2), b) > self.iou_thresh for b in accepted
            ):
                continue
            accepted.append((x1, y1, x2, y2))

            # Class id as string (may be overridden by heuristic below)
            cls_id = str(int(box.cls[0].item()))
            conf = float(box.conf[0].item())

            # Pixel width/height of bbox
            w_pix = x2 - x1
            h_pix = y2 - y1

            # Pixel center of bbox
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0

            # Lookup real cone height
            real_h = self.height_map.get(cls_id)
            if real_h is None or h_pix <= 0:
                continue

            # ------------------------------------------------------------
            # Range estimate from bbox height (pinhole camera)
            # ------------------------------------------------------------
            z = (self.fy * real_h) / float(h_pix)

            # Project center pixel into camera coordinates
            y_cam = ((v - self.cy) / self.fy) * z

            # ------------------------------------------------------------
            # Heuristic: force large cone if "low in image" (as coded here)
            # ------------------------------------------------------------
            if y_cam < self.large_cone_y_thresh:
                cls_id = "1"
                real_h = self.height_map["1"]
                z = (self.fy * real_h) / float(h_pix)
                y_cam = ((v - self.cy) / self.fy) * z

            x_cam = ((u - self.cx) / self.fx) * z

            # ------------------------------------------------------------
            # Visualize bbox + range text
            # ------------------------------------------------------------
            color = self.color_map.get(cls_id, (255, 255, 255))
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                vis,
                f"{z:.2f}m",
                (x1, max(0, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

            # ------------------------------------------------------------
            # Pose output (PoseArray) with your custom axis mapping
            # ------------------------------------------------------------
            pose_cov = PoseWithCovariance()
            pose_cov.pose.position.x = float(z)
            pose_cov.pose.position.y = float(-x_cam)
            pose_cov.pose.position.z = float(-y_cam)
            pose_cov.pose.orientation.w = 1.0

            pose_arr.poses.append(pose_cov.pose)

            # ------------------------------------------------------------
            # Detection2D output (bbox + class/score + pose)
            # ------------------------------------------------------------
            det = Detection2D()
            det.header = img_msg.header

            bb = BoundingBox2D()
            bb.center.position.x = float(u)
            bb.center.position.y = float(v)
            bb.size_x = float(w_pix)
            bb.size_y = float(h_pix)
            det.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls_id
            hyp.hypothesis.score = conf

            # Attach the estimated 3D pose (camera frame) into the hypothesis pose field
            hyp.pose = pose_cov

            det.results = [hyp]
            det_arr.detections.append(det)

            # ------------------------------------------------------------
            # RViz marker (cylinder) for visualization
            # ------------------------------------------------------------
            marker = Marker()
            marker.header = img_msg.header
            marker.ns = "cones"
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose_cov.pose

            marker.scale.x = 0.25
            marker.scale.y = 0.25
            marker.scale.z = real_h

            r, g, b = self.marker_color_map.get(cls_id, (1.0, 1.0, 1.0))
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            marker_arr.markers.append(marker)
            marker_id += 1

        # ------------------------------------------------------------
        # Publish outputs
        # ------------------------------------------------------------
        out = self.bridge.cv2_to_imgmsg(vis, "bgr8")
        out.header = img_msg.header
        self.image_pub.publish(out)

        self.det_pub.publish(det_arr)
        self.pose_pub.publish(pose_arr)
        self.marker_pub.publish(marker_arr)


def main(args=None):
    rclpy.init(args=args)
    node = YoloRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
