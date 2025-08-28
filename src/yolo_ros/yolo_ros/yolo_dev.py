#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance, PoseArray
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

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
        super().__init__('yolo_ros_node')
        self.bridge = CvBridge()

        # Camera intrinsics (filled once)
        self.fx = self.fy = self.cx = self.cy = None

        # Real cone heights (m) by YOLO class ID
        self.height_map = {
            '0': 0.335,  # blue
            '4': 0.335,  # yellow
            '2': 0.335,  # small orange
            '1': 0.475,  # large orange
        }
        # Overlay colors (BGR)
        self.color_map = {
            '0': (255, 0, 0),      # blue
            '4': (0, 255, 255),    # yellow
            '2': (0, 165, 255),    # small orange
            '1': (0, 0, 255),      # large orange (red)
        }
        # IoU threshold to filter overlapping detections
        self.iou_thresh = 0.3

        # Threshold on camera-frame Y below which any cone is classified as large
        self.large_cone_y_thresh = 0.65  # meters; tune to your setup

        # Subscribers
        # self.image_sub = self.create_subscription(
        #     Image, '/camera/image_raw', self.image_callback, 10)
        # self.info_sub  = self.create_subscription(
        #     CameraInfo, '/camera/camera_info', self.caminfo_callback, 10)

        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.caminfo_callback, 10
        )


        # Publishers
        self.image_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/yolo/camera_info', 10)
        self.det_pub   = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.pose_pub  = self.create_publisher(PoseArray, '/yolo/cone_poses', 10)

        # Load YOLO model
        self.model = YOLO(
            '/home/jay/Documents/yolo11-tutorial/runs/detect/train17/weights/best.pt'
        )
        self.get_logger().info("YOLO node up and running!")

    def caminfo_callback(self, info_msg: CameraInfo):
        # Store camera intrinsics from first message
        if self.fx is None:
            self.fx, self.fy = info_msg.k[0], info_msg.k[4]
            self.cx, self.cy = info_msg.k[2], info_msg.k[5]
            self.get_logger().info(
                f"Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        # Forward camera info
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # Wait for intrinsics
        if None in (self.fx, self.fy, self.cx, self.cy):
            self.get_logger().warning("No camera intrinsics yet.")
            return

        # Convert ROS Image to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        vis = cv_img.copy()
        frame_h, frame_w = cv_img.shape[:2]

        # Run YOLO inference
        results = self.model(cv_img)

        # Prepare detection and pose arrays
        det_arr = Detection2DArray()
        det_arr.header = img_msg.header
        pose_arr = PoseArray()
        pose_arr.header = img_msg.header

        accepted = []  # list of accepted bbox tuples

        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            # 1) Remove any box that touches the image border
            if x1 <= 0 or y1 <= 0 or x2 >= frame_w or y2 >= frame_h:
                continue

            # 2) Skip overlapping detections
            if any(compute_iou((x1, y1, x2, y2), b) > self.iou_thresh for b in accepted):
                continue
            accepted.append((x1, y1, x2, y2))

            # Original YOLO class & pixel dims
            cls_id = str(int(box.cls[0].item()))
            w_pix, h_pix = x2 - x1, y2 - y1
            u, v = (x1 + x2) / 2, (y1 + y2) / 2

            # First-pass depth estimate using detected class height
            real_h = self.height_map.get(cls_id)
            if real_h is None or h_pix <= 0:
                continue
            z = (self.fy * real_h) / h_pix
            y_cam = ((v - self.cy) / self.fy) * z

            # Force to large-orange if below Y threshold
            if y_cam < self.large_cone_y_thresh:
                cls_id = '1'
                real_h = self.height_map['1']
                z = (self.fy * real_h) / h_pix
                # recompute y_cam
                y_cam = ((v - self.cy) / self.fy) * z

            # Print y_cam for debugging
            self.get_logger().info(f"Detected cone → y_cam = {y_cam:.3f} m, class = {cls_id}")

            # Final 3D coordinates
            x_cam = ((u - self.cx) / self.fx) * z

            # Draw bounding box and depth text
            color = self.color_map.get(cls_id, (255, 255, 255))
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, f"{z:.2f}m", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # Build Detection2D
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
            hyp.hypothesis.score = float(box.conf[0].item())

            # Pose in camera optical frame
            pose_cov = PoseWithCovariance()
            pose_cov.pose.position.x = float(x_cam)
            pose_cov.pose.position.y = float(y_cam)
            pose_cov.pose.position.z = float(z)
            pose_cov.pose.orientation.w = 1.0
            hyp.pose = pose_cov

            det.results = [hyp]
            det_arr.detections.append(det)
            pose_arr.poses.append(pose_cov.pose)

        # Publish annotated image
        out = self.bridge.cv2_to_imgmsg(vis, 'bgr8')
        out.header = img_msg.header
        self.image_pub.publish(out)
        # Publish detections and poses
        self.det_pub.publish(det_arr)
        self.pose_pub.publish(pose_arr)

def main(args=None):
    rclpy.init(args=args)
    node = YoloRosNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

