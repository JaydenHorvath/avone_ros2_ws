#!/usr/bin/env python3
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
)
from geometry_msgs.msg import PoseWithCovariance
from cv_bridge import CvBridge
from ultralytics import YOLO

# Optional: GPU / half-precision
try:
    import torch
    _TORCH_OK = True
except Exception:
    _TORCH_OK = False


def round_up_to_multiple(n: int, m: int) -> int:
    """Round up n to the nearest multiple of m."""
    return ((n + m - 1) // m) * m


class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_node')
        self.bridge = CvBridge()

        # ---------------- Parameters ----------------
        # Camera topics
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')

        # Desired input size (will be rounded up to multiples of 32 for YOLO)
        self.declare_parameter('img_width', 1920)
        self.declare_parameter('img_height', 1080)

        # Model + inference params
        self.declare_parameter('model_path', '/home/jay/Documents/yolo11-tutorial/runs/detect/train17/weights/best.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('use_gpu', True)     # set False to force CPU
        self.declare_parameter('use_fp16', True)    # only effective if GPU available

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        caminfo_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.img_w = int(self.get_parameter('img_width').value)
        self.img_h = int(self.get_parameter('img_height').value)
        self.conf = float(self.get_parameter('conf').value)
        self.iou = float(self.get_parameter('iou').value)
        want_gpu = bool(self.get_parameter('use_gpu').value)
        want_fp16 = bool(self.get_parameter('use_fp16').value)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Round to nearest /32 to prevent internal resizing surprises
        self.imgsz: Tuple[int, int] = (
            round_up_to_multiple(self.img_h, 32),
            round_up_to_multiple(self.img_w, 32),
        )

        # ---------------- Subs/Pubs ----------------
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, caminfo_topic, self.caminfo_callback, 10)

        self.image_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/yolo/camera_info', 10)
        self.det_pub   = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # ---------------- Model ----------------
        self.model = YOLO(model_path)

        # Pick device
        self.device = 'cpu'
        self.half = False
        if _TORCH_OK and want_gpu and torch.cuda.is_available():
            self.device = 'cuda:0'
            self.half = want_fp16
        # Note: Ultralytics v8 picks device from args each call or from model.to()
        try:
            # Move model once; half precision only valid on CUDA
            self.model.to(self.device)
            if self.half:
                self.model.model.half()  # won’t error on non-CUDA if self.half=False
        except Exception as e:
            self.get_logger().warn(f'Could not move model to {self.device} / FP16={self.half}: {e}')
            self.device = 'cpu'
            self.half = False

        self.get_logger().info(
            f"YOLO node up. Target input ~{self.img_w}x{self.img_h} "
            f"(rounded to {self.imgsz[1]}x{self.imgsz[0]}), device={self.device}, fp16={self.half}"
        )

    # Just forward CameraInfo
    def caminfo_callback(self, info_msg: CameraInfo):
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # Convert to OpenCV BGR8
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # If your camera already publishes 1920x1080, you’re set.
        # YOLO will internally letterbox to imgsz=(H32,W32) but map boxes back to original size.
        # If the camera publishes lower res, you can’t conjure real 1080p detail by upscaling;
        # leave this as-is to avoid fake “1080p”.
        try:
            results = self.model(
                cv_img,
                imgsz=self.imgsz,     # (H, W), stride-aligned
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                half=self.half,
                verbose=False
            )
        except TypeError:
            # Some Ultralytics builds prefer .predict(...)
            results = self.model.predict(
                source=cv_img,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                half=self.half,
                verbose=False
            )

        res = results[0]

        # Annotated image (same size as original frame)
        annotated = res.plot()

        # Publish annotated image
        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = img_msg.header
        self.image_pub.publish(out)

        # Build Detection2DArray in original pixel coordinates
        det_arr = Detection2DArray()
        det_arr.header = img_msg.header

        if res.boxes is not None:
            for box in res.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                w = x2 - x1
                h = y2 - y1

                det = Detection2D()
                det.header = img_msg.header

                bb = BoundingBox2D()
                bb.center.position.x = (x1 + x2) / 2.0
                bb.center.position.y = (y1 + y2) / 2.0
                bb.size_x = w
                bb.size_y = h
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                # Class index as string; map to label if you have names
                hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                hyp.hypothesis.score = float(box.conf[0].item())
                hyp.pose = PoseWithCovariance()
                det.results = [hyp]

                det_arr.detections.append(det)

        self.det_pub.publish(det_arr)


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
