#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseWithCovariance

from ultralytics import YOLO


class YoloRosNode(Node):
    def __init__(self):
        super().__init__("yolo_ros_node")
        self.bridge = CvBridge()

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # If set, node publishes from this image path instead of subscribing (jpg/png)
        self.declare_parameter("image_path", "")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("frame_id", "camera")

        self.declare_parameter(
            "weights_path",
            "/home/jay/Documents/yolo11-tutorial/runs/detect/train17/weights/best.pt",
        )

        # Output sizing for /yolo/image
        # Typical use: set output_height:=1080 and leave output_width:=0 for auto width
        self.declare_parameter("enable_output_resize", True)
        self.declare_parameter("output_height", 1080)   # set to 0 to disable height constraint
        self.declare_parameter("output_width", 0)       # 0 means auto-compute if keep_aspect is True
        self.declare_parameter("keep_aspect", True)
        self.declare_parameter("letterbox", False)      # if True and both w/h given, pads to fit instead of stretching

        # -----------------------------
        # Display tuning
        # -----------------------------
        self.draw_boxes = True
        self.draw_text = False          # no labels
        self.box_thickness = 2

        # Your class-id -> BGR colour mapping
        self.color_map = {
            "0": (255, 0, 0),      # blue
            "4": (0, 255, 255),    # yellow
            "2": (0, 165, 255),    # small orange
            "1": (0, 0, 255),      # large orange (red)
        }
        self.default_color = (0, 255, 0)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.image_pub = self.create_publisher(Image, "/yolo/image", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/yolo/camera_info", 10)
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

        # -----------------------------
        # YOLO model
        # -----------------------------
        weights_path = self.get_parameter("weights_path").get_parameter_value().string_value
        self.model = YOLO(weights_path)

        # For live mode: store latest CameraInfo
        self.latest_caminfo = None

        # -----------------------------
        # Mode selection: live vs image file
        # -----------------------------
        self.image_path = self.get_parameter("image_path").get_parameter_value().string_value.strip()

        if self.image_path:
            if not os.path.exists(self.image_path):
                raise FileNotFoundError(f"image_path does not exist: {self.image_path}")

            self.static_cv_img = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
            if self.static_cv_img is None:
                raise RuntimeError(f"Failed to read image_path with OpenCV: {self.image_path}")

            rate_hz = float(self.get_parameter("publish_rate_hz").get_parameter_value().double_value)
            rate_hz = max(0.1, rate_hz)
            period = 1.0 / rate_hz

            self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
            self.timer = self.create_timer(period, self.timer_callback)

            self.get_logger().info(f"YOLO node up and running in FILE mode: {self.image_path}")
        else:
            image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
            caminfo_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value

            self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
            self.info_sub = self.create_subscription(CameraInfo, caminfo_topic, self.caminfo_callback, 10)

            self.get_logger().info(f"YOLO node up and running in LIVE mode, subscribing to {image_topic}")

    def caminfo_callback(self, info_msg: CameraInfo):
        self.latest_caminfo = info_msg
        self.info_pub.publish(info_msg)

    def _pick_color(self, cls_id_int: int):
        return self.color_map.get(str(cls_id_int), self.default_color)

    def _compute_output_size(self, in_w: int, in_h: int):
        enable = bool(self.get_parameter("enable_output_resize").get_parameter_value().bool_value)
        if not enable:
            return in_w, in_h, 1.0, 1.0, 0, 0, False

        out_h = int(self.get_parameter("output_height").get_parameter_value().integer_value)
        out_w = int(self.get_parameter("output_width").get_parameter_value().integer_value)
        keep_aspect = bool(self.get_parameter("keep_aspect").get_parameter_value().bool_value)
        letterbox = bool(self.get_parameter("letterbox").get_parameter_value().bool_value)

        # If both are zero or negative, treat as disabled
        if out_w <= 0 and out_h <= 0:
            return in_w, in_h, 1.0, 1.0, 0, 0, False

        # Auto-compute missing dimension if keeping aspect
        if keep_aspect:
            if out_w <= 0 and out_h > 0:
                scale = out_h / float(in_h)
                out_w = int(round(in_w * scale))
                return out_w, out_h, scale, scale, 0, 0, False

            if out_h <= 0 and out_w > 0:
                scale = out_w / float(in_w)
                out_h = int(round(in_h * scale))
                return out_w, out_h, scale, scale, 0, 0, False

            # Both provided
            if letterbox:
                scale = min(out_w / float(in_w), out_h / float(in_h))
                new_w = int(round(in_w * scale))
                new_h = int(round(in_h * scale))
                pad_x = int((out_w - new_w) // 2)
                pad_y = int((out_h - new_h) // 2)
                return out_w, out_h, scale, scale, pad_x, pad_y, True

            # keep_aspect but not letterbox means stretch to exact output
            sx = out_w / float(in_w)
            sy = out_h / float(in_h)
            return out_w, out_h, sx, sy, 0, 0, False

        # Not keeping aspect: require both, otherwise fill missing with input
        if out_w <= 0:
            out_w = in_w
        if out_h <= 0:
            out_h = in_h
        sx = out_w / float(in_w)
        sy = out_h / float(in_h)
        return out_w, out_h, sx, sy, 0, 0, False

    def _resize_for_output(self, img_bgr):
        in_h, in_w = img_bgr.shape[:2]
        out_w, out_h, sx, sy, pad_x, pad_y, do_letterbox = self._compute_output_size(in_w, in_h)

        if out_w == in_w and out_h == in_h and pad_x == 0 and pad_y == 0:
            return img_bgr

        if do_letterbox:
            scale = sx  # sx == sy in letterbox path
            new_w = int(round(in_w * scale))
            new_h = int(round(in_h * scale))
            resized = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

            canvas = np.zeros((out_h, out_w, 3), dtype=img_bgr.dtype)
            y0 = pad_y
            x0 = pad_x
            canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
            return canvas

        return cv2.resize(img_bgr, (out_w, out_h), interpolation=cv2.INTER_LINEAR)

    def _publish_basic_caminfo(self, header, width: int, height: int):
        info = CameraInfo()
        info.header = header
        info.width = int(width)
        info.height = int(height)
        self.info_pub.publish(info)

    def _process_and_publish(self, cv_img, header, publish_basic_caminfo=False):
        # 1) run inference
        results = self.model(cv_img, verbose=False)
        r0 = results[0]
        boxes = getattr(r0, "boxes", None)

        # 2) draw boxes only, colour-coded, no labels
        annotated = cv_img.copy()
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cls_id = int(box.cls[0].item())
                color = self._pick_color(cls_id)

                if self.draw_boxes:
                    cv2.rectangle(
                        annotated,
                        (x1, y1),
                        (x2, y2),
                        color,
                        thickness=self.box_thickness,
                    )

                if self.draw_text:
                    conf = float(box.conf[0].item())
                    label = f"{cls_id} {conf:.2f}"
                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.45,
                        color,
                        thickness=1,
                        lineType=cv2.LINE_AA,
                    )

        # 2.5) resize annotated output for /yolo/image
        annotated_out = self._resize_for_output(annotated)

        # publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated_out, encoding="bgr8")
        out_msg.header = header
        self.image_pub.publish(out_msg)

        # Optional: in file mode, publish a basic CameraInfo matching output image size
        if publish_basic_caminfo:
            h, w = annotated_out.shape[:2]
            self._publish_basic_caminfo(header, w, h)

        # 3) build Detection2DArray (kept in original input image coordinates)
        det_arr = Detection2DArray()
        det_arr.header = header

        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                w = x2 - x1
                h = y2 - y1

                det = Detection2D()
                det.header = header

                bb = BoundingBox2D()
                bb.center.position.x = float((x1 + x2) / 2.0)
                bb.center.position.y = float((y1 + y2) / 2.0)
                bb.size_x = float(w)
                bb.size_y = float(h)
                det.bbox = bb

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(box.cls[0].item()))
                hyp.hypothesis.score = float(box.conf[0].item())
                hyp.pose = PoseWithCovariance()

                det.results = [hyp]
                det_arr.detections.append(det)

        self.det_pub.publish(det_arr)

    def timer_callback(self):
        # FILE mode
        header = Image().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        self._process_and_publish(self.static_cv_img, header, publish_basic_caminfo=True)

    def image_callback(self, img_msg: Image):
        # LIVE mode
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        self._process_and_publish(cv_img, img_msg.header, publish_basic_caminfo=False)


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
