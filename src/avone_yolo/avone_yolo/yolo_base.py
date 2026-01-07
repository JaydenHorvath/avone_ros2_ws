#!/usr/bin/env python3

# YOLO ROS Base Node (Barebones)
# File: yolo_base.py

# Purpose:
#   - Subscribe to a ROS 2 camera stream (Image + CameraInfo).
#   - Run Ultralytics YOLO inference on each incoming image.
#   - Publish:
#       1) An annotated image with bounding boxes drawn
#       2) The CameraInfo forwarded (pass-through)
#       3) A Detection2DArray (vision_msgs) containing 2D bounding boxes + class/score

# Topics (default in this file):
#   Subscribes:
#     - /camera/image_raw          (sensor_msgs/Image)
#     - /camera/camera_info        (sensor_msgs/CameraInfo)

#   Publishes:
#     - /yolo/image                (sensor_msgs/Image)         annotated image
#     - /yolo/camera_info          (sensor_msgs/CameraInfo)    forwarded camera info
#     - /yolo/detections           (vision_msgs/Detection2DArray)

# Model:
#   - Loads best.pt from the yolo_ros package share directory:
#       avone_yolo/weights/best.pt
#   - This model was trained on YOLO11n using the FSCOCO Dataset with 100 epochs

# Notes:
#   - This is the simplest "base" iteration: it does not do cone color mapping,
#     does not fuse depth, and does not estimate 3D positions.
#   - Detection2D results contain:
#       - bbox center (pixel x,y) and bbox size (pixel width/height)
#       - class_id as a string of the numeric YOLO class index
#       - score as YOLO confidence
#       - pose left as default/zero (PoseWithCovariance), since this is 2D-only
#   - If you want to use the RealSense topics instead, the alternative subscriber
#     block is already present but commented.

# How to run:
#   ros2 run yolo_ros yolo_base


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseWithCovariance
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import os

# Model path
pkg_share = get_package_share_directory("avone_yolo")
model_path = os.path.join(pkg_share, "weights", "best.pt")


class YoloRosNode(Node):
    def __init__(self):
        super().__init__("yolo_ros_node")

        # Bridge: ROS Image <-> OpenCV
        self.bridge = CvBridge()

        # Default camera topics
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.caminfo_callback, 10
        )

        # RealSense D435 Camera Topics
        # self.image_sub = self.create_subscription(
        #     Image, '/camera/camera/color/image_raw', self.image_callback, 10
        # )
        # self.info_sub = self.create_subscription(
        #     CameraInfo, '/camera/camera/color/camera_info', self.caminfo_callback, 10
        # )

        # Publishers
        self.image_pub = self.create_publisher(Image, "/yolo/image", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/yolo/camera_info", 10)
        self.det_pub = self.create_publisher(Detection2DArray, "/yolo/detections", 10)

        # load your YOLO model
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO node up and running!")

    def caminfo_callback(self, info_msg: CameraInfo):
        # forward CameraInfo to /yolo/camera_info
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # 1) run inference
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        results = self.model(cv_img)  # list of ultralytics Results
        annotated = results[0].plot()  # numpy array with boxes drawn

        # 2) publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = img_msg.header
        self.image_pub.publish(out_msg)

        # 3) build Detection2DArray
        det_arr = Detection2DArray()
        det_arr.header = img_msg.header

        for box in results[0].boxes:  # each box has `.xyxy`, `.cls`, `.conf`
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w = x2 - x1
            h = y2 - y1

            det = Detection2D()
            det.header = img_msg.header

            # bbox
            bb = BoundingBox2D()
            bb.center.position.x = float((x1 + x2) / 2.0)
            bb.center.position.y = float((y1 + y2) / 2.0)
            bb.size_x = float(w)
            bb.size_y = float(h)
            det.bbox = bb

            # result (class label & score)
            # create the hypothesis‐with‐pose message
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(box.cls[0].item()))
            hyp.hypothesis.score = float(box.conf[0].item())

            # optional: leave pose at zero (you’re just using the id+score)
            hyp.pose = PoseWithCovariance()

            det.results = [hyp]

            det_arr.detections.append(det)

        # 4) publish detections
        self.det_pub.publish(det_arr)


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
