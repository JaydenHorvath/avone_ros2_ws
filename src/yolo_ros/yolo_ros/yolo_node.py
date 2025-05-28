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
from geometry_msgs.msg import PoseWithCovariance
from cv_bridge import CvBridge
from ultralytics import YOLO

class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_node')
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/rgbd/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/rgbd/camera_info', self.caminfo_callback, 10
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, '/yolo/image', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/yolo/camera_info', 10)
        self.det_pub   = self.create_publisher(Detection2DArray, '/yolo/detections', 10)

        # load your YOLO model
        self.model = YOLO('/home/jay/Documents/yolo11-tutorial/runs/detect/train17/weights/best.pt')
        self.get_logger().info("YOLO node up and running!")

    def caminfo_callback(self, info_msg: CameraInfo):
        # forward CameraInfo to /yolo/camera_info
        self.info_pub.publish(info_msg)

    def image_callback(self, img_msg: Image):
        # 1) run inference
        cv_img  = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        results = self.model(cv_img)                   # list of ultralytics Results
        annotated = results[0].plot()                  # numpy array with boxes drawn

        # 2) publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
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
            bb.size_x   = float(w)
            bb.size_y   = float(h)
            det.bbox    = bb

            # result (class label & score)
           # create the hypothesis‐with‐pose message
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(box.cls[0].item()))
            hyp.hypothesis.score    = float(box.conf[0].item())

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

if __name__ == '__main__':
    main()
