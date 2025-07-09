#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2

def iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    inter = max(0, xB-xA) * max(0, yB-yA)
    areaA = (boxA[2]-boxA[0]) * (boxA[3]-boxA[1])
    areaB = (boxB[2]-boxB[0]) * (boxB[3]-boxB[1])
    return inter / float(areaA + areaB - inter + 1e-8)

def filter_yellow_with_orange_proximity(detections,
                                        iou_thresh=0.25,
                                        pixel_thresh=30,
                                        z_thresh=0.2):
    oranges = [d for d in detections if d['label'] in ('orange','large_orange')]
    yellows = [d for d in detections if d['label']=='yellow']
    others  = [d for d in detections if d['label'] not in ('orange','large_orange','yellow')]
    keep = []
    keep += oranges
    for y in yellows:
        y_box = y['bbox']
        y_ctr = np.array([ (y_box[0]+y_box[2])/2, (y_box[1]+y_box[3])/2 ])
        y_z   = y['pos'][2]
        suppressed = False
        for o in oranges:
            o_box = o['bbox']
            o_ctr = np.array([ (o_box[0]+o_box[2])/2, (o_box[1]+o_box[3])/2 ])
            o_z   = o['pos'][2]
            if iou(y_box,o_box)>iou_thresh or \
               np.linalg.norm(y_ctr-o_ctr)<pixel_thresh or \
               abs(y_z-o_z)<z_thresh:
                suppressed = True
                break
        if not suppressed:
            keep.append(y)
    keep += others
    return keep

class YoloRosNode(Node):
    def __init__(self):
        super().__init__('yolo_ros_node')
        self.bridge = CvBridge()

        # --- YOLO model load ---
        weights = '/home/jay/Documents/yolo11-tutorial/runs/detect/train18/weights/best.pt'
        self.model = YOLO(weights)

        # --- class ↔ label ↔ visuals ↔ real heights ---
        self.color_map = {0:'blue', 2:'orange', 1:'large_orange', 4:'yellow'}
        self.visual_map = {
            'blue': (255, 0, 0),
            'yellow': (0,255,255),
            'orange': (0,128,255),
            'large_orange': (0,80,180),
            'unknown': (128,128,128)
        }
        self.height_map = {
            'blue': 0.325,
            'yellow': 0.325,
            'orange': 0.325,
            'large_orange': 0.505
        }

        # --- large_orange box‐size filtering params ---
        self.min_large_px = 50    # minimum pixel‐height
        self.min_ar = 0.2          # min width/height
        self.max_ar = 0.8          # max width/height

        # --- camera intrinsics (set once) ---
        self.fx = self.fy = self.cx = self.cy = None

        # --- ROS subscriptions & publishers ---
        self.create_subscription(CameraInfo,
                                 '/camera/camera_info',
                                 self.caminfo_cb, 10)
        self.create_subscription(Image,
                                 '/camera/image_raw',
                                 self.image_cb, 10)

        self.info_pub  = self.create_publisher(CameraInfo,
                                               '/yolo/camera_info', 10)
        self.det_pub   = self.create_publisher(Detection2DArray,
                                               '/yolo/detections', 10)
        self.pose_pub  = self.create_publisher(PoseArray,
                                               '/cone_poses', 10)
        self.image_pub = self.create_publisher(Image,
                                               '/yolo/image', 10)

        self.conf_thresh = 0.7
        self.get_logger().info('YOLO node with large_orange size filtering started')

    def caminfo_cb(self, msg:CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}"
            )
        self.info_pub.publish(msg)

    def image_cb(self, img_msg:Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        vis = frame.copy()
        results = self.model(frame)[0]

        dets = []
        for box in results.boxes:
            conf = float(box.conf[0].item())
            if conf < self.conf_thresh:
                continue

            cls_id = int(box.cls[0].item())
            label  = self.color_map.get(cls_id,'unknown')
            x1,y1,x2,y2 = box.xyxy[0].tolist()
            h = y2 - y1
            w = x2 - x1

            # -- large_orange bbox filtering --
            if label=='large_orange':
                if h < self.min_large_px:
                    continue
                ar = (w/h) if h>0 else 0
                if not (self.min_ar < ar < self.max_ar):
                    continue

            # -- estimate 3D Z from pixel height --
            if self.fx and label in self.height_map and h>0:
                Z = (self.fy * self.height_map[label]) / h
                u = (x1+x2)/2.0;  v = (y1+y2)/2.0
                X = (u - self.cx)*Z/self.fx
                Y = (v - self.cy)*Z/self.fy
            else:
                X=Y=0.0; Z=float(h)

            dets.append({
                'bbox': (x1,y1,x2,y2),
                'conf': conf,
                'label': label,
                'pos': np.array([X,Y,Z])
            })

        # suppress yellow near any orange
        dets = filter_yellow_with_orange_proximity(dets)

        det_arr  = Detection2DArray()
        det_arr.header = img_msg.header
        pose_arr = PoseArray()
        pose_arr.header = img_msg.header

        for d in dets:
            x1,y1,x2,y2 = d['bbox']
            X,Y,Z = d['pos']
            label = d['label']
            conf  = d['conf']
            color = self.visual_map.get(label,(128,128,128))

            # draw
            cv2.rectangle(vis, (int(x1),int(y1)),
                               (int(x2),int(y2)), color, 2)
            cv2.putText(vis,
                        f"{label} {conf*100:.1f}% {Z:.2f}m",
                        (int(x1),int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, color, 2)

            # Detection2D
            d2 = Detection2D()
            d2.header = img_msg.header
            bb = BoundingBox2D()
            bb.center.position.x = (x1+x2)/2.0
            bb.center.position.y = (y1+y2)/2.0
            bb.size_x = x2 - x1
            bb.size_y = y2 - y1
            d2.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score    = conf
            hyp.pose.pose.position.x = float(X)
            hyp.pose.pose.position.y = float(Y)
            hyp.pose.pose.position.z = float(Z)
            hyp.pose.pose.orientation.w = 1.0
            d2.results = [hyp]
            det_arr.detections.append(d2)

            # PoseArray
            p = Pose()
            p.position.x = float(X)
            p.position.y = float(Y)
            p.position.z = float(Z)
            p.orientation.w = 1.0
            pose_arr.poses.append(p)

        # publish
        out_img = self.bridge.cv2_to_imgmsg(vis,'bgr8')
        out_img.header = img_msg.header
        self.image_pub.publish(out_img)
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

if __name__=='__main__':
    main()
