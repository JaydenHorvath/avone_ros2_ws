#!/usr/bin/env python3
"""
Dual-camera YOLO + depth cone-tracking node for Formula SAE AV.ONE
------------------------------------------------------------------
 * Subscribes to the Gazebo RealSense replicas on topics:
     /rgb_left/{image,camera_info}
     /rgb_right/{image,camera_info}
 * Runs Ultralytics YOLOv11 + ByteTrack on each stream independently.
 * Projects detections into 3-D using per-camera intrinsics.
 * Tracks cones with per-camera Kalman filters.
 * Publishes:
     – Annotated images on /yolo_{left,right}/image
     – Detection2DArray on /yolo_{left,right}/detections
     – **Blue cone poses** (from the *left* camera) on /cone_poses_blue
     – **Yellow cone poses** (from the *right* camera) on /cone_poses_yellow
Author: ChatGPT (OpenAI) for Jay — 02 Jul 2025
"""

import os
import time
from collections import deque
from threading import Lock, Thread
from typing import Dict

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseWithCovariance
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

from yolo_ros.cone_kf import ConeKF  # Your existing Kalman helper


class YoloDualCamNode(Node):
    CAMS = {
        # "left": "camera1/camera1/color",
        # "right": "camera2/camera2/color",

          "left": "rgb_left",
        "right": "rgb_right",
    }

    def __init__(self):
        super().__init__("yolo_dual_cam_node")
        self.bridge = CvBridge()

        # ─── YOLO + ByteTrack config ───
        pkg = get_package_share_directory("yolo_ros")
        self.tracker_cfg = os.path.join(pkg, "config", "bytetrack_custom.yaml")
        weights = "/home/jay/Documents/yolo11-tutorial/runs/detect/train18/weights/best.pt"
        self.model = YOLO(weights)
        self.model.to("cuda:0")
        self.model.model.half()

        # map class → color & real height
        self.color_map  = {0: "blue", 1: "orange", 2: "orange", 4: "yellow"}
        self.height_map = {
            "blue":   0.325,
            "yellow": 0.325,
            "orange": 0.325
        }

        # intrinsics + frame buffers
        self.fx: Dict[str, float] = {}
        self.fy: Dict[str, float] = {}
        self.cx: Dict[str, float] = {}
        self.cy: Dict[str, float] = {}
        self.image_q = {cam: deque(maxlen=2) for cam in self.CAMS}

        # tracking state
        self.tracks            = {cam: {} for cam in self.CAMS}
        self.next_id           = {cam: 1   for cam in self.CAMS}
        self.max_missed        = 5
        self.match_dist_thresh = 0.5

        self.lock = Lock()

        # ─── ROS I/O ───
        # per-camera image + detection pubs
        for cam, base in self.CAMS.items():
            self.create_subscription(
                Image, f"/{base}/image",
                lambda m, c=cam: self.image_cb(m, c), 10
            )
            self.create_subscription(
                CameraInfo, f"/{base}/camera_info",
                lambda m, c=cam: self.caminfo_cb(m, c), 10
            )

            setattr(self, f"image_pub_{cam}",
                    self.create_publisher(Image, f"/yolo_{cam}/image", 10))
            setattr(self, f"det_pub_{cam}",
                    self.create_publisher(Detection2DArray, f"/yolo_{cam}/detections", 10))

        # two color-specific pose publishers
        self.pose_pub_blue   = self.create_publisher(PoseArray, "/cone_poses_blue", 10)
        self.pose_pub_yellow = self.create_publisher(PoseArray, "/cone_poses_yellow", 10)

        # start worker
        self.running = True
        Thread(target=self.inference_worker, daemon=True).start()
        self.get_logger().info("Dual-camera YOLO node initialised")

    def caminfo_cb(self, msg: CameraInfo, cam: str):
        self.fx[cam], self.fy[cam] = msg.k[0], msg.k[4]
        self.cx[cam], self.cy[cam] = msg.k[2], msg.k[5]

    def image_cb(self, msg: Image, cam: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"{cam} cv_bridge error: {e}")
            return
        with self.lock:
            self.image_q[cam].append((frame, msg.header))

    def inference_worker(self):
        last_t = time.time()
        while rclpy.ok() and self.running:
            now = time.time()
            dt  = now - last_t
            last_t = now

            for cam in self.CAMS:
                with self.lock:
                    if not self.image_q[cam]:
                        continue
                    frame, header = self.image_q[cam].pop()
                    self.image_q[cam].clear()

                # wait for intrinsics
                if cam not in self.fx:
                    continue
                fx, fy, cx, cy = (
                    self.fx[cam], self.fy[cam],
                    self.cx[cam], self.cy[cam]
                )

                # run YOLO+ByteTrack
                res = self.model.track(
                    source=frame, classes=[0,1,2,4],
                    tracker=self.tracker_cfg, imgsz=640,
                    half=True, verbose=False
                )[0]

                # convert + gate truncation
                dets = []
                h_img, w_img = frame.shape[:2]
                for box in res.boxes:
                    cls = int(box.cls[0].item())
                    label = self.color_map.get(cls, "unk")
                    x1,y1,x2,y2 = box.xyxy[0].tolist()

                    # skip any box touching the frame boundary
                    if x1 <= 0 or y1 <= 0 or x2 >= w_img-1 or y2 >= h_img-1:
                        continue

                    h = y2 - y1
                    u, v = (x1+x2)/2, (y1+y2)/2
                    if h > 0 and label in self.height_map:
                        Z = (fy * self.height_map[label]) / h
                        X = (u - cx) * Z / fx
                        Y = (v - cy) * Z / fy
                    else:
                        X = Y = 0.0
                        Z = float(h)

                    dets.append({
                        "pos":   np.array([X,Y,Z]),
                        "bbox":  (x1,y1,x2,y2),
                        "conf":  float(box.conf[0].item()),
                        "label": label
                    })

                # Kalman association
                tracks = self.tracks[cam]
                new_tracks, used, updated = {}, set(), set()
                for tid,t in tracks.items():
                    t["missed"] += 1
                    t["kf"].predict(dt=dt)

                for det in sorted(dets, key=lambda d:-d["conf"]):
                    pos = det["pos"]
                    best_id, best_d = None, self.match_dist_thresh
                    for tid,t in tracks.items():
                        if tid in used: continue
                        d = np.linalg.norm(t["kf"].kf.x[:3] - pos)
                        if d < best_d:
                            best_id, best_d = tid, d

                    if best_id is not None:
                        t = tracks[best_id]
                        t["kf"].update(pos)
                        new_tracks[best_id] = {"kf":t["kf"], "missed":0}
                        det["tid"] = best_id
                        used.add(best_id)
                        updated.add(best_id)
                    else:
                        tid = self.next_id[cam]
                        self.next_id[cam] += 1
                        kf = ConeKF(pos)
                        new_tracks[tid] = {"kf":kf, "missed":0}
                        det["tid"] = tid
                        updated.add(tid)

                # carry over survivors
                for tid,t in tracks.items():
                    if t["missed"] < self.max_missed and tid not in new_tracks:
                        new_tracks[tid] = t
                self.tracks[cam] = new_tracks

                # visualize + per-cam publishes
                vis = frame.copy()
                det_msg = Detection2DArray()
                det_msg.header = header

                # collect poses for this camera/color
                cam_poses = []

                for det in dets:
                    tid = det["tid"]
                    if tid not in updated:
                        continue
                    pos = self.tracks[cam][tid]["kf"].kf.x[:3]
                    x1,y1,x2,y2 = det["bbox"]
                    label = det["label"]
                    col = (255,0,0) if label=="blue" else ((0,255,255) if label=="yellow" else (0,165,255))

                    cv2.rectangle(vis, (int(x1),int(y1)), (int(x2),int(y2)), col, 2)
                    cv2.putText(vis, f"{pos[2]:.2f} m", (int(x1),int(y1)-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

                    d2 = Detection2D(); d2.header = header
                    bb = BoundingBox2D()
                    bb.center.position.x = (x1+x2)/2
                    bb.center.position.y = (y1+y2)/2
                    bb.size_x = x2-x1; bb.size_y = y2-y1
                    d2.bbox = bb

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label
                    hyp.hypothesis.score    = det["conf"]
                    pose = PoseWithCovariance()
                    pose.pose.position.x = float(pos[0])
                    pose.pose.position.y = float(pos[1])
                    pose.pose.position.z = float(pos[2])
                    pose.pose.orientation.w = 1.0
                    hyp.pose = pose
                    d2.results = [hyp]
                    det_msg.detections.append(d2)

                    # only blue on left, yellow on right
                    if cam == "left"  and label == "blue":
                        cam_poses.append(pose.pose)
                    if cam == "right" and label == "yellow":
                        cam_poses.append(pose.pose)

                # publish image + detections
                img_msg = self.bridge.cv2_to_imgmsg(vis, "bgr8")
                img_msg.header = header
                getattr(self, f"image_pub_{cam}").publish(img_msg)
                getattr(self, f"det_pub_{cam}").publish(det_msg)

                # publish color-specific poses
                if cam == "left" and cam_poses:
                    blue_msg = PoseArray(header=header, poses=cam_poses)
                    self.pose_pub_blue.publish(blue_msg)
                if cam == "right" and cam_poses:
                    yellow_msg = PoseArray(header=header, poses=cam_poses)
                    self.pose_pub_yellow.publish(yellow_msg)

            # if nothing processed, sleep briefly
            time.sleep(0.005)

        self.get_logger().info("Inference thread stopped")

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDualCamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
