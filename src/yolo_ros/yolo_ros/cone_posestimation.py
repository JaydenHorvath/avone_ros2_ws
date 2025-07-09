#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from collections import deque
import numpy as np

class ConePositionEstimator(Node):
    def __init__(self):
        super().__init__('cone_position_estimator_filtered')

        # camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None

        # cone heights (m)
        self.height_map = {'yellow': 0.325, 'blue': 0.325}

        # for each cone (label:id) keep last N estimates
        self.hist_len = 8
        self.history = {}  # dict: key -> deque of (x,y,z)

        # subs / pubs
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.caminfo_cb, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections',
                                 self.detections_cb, 10)
        self.pose_pub = self.create_publisher(PoseArray,
                                              '/cone_positions_filtered', 10)

    def caminfo_cb(self, msg: CameraInfo):
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.get_logger().info(f"Loaded intrinsics fx={self.fx}")

    def detections_cb(self, dets: Detection2DArray):
        if self.fx is None:
            return

        out = PoseArray()
        out.header = dets.header

        for det in dets.detections:
            cls = det.results[0].hypothesis.class_id  # "yellow:2"
            label, did = cls.split(':')
            if label not in self.height_map:
                continue

            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            h_pix = det.bbox.size_y
            H = self.height_map[label]
            Z = (self.fy * H) / h_pix
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            key = f"{label}:{did}"
            buf = self.history.setdefault(key, deque(maxlen=self.hist_len))
            buf.append((X, Y, Z))

            # compute average
            arr = np.array(buf)
            xm, ym, zm = arr.mean(axis=0)

            p = Pose()
            p.position.x, p.position.y, p.position.z = float(xm), float(ym), float(zm)
            out.poses.append(p)

        self.pose_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ConePositionEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
