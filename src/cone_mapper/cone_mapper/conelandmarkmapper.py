#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import PoseArray, Pose, PointStamped

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import math


class ConeLandmarkMapper(Node):
    def __init__(self):
        super().__init__('cone_landmark_mapper')

        # storage for the latest organized PointCloud2
        self.latest_pc2 = None
        self.pc2_width = 0
        self.pc2_height = 0

        # subscribe to the point cloud
        self.create_subscription(
            PointCloud2,
            '/camera/rgbd/points',
            self.on_pointcloud,
            10
        )

        # subscribe to 2D cone detections
        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.on_detections,
            10
        )

        # set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer, self)

        # publisher for the mapped cone poses
        self.landmark_pub = self.create_publisher(
            PoseArray,
            '/cone_landmarks',
            10
        )

    def on_pointcloud(self, msg: PointCloud2):
        # cache the latest cloud and its dimensions
        self.latest_pc2 = msg
        self.pc2_width = msg.width
        self.pc2_height = msg.height

    def on_detections(self, det_msg: Detection2DArray):
        # wait until we have a cloud
        if self.latest_pc2 is None:
            return

        w, h = self.pc2_width, self.pc2_height
        pa = PoseArray()
        pa.header.frame_id = 'map'
        pa.header.stamp = det_msg.header.stamp

        total = w * h

        for det in det_msg.detections:
            # pixel coordinates of the bbox center
            cx = int(det.bbox.center.position.x)
            cy = int(det.bbox.center.position.y)

            # skip out‐of‐bounds
            if not (0 <= cx < w and 0 <= cy < h):
                continue

            # compute flat index in row-major order
            idx = cy * w + cx

            # iterate through the cloud until that index
            point_iter = pc2.read_points(
                self.latest_pc2,
                field_names=('x', 'y', 'z'),
                skip_nans=False
            )
            x_cam = y_cam = z_cam = None
            for i, pt in enumerate(point_iter):
                if i == idx:
                    # explicit float conversion
                    x_cam = float(pt[0])
                    y_cam = float(pt[1])
                    z_cam = float(pt[2])
                    break

            # if we didn't find it or it's invalid, skip
            if x_cam is None or not all(math.isfinite(v) for v in (x_cam, y_cam, z_cam)):
                continue

            # stamp this 3D point in the camera frame
            p_cam = PointStamped()
            p_cam.header = self.latest_pc2.header
            p_cam.header.stamp = det_msg.header.stamp
            p_cam.point.x = x_cam
            p_cam.point.y = y_cam
            p_cam.point.z = z_cam

            # transform into the 'map' frame
            try:
                t = self.tf_buffer.lookup_transform(
                    'map',
                    p_cam.header.frame_id,
                    Time(),  # latest
                    timeout=Duration(seconds=0.5),
                )
                p_map = do_transform_point(p_cam, t)
            except Exception as e:
                self.get_logger().warn(f'TF error: {e}')
                continue

            # append to the PoseArray
            pose = Pose()
            pose.position = p_map.point
            pa.poses.append(pose)

        if pa.poses:
            self.landmark_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = ConeLandmarkMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
