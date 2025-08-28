#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import tf2_ros
import tf_transformations
from sensor_msgs_py import point_cloud2

class DebugImageNode(Node):
    def __init__(self):
        super().__init__('debug_image_node')
        self.bridge = CvBridge()

        # Topics - adjust these for your setup
        self.declare_parameter('camera_topic', '/yolo/image')
        self.declare_parameter('camera_info_topic', '/yolo/camera_info')
        self.declare_parameter('lidar_topic', '/cloud_no_ground_ransac')
        self.declare_parameter('camera_frame', 'camera_link')

        self.K = None
        self.img_w = None
        self.img_h = None

        # Subscriptions
        self.create_subscription(Image,
                                 self.get_parameter('camera_topic').value,
                                 self.image_cb, 10)
        self.create_subscription(CameraInfo,
                                 self.get_parameter('camera_info_topic').value,
                                 self.camera_info_cb, 10)
        self.create_subscription(PointCloud2,
                                 self.get_parameter('lidar_topic').value,
                                 self.pc_cb, 10)

        # Publisher for debug overlay
        self.debug_pub = self.create_publisher(Image, '/cone_fusion/debug_image', 10)

        # TF buffer/listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_img = None

    def camera_info_cb(self, msg: CameraInfo):
        fx = msg.k[0]; fy = msg.k[4]; cx = msg.k[2]; cy = msg.k[5]
        self.K = (fx, fy, cx, cy)
        self.img_w = msg.width
        self.img_h = msg.height

    def image_cb(self, msg: Image):
        self.latest_img = msg

    def pc_cb(self, msg: PointCloud2):
        if self.K is None or self.latest_img is None:
            return

        # TF lidar->camera
        try:
            t_l2c = self.tf_buffer.lookup_transform(
                self.get_parameter('camera_frame').value,
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Read point cloud
        pts_iter = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_list = [(float(x), float(y), float(z)) for x, y, z in pts_iter]
        if not pts_list:
            return
        P = np.array(pts_list, dtype=np.float32)

        # Transform to camera frame
        T = t_l2c.transform
        Rq = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
        R = tf_transformations.quaternion_matrix(Rq)[:3, :3]
        t_vec = np.array([T.translation.x, T.translation.y, T.translation.z], dtype=np.float32)
        Pc = (R @ P.T).T + t_vec

        # Project points
        fx, fy, cx, cy = self.K
        Z = Pc[:, 2]
        u = (fx * Pc[:, 0] / Z) + cx
        v = (fy * Pc[:, 1] / Z) + cy

        mask = (Z > 0) & (u >= 0) & (u < self.img_w) & (v >= 0) & (v < self.img_h)
        u = u[mask].astype(int)
        v = v[mask].astype(int)

        # Draw on image
        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.latest_img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        for px, py in zip(u, v):
            cv2.circle(cv_img, (px, py), 2, (0, 255, 0), -1)

        debug_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        debug_msg.header = self.latest_img.header
        self.debug_pub.publish(debug_msg)
        self.get_logger().info(f"Published debug overlay with {len(u)} projected points")

def main(args=None):
    rclpy.init(args=args)
    node = DebugImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
