
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
import tf2_ros
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf_transformations

class LidarImageOverlay(Node):
    def __init__(self):
        super().__init__('lidar_image_overlay')
        self.bridge = CvBridge()
        # Subscribers
        self.create_subscription(Image, '/yolo/image', self._image_cb, 10)
        self.create_subscription(CameraInfo, '/yolo/camera_info', self._info_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        # Publisher for overlay
        self.pub = self.create_publisher(Image, '/overlay_image', 10)

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage
        self.camera_info = None
        self.latest_image = None
        self.latest_scan = None

    def _info_cb(self, msg: CameraInfo):
        if self.camera_info is None:
            # store intrinsics once
            self.camera_info = msg
            # camera matrix
            self.K = np.array(msg.k).reshape(3, 3)
            # distortion coefficients (unused in this demo)
            self.dist = np.array(msg.d)
            self.get_logger().info('Camera info received')

    def _image_cb(self, msg: Image):
        self.latest_image = msg
        self._try_overlay()

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg
        self._try_overlay()

    def _try_overlay(self):
        # need at least one of each
        if not self.camera_info or not self.latest_image or not self.latest_scan:
            return
        try:
            # lookup transform from lidar to camera
            now = rclpy.time.Time().to_msg()
            trans = self.tf_buffer.lookup_transform(
                self.camera_info.header.frame_id,
                self.latest_scan.header.frame_id,
                now)
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # convert LaserScan -> Cartesian in lidar frame
        angles = np.arange(
            self.latest_scan.angle_min,
            self.latest_scan.angle_max,
            self.latest_scan.angle_increment)
        ranges = np.array(self.latest_scan.ranges[:angles.size])
        mask = np.isfinite(ranges)
        angles = angles[mask]
        ranges = ranges[mask]
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)
        ones = np.ones_like(xs)
        pts_lidar = np.vstack((xs, ys, zs, ones))  # shape (4, N)

        # build 4×4 transform matrix
        q = trans.transform.rotation
        t = trans.transform.translation
        T = tf_transformations.quaternion_matrix([
            q.x, q.y, q.z, q.w
        ])
        T[0:3, 3] = [t.x, t.y, t.z]
        # transform points into camera frame
        pts_cam = T.dot(pts_lidar)

        # project into image plane
        proj = self.K.dot(pts_cam[:3, :])
        proj[:2, :] /= proj[2, :]
        u = proj[0, :].astype(int)
        v = proj[1, :].astype(int)

        # convert ROS Image to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')

        # draw laser points
        h, w, _ = cv_img.shape
        for ui, vi, z in zip(u, v, pts_cam[2, :]):
            if z > 0 and 0 <= ui < w and 0 <= vi < h:
                cv2.circle(cv_img, (ui, vi), 6, (0, 255, 0), -1)

        # publish overlay
        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = self.latest_image.header
        self.pub.publish(out)

        # reset
        self.latest_image = None
        self.latest_scan = None


def main(args=None):
    rclpy.init(args=args)
    node = LidarImageOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
