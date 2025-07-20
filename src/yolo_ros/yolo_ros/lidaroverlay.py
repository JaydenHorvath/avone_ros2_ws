#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge

import message_filters
import tf2_ros

import numpy as np
import cv2
import colorsys
from sensor_msgs_py import point_cloud2

class LidarImageOverlay(Node):
    def __init__(self):
        super().__init__('lidar_image_overlay_rings')
        self.bridge = CvBridge()

        # for mapping ring→BGR colour
        self.ring_colors = {}

        # img_sub   = message_filters.Subscriber(self, Image,       '/camera/image_raw')
        # info_sub  = message_filters.Subscriber(self, CameraInfo,  '/camera/camera_info')
        # cloud_sub = message_filters.Subscriber(self, PointCloud2, '/cloud_no_ground_ransac')
        img_sub   = message_filters.Subscriber(self, Image,       '/camera/image_raw')
        info_sub  = message_filters.Subscriber(self, CameraInfo,  '/camera/camera_info')
        cloud_sub = message_filters.Subscriber(self, PointCloud2, '/cloud_no_ground_ransac')

        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, info_sub, cloud_sub],
            queue_size=10,
            slop=0.1
        )
        sync.registerCallback(self.callback)

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Image, '/lidar/image_overlay', 1)

    def callback(self, img_msg, info_msg, cloud_msg):
        # 1) grab the image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        h, w, _ = cv_image.shape

        # 2) projection matrix
        P = np.array(info_msg.p).reshape((3,4))

        # 3) lookup transform LiDAR→camera
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame=info_msg.header.frame_id,
                source_frame=cloud_msg.header.frame_id,
                time=cloud_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # build 4×4 transform
        trans = np.eye(4)
        q = t.transform.rotation
        trans[:3,:3] = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
        trans[0,3] = t.transform.translation.x
        trans[1,3] = t.transform.translation.y
        trans[2,3] = t.transform.translation.z

        # 4) decide if we have a 'ring' channel
        field_names = [f.name for f in cloud_msg.fields]
        use_rings = 'ring' in field_names

        # 5) read the points
        if use_rings:
            pc_iter = point_cloud2.read_points(
                cloud_msg,
                field_names=('x','y','z','ring'),
                skip_nans=True
            )
        else:
            pc_iter = point_cloud2.read_points(
                cloud_msg,
                field_names=('x','y','z'),
                skip_nans=True
            )

        projected = 0
        for pt in pc_iter:
            if use_rings:
                x, y, z, ring = pt
                ring = int(ring)
            else:
                x, y, z = pt
                ring = None

            # transform into camera frame
            Xc, Yc, Zc, _ = trans.dot([x, y, z, 1.0])
            if Zc <= 0:
                continue

            # project
            uvw = P.dot([Xc, Yc, Zc, 1.0])
            u_f, v_f, w_proj = uvw
            if not np.isfinite(w_proj) or w_proj == 0:
                continue

            u = int(u_f / w_proj)
            v = int(v_f / w_proj)
            if not (0 <= u < w and 0 <= v < h):
                continue

            # pick colour
            if use_rings:
                if ring not in self.ring_colors:
                    # spread hues by golden‐ratio fraction
                    h_frac = (ring * 0.618033988749895) % 1.0
                    r_, g_, b_ = colorsys.hsv_to_rgb(h_frac, 1.0, 1.0)
                    # OpenCV uses BGR
                    self.ring_colors[ring] = (
                        int(b_ * 255),
                        int(g_ * 255),
                        int(r_ * 255)
                    )
                color = self.ring_colors[ring]
            else:
                # fallback: white
                color = (255,255,255)

            # draw
            cv2.circle(cv_image, (u, v), 2, color, -1)
            projected += 1

        self.get_logger().info(f"Projected points onto image: {projected}")

        # 6) publish overlay
        out = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        out.header = img_msg.header
        self.pub.publish(out)

    @staticmethod
    def quaternion_to_matrix(x, y, z, w):
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z
        return np.array([
            [1 - 2*(yy + zz),   2*(xy - wz),       2*(xz + wy)],
            [2*(xy + wz),       1 - 2*(xx + zz),   2*(yz - wx)],
            [2*(xz - wy),       2*(yz + wx),       1 - 2*(xx + yy)]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = LidarImageOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
