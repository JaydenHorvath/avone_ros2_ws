#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import math

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_angle_distance_filter')

        self.sub = self.create_subscription(
            PointCloud2,
            '/quanergy/points',
            self.cloud_cb,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/nav2_points',
            10
        )

        self.get_logger().info("Lidar filtering node ready")

        self.max_distance = 5.0
        self.half_arc = 60.0   # 120 deg total
        self.center_deg = 180.0   # facing backward

    def cloud_cb(self, msg: PointCloud2):
        pts_iter = point_cloud2.read_points(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )

        xs = []
        ys = []
        zs = []

        for p in pts_iter:
            try:
                xs.append(p[0])
                ys.append(p[1])
                zs.append(p[2])
            except Exception:
                xs.append(p['x'])
                ys.append(p['y'])
                zs.append(p['z'])

        if len(xs) == 0:
            return

        x = np.array(xs, dtype=np.float32)
        y = np.array(ys, dtype=np.float32)
        z = np.array(zs, dtype=np.float32)

        # distance filter
        dist = np.sqrt(x*x + y*y + z*z)
        mask_dist = dist <= self.max_distance

        # ----- angle filter -----
        # raw angle in deg, range [-180, 180]
        ang = np.degrees(np.arctan2(y, x))

        # convert to 0 to 360
        ang = (ang + 360.0) % 360.0

        # we want the 120 deg arc centered on 180 deg
        left_bound = self.center_deg - self.half_arc  # 120 deg
        right_bound = self.center_deg + self.half_arc # 240 deg

        mask_angle = np.logical_and(ang >= left_bound, ang <= right_bound)

        mask = np.logical_and(mask_dist, mask_angle)

        if not np.any(mask):
            return

        filtered_pts = np.column_stack((x[mask], y[mask], z[mask]))

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_pts.tolist())
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
