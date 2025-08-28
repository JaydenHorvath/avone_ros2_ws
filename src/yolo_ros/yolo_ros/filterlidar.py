#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class PointCloud180(Node):
    def __init__(self):
        super().__init__('pc_180_filter')
        self.declare_parameter('in_topic',  '/quanergy/points')
        self.declare_parameter('out_topic', '/points_180')
        self.declare_parameter('min_angle', -math.pi/3)  # –90°
        self.declare_parameter('max_angle',  math.pi/3)  # +90°

        in_topic  = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        self.sub = self.create_subscription(
            PointCloud2, in_topic, self.cb, 10)
        self.pub = self.create_publisher(
            PointCloud2, out_topic, 10)

    def cb(self, msg):
        pts = pc2.read_points(msg, skip_nans=True)
        sec = []
        mn = self.get_parameter('min_angle').value
        mx = self.get_parameter('max_angle').value

        for x, y, z, *rest in pts:
            θ = math.atan2(y, x)
            if mn <= θ <= mx:
                sec.append((x, y, z, *rest))

        # --- DOWNSAMPLE ---
        decimation = 5  # keep 1 out of every 5 points
        sec = sec[::decimation]

        # build a new PointCloud2 – here we drop intensity, etc.
        header = msg.header
        new_pc = pc2.create_cloud_xyz32(header,
                    [(x, y, z) for x, y, z, *_ in sec])
        self.pub.publish(new_pc)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloud180()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
