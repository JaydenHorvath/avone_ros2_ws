#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import numpy as np

class GroundRemovalHeightNode(Node):
    """
    Keep points by height only:
      - keep points with z >= keep_above_z (and optionally z <= keep_below_z if set)
    Assumes +Z is up in the input frame. If your LiDAR frame is different, set tf before this node.
    """
    def __init__(self):
        super().__init__('ground_removal_height_node')

        # ---- Parameters -------------------------------------------------------
        # Keep points whose z >= keep_above_z (default 0.05 m)
        self.declare_parameter('keep_above_z', -0.9)
        # Optional upper cap: keep points whose z <= keep_below_z (default: disabled = very large number)
        self.declare_parameter('keep_below_z',  9999.0)
        # Input/output topics
        self.declare_parameter('input_topic',  '/quanergy/points')
        self.declare_parameter('output_topic', '/cloud_no_ground_height')
        # Max publish rate (Hz)
        self.declare_parameter('max_hz', 10.0)

        self.keep_above_z = float(self.get_parameter('keep_above_z').value)
        self.keep_below_z = float(self.get_parameter('keep_below_z').value)
        self.input_topic  = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.max_period_ns = int(1e9 / float(self.get_parameter('max_hz').value))

        self.last_pub_time = self.get_clock().now()

        # QoS: best effort, depth 1 to minimize lag on high-rate clouds
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self.cloud_cb, qos)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f"GroundRemovalHeightNode: keeping {self.keep_above_z:.3f} ≤ z ≤ {self.keep_below_z:.3f} "
            f"from '{self.input_topic}' → '{self.output_topic}' @ ≤ {1e9/self.max_period_ns:.1f} Hz"
        )

    def cloud_cb(self, msg: PointCloud2):
        # Throttle
        now = self.get_clock().now()
        if (now - self.last_pub_time).nanoseconds < self.max_period_ns:
            return
        self.last_pub_time = now

        # Extract xyz (skip NaNs). Using list → np array for simplicity/compatibility.
        pts = [(x, y, z) for x, y, z in point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )]

        if not pts:
            self.get_logger().warn("Received empty/all-NaN cloud; skipping.")
            return

        cloud = np.asarray(pts, dtype=np.float32)

        # Height mask: keep z in [keep_above_z, keep_below_z]
        z = cloud[:, 2]
        mask = (z >= self.keep_above_z) & (z <= self.keep_below_z)
        kept = cloud[mask]

        # If everything filtered, publish an empty cloud with same header
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id

        if kept.size == 0:
            # Publish truly empty XYZ32 cloud
            out = point_cloud2.create_cloud_xyz32(header, [])
            self.pub.publish(out)
            self.get_logger().debug("All points filtered by height; published empty cloud.")
            return

        out = point_cloud2.create_cloud_xyz32(header, kept.tolist())
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = GroundRemovalHeightNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
