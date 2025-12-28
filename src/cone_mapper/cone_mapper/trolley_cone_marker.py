#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PointStamped

import csv
import os

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # REQUIRED for PointStamped transform support


class PointCloudClusterMapper(Node):
    def __init__(self):
        super().__init__("pointcloud_cluster_mapper")

        # Parameters
        self.declare_parameter("cluster_eps", 0.25)
        self.declare_parameter("cluster_min_points", 35)
        self.declare_parameter("z_min", -0.2)
        self.declare_parameter("z_max", 1.0)
        self.declare_parameter("side_frame", "gps")
        self.declare_parameter("force_color", "")  # "", "blue", or "orange"

        self.eps = self.get_parameter("cluster_eps").value
        self.min_samples = self.get_parameter("cluster_min_points").value
        self.z_min = self.get_parameter("z_min").value
        self.z_max = self.get_parameter("z_max").value
        self.side_frame = self.get_parameter("side_frame").value
        self.force_color = self.get_parameter("force_color").value

        # Saved cones list
        self.saved_cones = []  # each entry: {"x":, "y":, "color":}

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sub & Pub
        self.sub_map = self.create_subscription(
            PointCloud2,
            "/global_lidar_map",
            self.map_callback,
            5
        )

        self.pub_markers = self.create_publisher(
            MarkerArray,
            "/cluster_markers",
            20
        )

        self.get_logger().info(f"PointCloud Cluster Mapper running (side_frame={self.side_frame})")

    # ----------------------------------------------------------------------
    def transform_to_side_frame(self, x, y, z, source_frame, stamp):
        """Transform a centroid to side_frame using tf2."""
        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = stamp
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)

        try:
            ps.header.stamp = rclpy.time.Time().to_msg()  # force NOW
            out = self.tf_buffer.transform(
                ps,
                self.side_frame,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            return out.point.x, out.point.y, out.point.z
        except TransformException as ex:
            self.get_logger().warn(f"Transform failed: {ex}")
            return None

    # ----------------------------------------------------------------------
    def map_callback(self, msg: PointCloud2):
        raw_points = []
        for p in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            if self.z_min < p[2] < self.z_max:
                raw_points.append([p[0], p[1], p[2]])

        if len(raw_points) == 0:
            return

        pts_np = np.array(raw_points)

        # DBSCAN clustering
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(pts_np)
        labels = clustering.labels_

        unique_clusters = [l for l in set(labels) if l != -1]

        markers = MarkerArray()
        marker_id = 0

        # Add existing saved cones first
        for c in self.saved_cones:
            markers.markers.append(
                self.make_marker(marker_id, c["x"], c["y"], c["colour"], msg.header)
            )
            marker_id += 1

        # Process new clusters
        for cluster_id in unique_clusters:
            cluster_points = pts_np[labels == cluster_id]
            cx, cy = np.mean(cluster_points[:, :2], axis=0)

            # Skip duplicates
            if self.is_duplicate(cx, cy):
                continue

            # Transform into side frame
            tf_out = self.transform_to_side_frame(
                cx, cy, 0.0, msg.header.frame_id, msg.header.stamp
            )
            if tf_out is None:
                continue

            tx, ty, _ = tf_out

            # Determine side of car
            if self.force_color in ("blue", "orange"):
                color = self.force_color
            else:
                color = "orange" if ty > 0 else "blue"

            # Save permanently
            self.saved_cones.append({"x": cx, "y": cy, "colour": color})

            # Add marker
            markers.markers.append(
                self.make_marker(marker_id, cx, cy, color, msg.header)
            )
            marker_id += 1

        self.pub_markers.publish(markers)
        self.get_logger().info(f"Clusters detected: {len(unique_clusters)}")

    # ----------------------------------------------------------------------
    def is_duplicate(self, x, y):
        for c in self.saved_cones:
            if np.hypot(c["x"] - x, c["y"] - y) < 0.5:
                return True
        return False

    # ----------------------------------------------------------------------
    def make_marker(self, marker_id, x, y, color, header):
        m = Marker()
        m.header = header
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.5

        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.0

        if color == "blue":
            m.color = ColorRGBA(r=0.0, g=0.3, b=1.0, a=1.0)
        else:
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)

        return m

    # ----------------------------------------------------------------------
    def save_csv(self, path="~/clustered_cones.csv"):
        filepath = os.path.expanduser(path)
        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "colour"])
            for c in self.saved_cones:
                writer.writerow([c["x"], c["y"], c["colour"]])
        self.get_logger().info(f"CSV saved: {filepath}")

# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudClusterMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Saving cone CSV before shutdown...")
        node.save_csv()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
