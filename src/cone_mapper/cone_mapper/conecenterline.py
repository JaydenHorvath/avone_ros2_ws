#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np


class ConeCenterLineNode(Node):
    """
    Subscribes to:
      • /cone_landmarks_colored  (MarkerArray of validated cones, colored by class)
      • /cluster_centroids       (MarkerArray of raw cluster centroids)
    Whenever new data arrives on either topic, it:
      1) Splits validated cones into blue vs. yellow by color.
      2) If validated of a given color is empty, falls back to raw cluster centroids.
      3) Pairs each yellow-point with its nearest blue-point.
      4) Computes midpoints, sorts them along X, and publishes a thin green LINE_STRIP on /cone_center_line.
      5) If there are fewer than two total points on either side, publishes an empty MarkerArray to clear previous lines.
    """

    def __init__(self):
        super().__init__('cone_centerline_node')

        # Storage for the latest raw cluster centroids (as np.array([x,y,z]))
        self.raw_clusters = []

        # Storage for the latest validated cone positions by color
        self.validated_blue   = []  # list of np.array([x,y,z])
        self.validated_yellow = []  # list of np.array([x,y,z])

        # Subscriber: raw cluster centroids published as SPHERE markers on /cluster_centroids
        self.raw_sub = self.create_subscription(
            MarkerArray,
            '/cluster_centroids',
            self.raw_callback,
            10
        )

        # Subscriber: validated cones (CYLINDER markers colored by class) on /cone_landmarks_colored
        self.validated_sub = self.create_subscription(
            MarkerArray,
            '/cone_landmarks_colored',
            self.validated_callback,
            10
        )

        # Publisher: center-line as a LINE_STRIP MarkerArray
        self.line_pub = self.create_publisher(
            MarkerArray,
            '/cone_center_line',
            10
        )

        self.get_logger().info("ConeCenterLineNode running: subscribing to /cluster_centroids and /cone_landmarks_colored")

    def raw_callback(self, msg: MarkerArray):
        """
        Store all raw cluster centroid positions, then attempt to recompute center-line.
        Raw centroids have no color information—we only capture their (x,y,z).
        """
        self.raw_clusters.clear()
        for m in msg.markers:
            x = m.pose.position.x
            y = m.pose.position.y
            z = m.pose.position.z
            self.raw_clusters.append(np.array([x, y, z]))

        self.compute_and_publish_centerline(msg.markers[0].header.frame_id)

    def validated_callback(self, msg: MarkerArray):
        """
        Separate validated cones into blue vs. yellow by marker.color,
        then attempt to recompute center-line.
        Blue cones: roughly (r≈0, g≈0, b≈1). 
        Yellow cones: roughly (r≈1, g≈1, b≈0).
        """
        self.validated_blue.clear()
        self.validated_yellow.clear()

        for m in msg.markers:
            x = m.pose.position.x
            y = m.pose.position.y
            z = m.pose.position.z

            r = m.color.r
            g = m.color.g
            b = m.color.b

            # Tolerances allow slight variation in color
            if b > 0.9 and r < 0.5 and g < 0.5:
                # Blue cone
                self.validated_blue.append(np.array([x, y, z]))
            elif r > 0.9 and g > 0.9 and b < 0.5:
                # Yellow cone
                self.validated_yellow.append(np.array([x, y, z]))

        self.compute_and_publish_centerline(msg.markers[0].header.frame_id)

    def compute_and_publish_centerline(self, frame_id: str):
        """
        1) Determine which lists to use for blue vs. yellow:
           - If validated_blue has entries, use that; otherwise use raw_clusters.
           - If validated_yellow has entries, use that; otherwise use raw_clusters.
        2) If either side has fewer than 1 point, publish empty MarkerArray to clear any existing line.
        3) Pair each yellow-point with nearest blue-point (no reuse of the same blue-point).
        4) Compute midpoints, sort by X, build a LINE_STRIP marker, and publish.
        """
        # Choose data for each color
        blue_list   = self.validated_blue   if len(self.validated_blue)   > 0 else list(self.raw_clusters)
        yellow_list = self.validated_yellow if len(self.validated_yellow) > 0 else list(self.raw_clusters)

        # Need at least one on each side to form a pair
        if len(blue_list) < 1 or len(yellow_list) < 1:
            # Clear previous line
            empty = MarkerArray()
            self.line_pub.publish(empty)
            return

        # Convert to NumPy arrays if they aren't already
        bc_arr = np.stack(blue_list, axis=0) if not isinstance(blue_list, np.ndarray) else blue_list
        yc_arr = np.stack(yellow_list, axis=0) if not isinstance(yellow_list, np.ndarray) else yellow_list

        midpoints = []
        used_blue = set()

        # Pair each yellow with nearest blue
        for y_pt in yc_arr:
            # Compute squared distances to all blues
            d2 = np.sum((bc_arr - y_pt) ** 2, axis=1)
            best_idx = int(np.argmin(d2))
            if best_idx in used_blue:
                continue
            used_blue.add(best_idx)
            b_pt = bc_arr[best_idx]
            midpoint = (y_pt + b_pt) / 2.0
            midpoints.append(midpoint)

        # Need at least two midpoints to draw a line
        if len(midpoints) < 2:
            empty = MarkerArray()
            self.line_pub.publish(empty)
            return

        # Sort midpoints along the X axis (adjust if your track’s forward direction differs)
        midpoints.sort(key=lambda p: float(p[0]))

        # Build the LINE_STRIP marker
        line_marker = Marker()
        line_marker.header.frame_id = frame_id
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = "cone_center_line"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        # Fill in the points for the LINE_STRIP
        for mp in midpoints:
            pt = Point()
            pt.x = float(mp[0])
            pt.y = float(mp[1])
            pt.z = float(mp[2]) + 0.05  # lift slightly above ground to avoid z-fighting
            line_marker.points.append(pt)

        # Style: thin green line
        line_marker.scale.x = 0.05  # width (meters)
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        out_array = MarkerArray()
        out_array.markers.append(line_marker)
        self.line_pub.publish(out_array)


def main(args=None):
    rclpy.init(args=args)
    node = ConeCenterLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
