#!/usr/bin/env python3


# avone_mapping/pointcloud_cluster_mapper.py (LiDAR cone clustering + persistent cone map)
# --------------------------------------------------------------------------------------
# This node takes a raw LiDAR PointCloud2 stream and extracts cone-like clusters using DBSCAN.
# Each detected cluster is converted into a single representative (x,y) observation, transformed into a
# chosen mapping frame (default `map`), and then associated to an existing saved cone if it is nearby.
# If no saved cone is close enough, a new cone is created.

# Key features (what this script is doing for AV.ONE):
#   - Z filtering: keeps only points in a band (z_min..z_max) to remove ground and tall objects.
#   - DBSCAN clustering: groups nearby points into "cone candidates" (typically in XY only).
#   - Robust representative point: uses median of near-base points, optionally snapped to an actual point
#     so the marker sits on real point cloud data rather than a mean centroid.
#   - Left/right colour classification: transforms the representative into `side_frame` (default base_link)
#     and assigns "blue" vs "yellow" based on the sign of lateral Y.
#   - Persistent cone map: keeps a list of cones in `marker_frame` (default map) and merges new observations
#     via nearest-neighbour association + optional exponential averaging.
#   - RViz visualisation: publishes 3 marker arrays (all cones, blue cones, yellow cones) and clears old
#     markers each cycle with DELETEALL.
#   - CSV export on shutdown: saves lat/lon (if origin provided) + colour for each saved cone.

# Assumptions / gotchas:
#   - Your TF tree must support transforms: pointcloud frame -> marker_frame and pointcloud frame -> side_frame.
#   - If side_frame is base_link, "left/right" is meaningful only if the vehicle frame is correct (REP-103).
#   - association_distance and update_alpha control map stability vs responsiveness.
#   - The lat/lon CSV export uses a simple local tangent approximation; useful for data analysis


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
import math

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401


WGS84_A = 6378137.0


class PointCloudClusterMapper(Node):
    def __init__(self):
        super().__init__("pointcloud_cluster_mapper")

        # Cone clustering parameters
        self.declare_parameter("cluster_eps", 0.25)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("z_min", -0.5)
        self.declare_parameter("z_max", 1.0)

        # Extra robustness
        self.declare_parameter("base_z_max", 0.35)  # use near-base points for centroid
        self.declare_parameter(
            "snap_to_point", True
        )  # snap marker onto nearest actual point
        self.declare_parameter("cluster_xy_only", True)  # DBSCAN on XY only

        # Frame used to determine left/right side of vehicle
        self.declare_parameter("side_frame", "base_link")

        # Frame where markers are placed (and where x/y are stored)
        self.declare_parameter("marker_frame", "map")

        # Optional override: "", "blue", or "yellow"
        self.declare_parameter("force_color", "")

        # CSV output
        self.declare_parameter("csv_path", "~/clustered_cones_latlon.csv")

        # Lat/lon origin for converting marker_frame (x,y) meters -> (lat,lon)
        self.declare_parameter("gps_origin_lat", 0.0)
        self.declare_parameter("gps_origin_lon", 0.0)
        self.declare_parameter("local_xy_rotation_deg", 0.0)

        # Association and localisation tuning
        self.declare_parameter("association_distance", 0.5)  # lowered from 1.0
        self.declare_parameter("update_alpha", 0.0)

        # Backwards compat
        self.declare_parameter("duplicate_threshold", 0.5)

        self.eps = float(self.get_parameter("cluster_eps").value)
        self.min_samples = int(self.get_parameter("cluster_min_points").value)
        self.z_min = float(self.get_parameter("z_min").value)
        self.z_max = float(self.get_parameter("z_max").value)

        self.base_z_max = float(self.get_parameter("base_z_max").value)
        self.snap_to_point = bool(self.get_parameter("snap_to_point").value)
        self.cluster_xy_only = bool(self.get_parameter("cluster_xy_only").value)

        self.side_frame = str(self.get_parameter("side_frame").value)
        self.marker_frame = str(self.get_parameter("marker_frame").value)
        self.force_color = str(self.get_parameter("force_color").value).strip().lower()

        self.csv_path = os.path.expanduser(str(self.get_parameter("csv_path").value))

        self.gps_origin_lat = float(self.get_parameter("gps_origin_lat").value)
        self.gps_origin_lon = float(self.get_parameter("gps_origin_lon").value)
        self.local_xy_rotation_deg = float(
            self.get_parameter("local_xy_rotation_deg").value
        )

        assoc = float(self.get_parameter("association_distance").value)
        alpha = float(self.get_parameter("update_alpha").value)
        dup = float(self.get_parameter("duplicate_threshold").value)

        self.association_distance = assoc if assoc > 0.0 else dup
        self.update_alpha = min(max(alpha, 0.0), 1.0)

        # Saved cones in marker_frame:
        # { "id": int, "x": float, "y": float, "blue_votes": int, "yellow_votes": int }
        self.saved_cones = []
        self.next_cone_id = 1

        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Sub
        self.sub_map = self.create_subscription(
            PointCloud2, "/lidar/points", self.map_callback, 5
        )

        # Pubs
        self.pub_markers_all = self.create_publisher(
            MarkerArray, "/cluster_markers", 20
        )
        self.pub_markers_blue = self.create_publisher(
            MarkerArray, "/cluster_markers_blue", 20
        )
        self.pub_markers_yellow = self.create_publisher(
            MarkerArray, "/cluster_markers_yellow", 20
        )

        self.get_logger().info(
            "PointCloud Cluster Mapper running "
            f"(side_frame={self.side_frame}, marker_frame={self.marker_frame}, "
            f"eps={self.eps}, min_pts={self.min_samples}, assoc_dist={self.association_distance}, "
            f"base_z_max={self.base_z_max}, snap={self.snap_to_point}, xy_only={self.cluster_xy_only})"
        )

    # ----------------------------------------------------------------------
    def _transform_point_latest(self, x, y, z, source_frame, target_frame):

        #  Transform a point (x,y,z) from source_frame -> target_frame using the latest available TF.
        # Returns (x,y,z) in target_frame, or None if transform fails.

        ps = PointStamped()
        ps.header.frame_id = source_frame
        ps.header.stamp = rclpy.time.Time().to_msg()  # latest available
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)

        try:
            out = self.tf_buffer.transform(
                ps, target_frame, timeout=rclpy.duration.Duration(seconds=0.05)
            )
            return out.point.x, out.point.y, out.point.z
        except TransformException as ex:
            self.get_logger().warn(
                f"Transform failed {source_frame} -> {target_frame}: {ex}"
            )
            return None

    # ----------------------------------------------------------------------
    def _classify_colour(self, x_src, y_src, source_frame):

        # Determine cone side (blue/yellow) by transforming point into side_frame (usually base_link)
        # and checking sign of lateral Y.

        if self.force_color in ("blue", "yellow"):
            return self.force_color

        tf_out = self._transform_point_latest(
            x_src, y_src, 0.0, source_frame, self.side_frame
        )
        if tf_out is None:
            return None

        _, ty, _ = tf_out
        # Keep your convention
        return "blue" if ty > 0.0 else "yellow"

    # ----------------------------------------------------------------------
    def _to_marker_frame_xy(self, x_src, y_src, source_frame):

        # Transform (x,y,0) into marker_frame and return (mx,my) for storage/markers

        tf_out = self._transform_point_latest(
            x_src, y_src, 0.0, source_frame, self.marker_frame
        )
        if tf_out is None:
            return None
        mx, my, _ = tf_out
        return mx, my

    # ----------------------------------------------------------------------
    def _nearest_cone_index(self, x, y):

        # Find nearest saved cone to (x,y) in marker_frame. Returns (index, distance) or (None, None)

        if not self.saved_cones:
            return None, None

        best_i = None
        best_d = None
        for i, c in enumerate(self.saved_cones):
            d = math.hypot(c["x"] - x, c["y"] - y)
            if best_d is None or d < best_d:
                best_d = d
                best_i = i
        return best_i, best_d

    # ----------------------------------------------------------------------
    def _associate_or_create(self, x_obs, y_obs, colour_obs):

        # Nearest-neighbour association:
        #   - If observation is within association_distance of a saved cone, update votes and optionally position.
        #   - Else create a new cone entry.
        # Returns True if a new cone was created, False if it was associated.

        i, d = self._nearest_cone_index(x_obs, y_obs)

        if i is not None and d is not None and d < self.association_distance:
            c = self.saved_cones[i]
            a = self.update_alpha
            c["x"] = (1.0 - a) * c["x"] + a * float(x_obs)
            c["y"] = (1.0 - a) * c["y"] + a * float(y_obs)

            if colour_obs == "blue":
                c["blue_votes"] += 1
            elif colour_obs == "yellow":
                c["yellow_votes"] += 1

            return False

        new_cone = {
            "id": int(self.next_cone_id),
            "x": float(x_obs),
            "y": float(y_obs),
            "blue_votes": 1 if colour_obs == "blue" else 0,
            "yellow_votes": 1 if colour_obs == "yellow" else 0,
        }
        self.next_cone_id += 1
        self.saved_cones.append(new_cone)
        return True

    # ----------------------------------------------------------------------
    def _cone_colour(self, cone):

        # Final colour decision based on vote counts.

        return "blue" if cone["blue_votes"] >= cone["yellow_votes"] else "yellow"

    # ----------------------------------------------------------------------
    def _cluster_representative_xy(self, cluster_points_xyz):

        # Compute a representative (x,y) for a cluster in the SOURCE frame.

        # Approach:
        #   1) Prefer near-base points (z <= base_z_max) to reduce influence of sparse top returns.
        #   2) Use median XY for robustness against outliers.
        #   3) Optionally snap that median location to the nearest actual point.

        if cluster_points_xyz.shape[0] == 0:
            return None

        # Prefer near-base points if available
        base = cluster_points_xyz[cluster_points_xyz[:, 2] <= self.base_z_max]
        use = (
            base
            if base.shape[0] >= max(3, self.min_samples // 2)
            else cluster_points_xyz
        )

        # Robust center (median resists outliers)
        center_xy = np.median(use[:, :2], axis=0)
        cx = float(center_xy[0])
        cy = float(center_xy[1])

        if not self.snap_to_point:
            return cx, cy

        # Snap to nearest actual point in XY (guarantees marker sits on point cloud)
        dx = use[:, 0] - cx
        dy = use[:, 1] - cy
        idx = int(np.argmin(dx * dx + dy * dy))
        return float(use[idx, 0]), float(use[idx, 1])

    # ----------------------------------------------------------------------
    def map_callback(self, msg: PointCloud2):

        # Compute a representative (x,y) for a cluster in the SOURCE frame.

        # Approach:
        #   1) Prefer near-base points (z <= base_z_max) to reduce influence of sparse top returns.
        #   2) Use median XY for robustness against outliers.
        #   3) Optionally snap that median location to the nearest actual point.

        raw_points = []
        for p in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            z = float(p[2])
            if self.z_min < z < self.z_max:
                raw_points.append([float(p[0]), float(p[1]), z])

        if not raw_points:
            return

        pts_np = np.array(raw_points, dtype=np.float32)

        # DBSCAN clustering, usually better in XY for cones
        if self.cluster_xy_only:
            fit_data = pts_np[:, :2]
        else:
            fit_data = pts_np

        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(fit_data)
        labels = clustering.labels_
        unique_clusters = [l for l in set(labels) if l != -1]

        source_frame = msg.header.frame_id

        new_count = 0
        assoc_count = 0

        for cluster_id in unique_clusters:
            cluster_points = pts_np[labels == cluster_id]
            rep = self._cluster_representative_xy(cluster_points)
            if rep is None:
                continue
            x_rep, y_rep = rep

            # Place marker in marker_frame, store in marker_frame
            mxy = self._to_marker_frame_xy(x_rep, y_rep, source_frame)
            if mxy is None:
                continue
            mx, my = mxy

            # Colour from side_frame logic (use representative point, not mean centroid)
            colour = self._classify_colour(x_rep, y_rep, source_frame)
            if colour is None:
                continue

            is_new = self._associate_or_create(mx, my, colour)
            if is_new:
                new_count += 1
            else:
                assoc_count += 1

        markers_all = MarkerArray()
        markers_blue = MarkerArray()
        markers_yellow = MarkerArray()

        markers_all.markers.append(self.make_deleteall_marker())
        markers_blue.markers.append(self.make_deleteall_marker())
        markers_yellow.markers.append(self.make_deleteall_marker())

        for c in self.saved_cones:
            colour = self._cone_colour(c)

            markers_all.markers.append(
                self.make_marker(c["id"], c["x"], c["y"], colour, ns="cones_all")
            )
            if colour == "blue":
                markers_blue.markers.append(
                    self.make_marker(c["id"], c["x"], c["y"], "blue", ns="cones_blue")
                )
            else:
                markers_yellow.markers.append(
                    self.make_marker(
                        c["id"], c["x"], c["y"], "yellow", ns="cones_yellow"
                    )
                )

        self.pub_markers_all.publish(markers_all)
        self.pub_markers_blue.publish(markers_blue)
        self.pub_markers_yellow.publish(markers_yellow)

        self.get_logger().info(
            f"Clusters: {len(unique_clusters)} | Saved: {len(self.saved_cones)} | "
            f"New: {new_count} | Associated: {assoc_count}"
        )

    # ----------------------------------------------------------------------
    def make_deleteall_marker(self):

        # """Marker helper: clears all previous markers in the namespace for clean redraw."""

        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.action = Marker.DELETEALL
        return m

    # ----------------------------------------------------------------------
    def make_marker(self, marker_id, x, y, colour, ns="cones"):

        # Create a cylinder marker at (x,y) in marker_frame.
        # Note: Colour is chosen based on your blue/yellow convention.

        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = str(ns)
        m.id = int(marker_id)
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.5

        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.position.z = 0.0

        if colour == "blue":
            m.color = ColorRGBA(r=0.0, g=0.3, b=1.0, a=1.0)
        else:
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        return m

    # ----------------------------------------------------------------------
    def _marker_xy_to_latlon(self, x_m, y_m):

        # Convert local (x,y) in meters back to (lat,lon) using a simple tangent plane approximation.
        # Requires gps_origin_lat/lon to be set (non-zero).

        lat0 = self.gps_origin_lat
        lon0 = self.gps_origin_lon

        if abs(lat0) < 1e-9 and abs(lon0) < 1e-9:
            return None, None

        theta = math.radians(self.local_xy_rotation_deg)
        c = math.cos(theta)
        s = math.sin(theta)

        east_m = c * x_m - s * y_m
        north_m = s * x_m + c * y_m

        lat0_rad = math.radians(lat0)

        dlat = (north_m / WGS84_A) * (180.0 / math.pi)
        dlon = (east_m / (WGS84_A * math.cos(lat0_rad))) * (180.0 / math.pi)

        return lat0 + dlat, lon0 + dlon

    # ----------------------------------------------------------------------
    def save_csv(self):

        # Save the current persistent cone list to CSV.
        # If gps_origin_lat/lon are not set, lat/lon columns are left blank.

        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["lat", "lon", "colour"])

            for c in self.saved_cones:
                colour = self._cone_colour(c)
                lat, lon = self._marker_xy_to_latlon(c["x"], c["y"])
                if lat is None or lon is None:
                    writer.writerow(["", "", colour])
                else:
                    writer.writerow([f"{lat:.10f}", f"{lon:.10f}", colour])

        self.get_logger().info(f"CSV saved: {self.csv_path}")


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
