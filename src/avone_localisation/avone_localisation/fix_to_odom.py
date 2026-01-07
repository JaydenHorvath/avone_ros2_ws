#!/usr/bin/env python3

# fix_to_odom (GNSS fix + heading → local ENU Odometry + TF)
# ----------------------------------------------------------------------------
# ROS 2 node that converts GNSS latitude/longitude from /fix (NavSatFix) into a local ENU position (east,north,up)
# relative to the first valid fix, and publishes:

#   - /odometry/gps   (nav_msgs/Odometry)
#   - TF transform    <world_frame> -> <base_frame>

# It also consumes an external heading quaternion from /heading (QuaternionStamped) and uses that as the odometry
# orientation (with a safe identity fallback if NaNs appear).

# Typical use on AV.ONE:
#   - Quick testing solution to achieve a gps based odometry instead of using the traditional Navsat Transform
#   - Provide a simple "GPS odom" source into robot_localization (EKF) and a TF for visualisation/debug.
#   - Use z_mode="initial" to flatten noisy altitude into a constant plane for 2D racing.

# Topic IO:
#   Subscribes:
#     - /fix      (sensor_msgs/NavSatFix)
#     - /heading  (geometry_msgs/QuaternionStamped)
#   Publishes:
#     - /odometry/gps (nav_msgs/Odometry)
#     - TF: world_frame -> base_frame

# Parameters:
#   - world_frame: parent frame for odom + TF (default "odom")
#   - base_frame : child frame for odom + TF (default "base_link")
#   - z_mode     : "initial" locks Z to first observed Up, "zero" forces Z=0

# Notes / gotchas:
#   - The first valid GNSS fix defines the origin. Restart the node to reset origin.
#   - This file has a duplicate "import math" line (harmless, but clean it up).
#   - Covariance mapping is simplified: only uses position_covariance[0] for both X and Y.
#   - No twist (velocity) is published in Odometry; downstream filters will need another velocity source
#     or will estimate it internally.


import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


# WGS84 ellipsoid constants for LLA->ECEF conversion
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def lla_to_ecef(lat_rad, lon_rad, alt):

    #  Convert geodetic coordinates (lat, lon in radians, alt in meters) into ECEF XYZ (meters).

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1.0 - WGS84_E2) + alt) * sin_lat
    return x, y, z


def ecef_to_enu(x, y, z, lat0_rad, lon0_rad, x0, y0, z0):

    #  Convert ECEF XYZ to local ENU (east,north,up) meters relative to an origin (x0,y0,z0) at (lat0, lon0).

    dx = x - x0
    dy = y - y0
    dz = z - z0
    sin_lat0 = math.sin(lat0_rad)
    cos_lat0 = math.cos(lat0_rad)
    sin_lon0 = math.sin(lon0_rad)
    cos_lon0 = math.cos(lon0_rad)
    e = -sin_lon0 * dx + cos_lon0 * dy
    n = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    u = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return e, n, u


def is_nan_quaternion(q: Quaternion) -> bool:

    # Returns True if any quaternion component is NaN.
    # Prevents TF/odom poisoning with invalid orientation values.

    return math.isnan(q.x) or math.isnan(q.y) or math.isnan(q.z) or math.isnan(q.w)


class FixToOdomWithHeading(Node):
    def __init__(self):

        # Minimal GNSS fix -> ENU odometry converter with external heading injection.

        super().__init__("fix_to_odom_with_heading")

        # -----------------------------
        # Frame parameters
        # -----------------------------
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("base_frame", "gps_link1")
        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # -----------------------------
        # Z-lock behaviour
        # -----------------------------
        # "initial": lock Z to first U measurement (flat plane)

        self.declare_parameter("z_mode", "initial")  # "initial" or "zero"
        self.z_mode = str(self.get_parameter("z_mode").value).lower()
        self.z_plane: Optional[float] = None

        # -----------------------------
        # Origin state (set once)
        # -----------------------------
        self.origin_set = False
        self.lat0_rad = None
        self.lon0_rad = None
        self.x0 = self.y0 = self.z0 = None
        self.latest_q: Optional[Quaternion] = None

        # IO
        self.fix_sub = self.create_subscription(NavSatFix, "/fix", self.on_fix, 10)
        self.heading_sub = self.create_subscription(
            QuaternionStamped, "/heading", self.on_heading, 10
        )
        self.odom_pub = self.create_publisher(Odometry, "/odometry/gps", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f"Publishing /odometry/gps + TF {self.world_frame}->{self.base_frame} | z_mode={self.z_mode}"
        )

    def on_heading(self, msg: QuaternionStamped):

        # Stores the latest heading quaternion.

        # If the incoming quaternion contains NaNs, reset to identity yaw (0).
        # This avoids TF tree corruption and keeps the node publishing stable.

        if is_nan_quaternion(msg.quaternion):
            self.get_logger().warn(
                "Received NaN heading quaternion — resetting to 0 yaw"
            )
            self.latest_q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            self.latest_q = msg.quaternion

    def on_fix(self, msg: NavSatFix):

        # Converts /fix into local ENU and publishes Odometry + TF.

        if msg.status.status < 0:  # no fix
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt = msg.altitude

        # LLA -> ECEF
        x, y, z = lla_to_ecef(lat_rad, lon_rad, alt)

        if not self.origin_set:
            self.lat0_rad = lat_rad
            self.lon0_rad = lon_rad
            self.x0, self.y0, self.z0 = x, y, z
            self.origin_set = True
            self.get_logger().info(
                f"Origin set at lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={alt:.2f}"
            )

        # ECEF -> ENU relative to origin
        e, n, u = ecef_to_enu(
            x, y, z, self.lat0_rad, self.lon0_rad, self.x0, self.y0, self.z0
        )

        # Z locking
        if self.z_mode == "zero":
            z_locked = 0.0
        else:
            if self.z_plane is None:
                self.z_plane = u
                self.get_logger().info(f"Z plane set to {self.z_plane:.3f} m")
            z_locked = self.z_plane

        # -----------------------------
        # Publish Odometry
        # -----------------------------
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = e
        odom.pose.pose.position.y = n
        odom.pose.pose.position.z = z_locked

        # Orientation from heading (fallback identity)
        if self.latest_q is not None and not is_nan_quaternion(self.latest_q):
            odom.pose.pose.orientation = self.latest_q
        else:
            odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Covariance
        cov = [0.0] * 36
        if msg.position_covariance_type in (1, 2, 3):
            cov_xy = msg.position_covariance[0]
            cov[0] = cov_xy
            cov[7] = cov_xy
        cov[14] = 0.01  # small Z covariance
        odom.pose.covariance = cov

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = e
        t.transform.translation.y = n
        t.transform.translation.z = z_locked
        if self.latest_q is not None and not is_nan_quaternion(self.latest_q):
            t.transform.rotation = self.latest_q
        else:
            t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixToOdomWithHeading()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
