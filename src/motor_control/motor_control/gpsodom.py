#!/usr/bin/env python3
# ROS 2 Humble: Make odometry from /fix (NavSatFix) + /heading (QuaternionStamped)

import math
from math import sin, cos, sqrt, radians, isfinite
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf2_ros

# WGS-84
A  = 6378137.0                 # semi-major axis [m]
E2 = 6.6943799901413165e-3     # first eccentricity squared

def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat = radians(lat_deg)
    lon = radians(lon_deg)
    chi = 1.0 / sqrt(1.0 - E2 * (sin(lat) ** 2))
    x = (A * chi + h) * cos(lat) * cos(lon)
    y = (A * chi + h) * cos(lat) * sin(lon)
    z = (A * (1.0 - E2) * chi + h) * sin(lat)
    return x, y, z

def ecef_to_enu(x, y, z, lat0_deg, lon0_deg, h0, x0, y0, z0):
    lat0 = radians(lat0_deg)
    lon0 = radians(lon0_deg)
    dx, dy, dz = x - x0, y - y0, z - z0
    slat, clat = sin(lat0), cos(lat0)
    slon, clon = sin(lon0), cos(lon0)
    e = -slon * dx +  clon * dy
    n = -clon * slat * dx - slon * slat * dy + clat * dz
    u =  clon * clat * dx + slon * clat * dy + slat * dz
    return e, n, u

class GPSHeadingOdomNode(Node):
    def __init__(self):
        super().__init__('gps_heading_odometry')

        # Params
        self.declare_parameter('fix_topic', '/fix')
        self.declare_parameter('heading_topic', '/heading')  # QuaternionStamped
        self.declare_parameter('odom_topic', '/odometry/gps')
        self.declare_parameter('world_frame', 'odom')        # output frame
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps')           # expected frame_id for /fix and /heading
        self.declare_parameter('zero_altitude', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('velocity_smoothing', 0.0)    # 0..1 EMA on velocity, 0=off
        self.declare_parameter('set_origin_from_first_fix', True)
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('origin_alt', 0.0)

        self.fix_topic     = self.get_parameter('fix_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.odom_topic    = self.get_parameter('odom_topic').value
        self.world_frame   = self.get_parameter('world_frame').value
        self.base_link     = self.get_parameter('base_link_frame').value
        self.gps_frame     = self.get_parameter('gps_frame').value
        self.zero_alt      = self.get_parameter('zero_altitude').value
        self.publish_tf    = self.get_parameter('publish_tf').value
        self.v_ema_alpha   = float(self.get_parameter('velocity_smoothing').value)
        self.use_first_fix_as_origin = self.get_parameter('set_origin_from_first_fix').value

        # Origin (datum) state
        self.origin_set = False
        self.lat0 = self.lon0 = self.h0 = 0.0
        self.x0 = self.y0 = self.z0 = 0.0

        # Latest heading and previous ENU for velocity
        self.last_heading: Optional[QuaternionStamped] = None
        self.prev_t: Optional[Time] = None
        self.prev_e = self.prev_n = self.prev_u = None
        self.vx_ema = self.vy_ema = self.vz_ema = 0.0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Sensor-data QoS for NMEA-like streams
        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Subscribers
        self.fix_sub     = self.create_subscription(NavSatFix, self.fix_topic, self.on_fix, sensor_qos)
        self.heading_sub = self.create_subscription(QuaternionStamped, self.heading_topic, self.on_heading, sensor_qos)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(f"Building odom from {self.fix_topic} + {self.heading_topic} → {self.odom_topic}")
        self.get_logger().info(f"World frame: {self.world_frame}, base: {self.base_link}, gps frame expected: {self.gps_frame}")

        # If a fixed origin is supplied
        if not self.use_first_fix_as_origin:
            self.lat0 = float(self.get_parameter('origin_lat').value)
            self.lon0 = float(self.get_parameter('origin_lon').value)
            self.h0   = float(self.get_parameter('origin_alt').value)
            self.x0, self.y0, self.z0 = geodetic_to_ecef(self.lat0, self.lon0, self.h0)
            self.origin_set = True
            self.get_logger().info(f"Using configured origin: lat={self.lat0:.8f}, lon={self.lon0:.8f}, alt={self.h0:.2f}")

    def on_heading(self, msg: QuaternionStamped):
        self.last_heading = msg

    def _safe_cov9(self, fix: NavSatFix):
        """Return a 9-length covariance list with sane defaults and finite entries."""
        cov_default = [25.0, 0.0, 0.0,
                       0.0, 25.0, 0.0,
                       0.0, 0.0, 100.0]
        try:
            seq = fix.position_covariance
            if len(seq) == 9:
                tmp = [float(x) for x in seq]
                for i in range(9):
                    if not isfinite(tmp[i]):
                        tmp[i] = cov_default[i]
                return tmp
        except Exception:
            pass
        return cov_default

    def on_fix(self, msg: NavSatFix):
        # 1) Require a valid fix
        if msg.status.status < 0:
            return

        # 2) Frame sanity (warn once)
        if msg.header.frame_id and msg.header.frame_id != self.gps_frame:
            self.get_logger().warn_once(f"/fix frame_id is '{msg.header.frame_id}', expected '{self.gps_frame}'")
        if self.last_heading and self.last_heading.header.frame_id and self.last_heading.header.frame_id != self.gps_frame:
            self.get_logger().warn_once(f"/heading frame_id is '{self.last_heading.header.frame_id}', expected '{self.gps_frame}'")

        # 3) Geodetic -> ECEF
        lat = msg.latitude
        lon = msg.longitude
        alt = 0.0 if self.zero_alt else msg.altitude
        x, y, z = geodetic_to_ecef(lat, lon, alt)

        # 4) Initialize origin (datum) from first valid fix if requested
        if not self.origin_set and self.use_first_fix_as_origin:
            self.lat0, self.lon0, self.h0 = lat, lon, alt
            self.x0, self.y0, self.z0 = x, y, z
            self.origin_set = True
            self.get_logger().info(f"Origin set: lat={lat:.8f}, lon={lon:.8f}, alt={alt:.2f}")

        if not self.origin_set:
            return  # wait until origin is set

        # 5) ECEF -> ENU
        e, n, u = ecef_to_enu(x, y, z, self.lat0, self.lon0, self.h0, self.x0, self.y0, self.z0)

        # 6) Timestamp and velocity estimate (with optional EMA smoothing)
        now = Time.from_msg(msg.header.stamp) if msg.header.stamp.sec != 0 else self.get_clock().now()
        vx = vy = vz = 0.0
        if self.prev_t is not None and self.prev_e is not None:
            dt = (now - self.prev_t).nanoseconds * 1e-9
            if dt > 1e-3:
                vx = (e - self.prev_e) / dt
                vy = (n - self.prev_n) / dt
                vz = (u - self.prev_u) / dt
                a = self.v_ema_alpha
                if a > 0.0:
                    self.vx_ema = a * vx + (1.0 - a) * self.vx_ema
                    self.vy_ema = a * vy + (1.0 - a) * self.vy_ema
                    self.vz_ema = a * vz + (1.0 - a) * self.vz_ema
                    vx, vy, vz = self.vx_ema, self.vy_ema, self.vz_ema
        self.prev_t, self.prev_e, self.prev_n, self.prev_u = now, e, n, u

        # 7) Orientation: prefer valid /heading, else fallback fixed yaw
        q = self._heading_quat_or_fallback()

        # 8) Build Odometry (pose + twist)
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_link
        odom.pose.pose.position.x = float(e)
        odom.pose.pose.position.y = float(n)
        odom.pose.pose.position.z = float(0.0 if self.zero_alt else u)
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = float(vz)

        # 9) Covariance: sanitize to exactly 36 native floats (no NaN/Inf)
        pc_raw = self._safe_cov9(msg)  # returns 9 finite floats or defaults
        def safe(v, default):
            try:
                vf = float(v)
                return vf if math.isfinite(vf) else default
            except Exception:
                return default
        cov36 = [
            safe(pc_raw[0], 25.0), 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, safe(pc_raw[4], 25.0), 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, safe(pc_raw[8], 100.0), 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, float(math.radians(5.0)**2), 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, float(math.radians(5.0)**2), 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, float(math.radians(3.0)**2)
        ]
        odom.pose.covariance = cov36  # exact length 36, all native floats

        # 10) Publish odometry
        self.odom_pub.publish(odom)

        # 11) Optional TF: world_frame -> base_link
        if self.publish_tf and self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.base_link
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def _heading_quat_or_fallback(self) -> Quaternion:
        """
        Returns a valid orientation quaternion.
        Uses the latest /heading if valid, otherwise uses fallback heading.
        """
        # If we have a heading message, try to use it
        if self.last_heading is not None:
            q = self.last_heading.quaternion
            # Validate quaternion (check for NaNs, Infs, zero-norm)
            vals = [q.x, q.y, q.z, q.w]
            if all(math.isfinite(v) for v in vals):
                n2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
                if n2 > 1e-12:  # valid, normalize it
                    n = math.sqrt(n2)
                    q.x /= n
                    q.y /= n
                    q.z /= n
                    q.w /= n
                    return q
            # If heading data invalid, warn once
            if not hasattr(self, "_warned_heading_invalid"):
                self.get_logger().warn("Invalid /heading quaternion detected — using fallback heading instead.")
                self._warned_heading_invalid = True

        # Fallback: build quaternion from fallback_heading_deg param
        fallback_deg = float(self.get_parameter('fallback_heading_deg').value) \
            if self.has_parameter('fallback_heading_deg') else 0.0
        yaw_rad = math.radians(fallback_deg)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = GPSHeadingOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
