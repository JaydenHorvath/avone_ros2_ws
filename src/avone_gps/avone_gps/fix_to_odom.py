#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped, TwistStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3

def lla_to_ecef(lat_rad, lon_rad, alt):
    sin_lat = math.sin(lat_rad); cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad); cos_lon = math.cos(lon_rad)
    N = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1.0 - WGS84_E2) + alt) * sin_lat
    return x, y, z

def ecef_to_enu(x, y, z, lat0_rad, lon0_rad, x0, y0, z0):
    dx = x - x0; dy = y - y0; dz = z - z0
    sin_lat0 = math.sin(lat0_rad); cos_lat0 = math.cos(lat0_rad)
    sin_lon0 = math.sin(lon0_rad); cos_lon0 = math.cos(lon0_rad)
    e = -sin_lon0*dx + cos_lon0*dy
    n = -sin_lat0*cos_lon0*dx - sin_lat0*sin_lon0*dy + cos_lat0*dz
    u =  cos_lat0*cos_lon0*dx + cos_lat0*sin_lon0*dy + sin_lat0*dz
    return e, n, u

def quat_multiply(q1, q2):
    x1,y1,z1,w1 = q1.x,q1.y,q1.z,q1.w
    x2,y2,z2,w2 = q2.x,q2.y,q2.z,q2.w
    return Quaternion(
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    )

def quat_conj(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)

def rotate_vector_by_quat(vx, vy, vz, q: Quaternion):
    vq = Quaternion(x=vx, y=vy, z=vz, w=0.0)
    t = quat_multiply(q, vq)
    r = quat_multiply(t, quat_conj(q))
    return r.x, r.y, r.z

class FixToOdom(Node):
    def __init__(self):
        super().__init__('fix_to_odom')
        # Frames
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Antenna offset (in base_link frame): [x_fwd, y_left, z_up] meters
        self.declare_parameter('antenna_offset', [0.0, 0.0, 0.0])
        self.antenna_offset = self.get_parameter('antenna_offset').value
        if len(self.antenna_offset) != 3:
            self.antenna_offset = [0.0, 0.0, 0.0]

        # Z-lock behavior
        # "initial" -> keep Z at first computed ENU height
        # "zero"    -> keep Z at 0.0
        self.declare_parameter('z_mode', 'initial')
        self.z_mode = str(self.get_parameter('z_mode').value).lower()
        if self.z_mode not in ('initial', 'zero'):
            self.z_mode = 'initial'
        self.z_plane: Optional[float] = None

        # State
        self.origin_set = False
        self.lat0_rad = None
        self.lon0_rad = None
        self.x0 = self.y0 = self.z0 = None
        self.latest_q: Optional[Quaternion] = None
        self.latest_twist: Optional[TwistStamped] = None

        # IO
        self.fix_sub = self.create_subscription(NavSatFix, '/fix', self.on_fix, 10)
        self.heading_sub = self.create_subscription(QuaternionStamped, '/heading', self.on_heading, 10)
        self.vel_sub = self.create_subscription(TwistStamped, '/vel', self.on_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/gps', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Publishing /odometry/gps + TF {self.world_frame}->{self.base_frame} | '
            f'antenna_offset={self.antenna_offset} | z_mode={self.z_mode}'
        )

    def on_heading(self, msg: QuaternionStamped):
        self.latest_q = msg.quaternion

    def on_vel(self, msg: TwistStamped):
        self.latest_twist = msg

    def on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:  # no fix
            return

        lat_rad = math.radians(msg.latitude)
        lon_rad = math.radians(msg.longitude)
        alt = msg.altitude

        x, y, z = lla_to_ecef(lat_rad, lon_rad, alt)

        if not self.origin_set:
            self.lat0_rad = lat_rad
            self.lon0_rad = lon_rad
            self.x0, self.y0, self.z0 = x, y, z
            self.origin_set = True
            self.get_logger().info(f'Origin set at lat={msg.latitude:.8f}, lon={msg.longitude:.8f}, alt={alt:.2f}')

        # Antenna position in ENU
        e_ant, n_ant, u_ant = ecef_to_enu(x, y, z, self.lat0_rad, self.lon0_rad, self.x0, self.y0, self.z0)

        # Convert antenna ENU -> base_link ENU by removing rotated offset
        ex, ey, ez = self.antenna_offset  # in base_link frame (x fwd, y left, z up)
        if self.latest_q is not None and (ex != 0.0 or ey != 0.0 or ez != 0.0):
            ox, oy, oz = rotate_vector_by_quat(ex, ey, ez, self.latest_q)
            e_base = e_ant - ox
            n_base = n_ant - oy
            u_base = u_ant - oz
        else:
            e_base, n_base, u_base = e_ant, n_ant, u_ant

        # --- Z LOCK: keep z on a plane ---
        if self.z_mode == 'zero':
            z_locked = 0.0
        else:  # 'initial'
            if self.z_plane is None:
                self.z_plane = u_base
                self.get_logger().info(f'Z plane set to initial ENU height: {self.z_plane:.3f} m')
            z_locked = self.z_plane

        # Publish Odometry (XY only; Z locked)
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = e_base
        odom.pose.pose.position.y = n_base
        odom.pose.pose.position.z = z_locked  # locked Z

        if self.latest_q is not None:
            odom.pose.pose.orientation = self.latest_q
        else:
            odom.pose.pose.orientation.w = 1.0

        # Covariance: keep XY from GPS; clamp Z covariance small (flat plane)
        cov = [0.0]*36
        if msg.position_covariance_type in (1,2,3):
            cov_xy = msg.position_covariance[0]
            cov[0] = cov_xy    # xx
            cov[7] = cov_xy    # yy
        cov[14] = 0.01         # zz small since we lock Z
        odom.pose.covariance = cov

        # Twist: forward XY; force Z velocity to 0 on the plane
        if self.latest_twist is not None:
            odom.twist.twist = self.latest_twist.twist
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0  # heading-only systems typically don't give roll/pitch rates
            odom.twist.twist.angular.y = 0.0

        self.odom_pub.publish(odom)

        # Publish dynamic TF: odom -> base_link (Z locked)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = e_base
        t.transform.translation.y = n_base
        t.transform.translation.z = z_locked
        if self.latest_q is not None:
            t.transform.rotation = self.latest_q
        else:
            t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = FixToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
