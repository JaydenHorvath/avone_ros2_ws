#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

import utm
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class GPSRelativeNode(Node):
    def __init__(self):
        super().__init__('gps_relative')

        # origin for relative UTM
        self.origin = None

        # last heading (radians)
        self.heading = 0.0

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # path builder
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # subscribers
        self.create_subscription(NavSatFix, '/fix', self.gps_cb, 10)
        self.create_subscription(Float64,   '/heading', self.hdg_cb, 10)

        # publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/gpsold_pose', 10)
        self.path_pub = self.create_publisher(Path,        '/gps_path',     10)

    def hdg_cb(self, msg: Float64):
        # convert degrees → radians
        self.heading = math.radians(msg.data)

    def gps_cb(self, msg: NavSatFix):
        # lat/lon → UTM
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        if self.origin is None:
            self.origin = (easting, northing)

        # relative coordinates
        x = easting  - self.origin[0]
        y = northing - self.origin[1]

        # build the quaternion from current heading
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.heading)

        # --- publish PoseStamped ---
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        # --- append & publish Path ---
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        # --- broadcast TF map → gps_frame ---
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id  = 'gps_frame'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = GPSRelativeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
