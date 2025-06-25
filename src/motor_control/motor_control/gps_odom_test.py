#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import utm
import math

class GPSRelativeNode(Node):
    def __init__(self):
        super().__init__('gps_relative')
        self.origin = None
        self.heading = 0.0

        # subscribe to the fused GPS fix
        self.sub_fix = self.create_subscription(
            NavSatFix, '/navsat', self.fix_cb, 10)

        # subscribe to the RTK heading (degrees)
        self.sub_heading = self.create_subscription(
            Float64, '/heading', self.heading_cb, 10)

        # publish a PoseStamped with position + orientation
        self.pub_pose = self.create_publisher(
            PoseStamped, '/gps1_pose', 10)

    def heading_cb(self, msg: Float64):
        # store the latest heading in degrees
        self.heading = msg.data

    def fix_cb(self, msg: NavSatFix):
        # convert lat/lon → UTM
        e, n, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        if self.origin is None:
            self.origin = (e, n)

        # relative X,Y
        x = e - self.origin[0]
        y = n - self.origin[1]

        # build the PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'   # or 'odom', whichever you use
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # convert heading (deg) → quaternion about Z
        # yaw = heading clockwise from North, but ROS uses right-handed:
        heading_rad = math.radians(self.heading)

        # initial conversion: 0° → +Y, 90° → +X
        ros_yaw = math.pi/2 - heading_rad

        # flip the arrow 180°
        ros_yaw += math.pi

        # wrap into [-π, π)
        ros_yaw = (ros_yaw + math.pi) % (2*math.pi) - math.pi

        half = ros_yaw * 0.5
        qz   = math.sin(half)
        qw   = math.cos(half)

       
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        # publish it
        self.pub_pose.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = GPSRelativeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
