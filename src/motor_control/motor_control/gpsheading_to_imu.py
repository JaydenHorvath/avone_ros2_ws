#!/usr/bin/env python3
# ROS 2 Humble: Convert NMEA HDT/VTG to sensor_msgs/Imu orientation (yaw)
import math
import re
from typing import Optional

import rclpy
from rclpy.node import Node

from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

def yaw_to_quaternion(yaw_rad: float):
    # roll = pitch = 0, yaw about +Z
    half = yaw_rad * 0.5
    qz = math.sin(half)
    qw = math.cos(half)
    return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

def wrap_pi(a: float) -> float:
    # wrap to [-pi, pi]
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a

class HdtToImuNode(Node):
    def __init__(self):
        super().__init__('hdt_to_imu')

        # Params
        self.declare_parameter('nmea_topic', '/nmea_sentence')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('prefer_hdt', True)
        self.declare_parameter('yaw_offset_deg', 0.0)   # additional offset to apply (mounting, etc.)

        self.nmea_topic = self.get_parameter('nmea_topic').get_parameter_value().string_value
        self.imu_topic  = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.imu_frame  = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.prefer_hdt = self.get_parameter('prefer_hdt').get_parameter_value().bool_value
        self.yaw_offset = math.radians(self.get_parameter('yaw_offset_deg').get_parameter_value().double_value)

        self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)
        self.sub_nmea = self.create_subscription(Sentence, self.nmea_topic, self.on_sentence, 50)

        # Regex for HDT (true heading) and VTG (course over ground)
        self.re_hdt = re.compile(r'^\$..HDT,([0-9]+(?:\.[0-9]*)?),T')
        # VTG: $--VTG,<course_true>,T,<course_mag>,M,<speed_knots>,N,<speed_kmh>,K
        self.re_vtg = re.compile(r'^\$..VTG,([0-9]+(?:\.[0-9]*)?),T,')

        self.last_hdt_deg: Optional[float] = None
        self.last_vtg_deg: Optional[float] = None

        self.get_logger().info(f"Listening for NMEA on {self.nmea_topic}; publishing IMU yaw on {self.imu_topic}")

    def on_sentence(self, msg: Sentence):
        s = msg.sentence.strip()

        # Try HDT first (true heading)
        m_hdt = self.re_hdt.match(s)
        if m_hdt:
            try:
                self.last_hdt_deg = float(m_hdt.group(1))
            except ValueError:
                pass

        # Also parse VTG (course over ground true)
        m_vtg = self.re_vtg.match(s)
        if m_vtg:
            try:
                self.last_vtg_deg = float(m_vtg.group(1))
            except ValueError:
                pass

        # Decide which value to use
        heading_deg = None
        if self.prefer_hdt and self.last_hdt_deg is not None:
            heading_deg = self.last_hdt_deg
        elif self.last_vtg_deg is not None:
            heading_deg = self.last_vtg_deg

        if heading_deg is None:
            return  # nothing to publish yet

        # Convert NMEA heading (degrees clockwise from TRUE NORTH) to ROS ENU yaw:
        # In ENU, yaw=0 is +X (East), positive CCW toward +Y (North).
        # Mapping: yaw_rad = (90° - heading_true) in radians
        yaw = math.radians(90.0 - heading_deg) + self.yaw_offset
        yaw = wrap_pi(yaw)

        imu = Imu()
        imu.header.stamp = msg.header.stamp  # nmea_sentence should carry a header; ok if zero
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quaternion(yaw)

        # Optional covariance: trust orientation moderately (tune for your system)
        # diag variances [roll, pitch, yaw]; here we trust yaw ~ (3 deg)^2
        yaw_var = math.radians(3.0) ** 2
        imu.orientation_covariance = [1e6, 0.0, 0.0,
                                      0.0, 1e6, 0.0,
                                      0.0, 0.0, yaw_var]

        # Leave angular_velocity and linear_acceleration zero/unknown:
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0

        self.pub_imu.publish(imu)

def main():
    rclpy.init()
    node = HdtToImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
