#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class HeadingToImu(Node):
    def __init__(self):
        super().__init__('heading_to_imu')
        self.declare_parameter('imu_frame', 'imu_link')
        self.imu_frame = self.get_parameter('imu_frame').value

        self.sub = self.create_subscription(QuaternionStamped, '/heading', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

    def cb(self, msg: QuaternionStamped):
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation = msg.quaternion
        # leave covariances unknown (-1) except orientation
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(imu)

def main():
    rclpy.init()
    node = HeadingToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
launch/gps_viz.launch.py