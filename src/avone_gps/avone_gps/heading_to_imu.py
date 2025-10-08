#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import tf_transformations  # for quaternion → euler

class HeadingToImu(Node):
    def __init__(self):
        super().__init__('heading_to_imu')

        self.declare_parameter('imu_frame', 'imu_link')
        self.imu_frame = self.get_parameter('imu_frame').value

        # Subscribe to quaternion heading
        self.sub = self.create_subscription(
            QuaternionStamped,
            '/heading',     # input topic with quaternion
            self.cb,
            10
        )

        # Publish as IMU
        self.pub = self.create_publisher(Imu, '/imu/gps', 10)

    def cb(self, msg: QuaternionStamped):
        imu = Imu()
        imu.header = Header()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.imu_frame

        # Copy orientation directly
        imu.orientation = msg.quaternion

        # Optional: compute yaw from quaternion just to sanity-check
        q = msg.quaternion
        quat_list = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat_list)
        self.get_logger().debug(f"Yaw (rad): {yaw:.3f}")

        # Covariances (you can tune these)
        imu.orientation_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05
        ]
        imu.angular_velocity_covariance = [0.0]*9
        imu.linear_acceleration_covariance = [0.0]*9

        # Leave angular velocity & acceleration empty (zero)
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0

        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.8  # static gravity if you want

        self.pub.publish(imu)

def main():
    rclpy.init()
    node = HeadingToImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
