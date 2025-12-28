#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import copy

class ImuCovFix(Node):
    def __init__(self):
        super().__init__("imu_cov_fix")

        # Allow custom topics, but default matches AV.ONE
        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/imu_fixed')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Imu, in_topic, self.cb, 20)
        self.pub = self.create_publisher(Imu, out_topic, 20)

        self.get_logger().info(
            f"IMU covariance fixer listening on {in_topic}, publishing on {out_topic}"
        )

        # Reasonable default covariances for your racecar IMU
        self.orient_cov = [0.0025, 0.0,    0.0,
                           0.0,    0.0025, 0.0,
                           0.0,    0.0,    0.01]

        self.angvel_cov = [0.01,  0.0,   0.0,
                           0.0,   0.01,  0.0,
                           0.0,   0.0,   0.02]

        self.linacc_cov = [0.1,  0.0,   0.0,
                           0.0,  0.1,   0.0,
                           0.0,  0.0,   0.2]

    def cb(self, msg: Imu):
        imu = copy.deepcopy(msg)

        imu.orientation_covariance = self.orient_cov
        imu.angular_velocity_covariance = self.angvel_cov
        imu.linear_acceleration_covariance = self.linacc_cov

        self.pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = ImuCovFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
