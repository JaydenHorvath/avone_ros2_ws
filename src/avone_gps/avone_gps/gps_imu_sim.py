#!/usr/bin/env python3
"""
Gazebo-style static odometry and IMU publisher (sim-time aware)

Mimics Gazebo /odom and /imu behavior (stationary at origin with realistic covariances)
so robot_localization will accept the data and produce base_link -> odom TF.

- Uses rclpy clock for timestamps (honors /clock when use_sim_time:=true)
- Declares 'use_sim_time' (default True) safely, without double-declare crashes
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class GazeboLikeOdom(Node):
    def __init__(self):
        super().__init__('gazebo_like_odom')

        # --- Parameters ---
        # Safe-declare use_sim_time with default True so the node runs in sim time if /clock is present.
        try:
            self.declare_parameter('use_sim_time', True)
        except ParameterAlreadyDeclaredException:
            # Some launch setups auto-declare this; that's fine.
            pass

        self.declare_parameter('rate_hz', 50.0)
        self.rate = float(self.get_parameter('rate_hz').value)
        self.dt = 1.0 / self.rate

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, '/odometry/sim', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/sim', 10)

        # --- Timer (honors sim time when use_sim_time is true and /clock is available) ---
        self.timer = self.create_timer(self.dt, self.timer_cb)

        # --- Stationary pose at origin ---
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        # --- Gazebo-like covariances ---
        # Pose covariance (keep very small or zero to emulate a "frozen" pose source)
        self.odom_pose_cov = [0.0] * 36
        # Twist covariance (small non-zero so EKF doesn't instantly reject)
        self.odom_twist_cov = [0.0] * 36
        self.odom_twist_cov[0] = 0.05   # vx
        self.odom_twist_cov[7] = 0.05   # vy
        self.odom_twist_cov[35] = 0.5   # wz

        # IMU covariances (small/static)
        self.imu_orientation_cov = [0.0] * 9
        self.imu_angular_vel_cov = [0.0] * 9
        self.imu_linear_accel_cov = [0.0] * 9

        self.get_logger().info(
            "Publishing stationary Gazebo-like /odometry/sim and /imu/sim "
            "(sim-time aware: set use_sim_time:=true and provide /clock)."
        )

    def timer_cb(self):
        # Use rclpy clock (this is sim time when use_sim_time is true and /clock is active)
        stamp = self.get_clock().now().to_msg()
        self.publish_odometry(stamp)
        self.publish_imu(stamp)

    def publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Pose (fixed at origin)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.pose.covariance = self.odom_pose_cov

        # Tiny twist noise (like idle dynamics)
        msg.twist.twist.linear.x = random.uniform(-3e-19, 3e-19)
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = random.uniform(-7e-33, 7e-33)
        msg.twist.covariance = self.odom_twist_cov

        self.odom_pub.publish(msg)

    def publish_imu(self, stamp):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        # Orientation = flat, level
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.orientation_covariance = self.imu_orientation_cov

        # Angular velocity ~ 0
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = random.uniform(-7e-33, 7e-33)
        msg.angular_velocity_covariance = self.imu_angular_vel_cov

        # Linear acceleration = gravity only
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance = self.imu_linear_accel_cov

        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = GazeboLikeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
