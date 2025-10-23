#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import QuaternionStamped, TwistStamped
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker


def yaw_from_quaternion(q):
    """Convert quaternion to yaw (radians, ENU)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """Convert yaw (rad) into quaternion (x=0,y=0)."""
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class HeadingVelVisualizer(Node):
    def __init__(self):
        super().__init__('heading_vel_visualizer')

        # ---------------- Parameters ----------------
        self.declare_parameter('invert_heading', False)
        self.declare_parameter('heading_offset_deg', -90.0)


        self.invert_heading = self.get_parameter('invert_heading').value
        self.heading_offset_deg = self.get_parameter('heading_offset_deg').value
        use_sim_time = self.get_parameter('use_sim_time').value

        # Get or set use_sim_time if passed via CLI or launch
        if self.has_parameter('use_sim_time'):
            use_sim_time = self.get_parameter('use_sim_time').value
        else:
            use_sim_time = False

        if use_sim_time:
            self.get_logger().info("Using simulation time (use_sim_time=True)")
            self.set_parameters([rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True
            )])


        # ---------------- Subscribers ----------------
        self.sub_heading = self.create_subscription(
            QuaternionStamped, "/heading", self.heading_cb, 10)
        self.sub_vel = self.create_subscription(
            TwistStamped, "/vel", self.vel_twist_cb, 10)

        # ---------------- Publishers ----------------
        self.imu_pub = self.create_publisher(Imu, "/imu_gps", 10)
        self.marker_pub = self.create_publisher(Marker, "/vel_marker", 10)

        self.latest_yaw = None
        self.latest_vel = None

    # ---------------- Callbacks ----------------
    def heading_cb(self, msg: QuaternionStamped):
        """Convert /heading quaternion into yaw, apply offset, publish fake IMU."""
        yaw = yaw_from_quaternion(msg.quaternion)

        # Flip 180° if requested
        if self.invert_heading:
            yaw = yaw - math.pi

        # Apply configurable degree offset
        yaw += math.radians(self.heading_offset_deg)

        yaw = -yaw

        # Normalize [-pi, pi]
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.latest_yaw = yaw

        # Publish fake IMU
        imu_msg = Imu()
        imu_msg.header.stamp = msg.header.stamp
        imu_msg.header.frame_id = "gps_link"
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_pub.publish(imu_msg)

        self.update_marker()

    def vel_twist_cb(self, msg: TwistStamped):
        self.latest_vel = msg.twist.linear
        self.update_marker()

    def update_marker(self):
        if self.latest_yaw is None or self.latest_vel is None:
            return

        vel = self.latest_vel
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        if speed < 0.01:
            return

        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "velocity_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # Orientation: from velocity vector
        yaw = math.atan2(vel.y, vel.x)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        # Scale
        marker.scale.x = speed
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


# ---------------- Main Entry ----------------
def main(args=None):
    rclpy.init(args=args)
    node = HeadingVelVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
