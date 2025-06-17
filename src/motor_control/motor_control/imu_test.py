#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import numpy as np
from ahrs.filters import Madgwick

class ImuMadgwickPublisher(Node):
    def __init__(self):
        super().__init__('imu_madgwick_publisher')

        # 1) Open serial port to the Leonardo
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f"Cannot open serial port: {e}")
            raise SystemExit

        # 2) ROS publisher for IMU data
        self.pub = self.create_publisher(Imu, 'imu/data', 10)

        # 3) TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4) Madgwick filter @100 Hz, initialise quaternion state
        self.filter = Madgwick(frequency=100.0)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])

        # 5) Timer @100 Hz
        self.create_timer(0.01, self.read_and_publish)

        self.get_logger().info("IMU Madgwick node started. Waiting for data...")

    def read_and_publish(self):
        # Read one line of raw data
        raw = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
        self.get_logger().debug(f"[RAW ] → '{raw}'")
        if not raw:
            return

        parts = raw.split(',')
        if len(parts) != 6:
            self.get_logger().warn(f"[PARSE] Expected 6 values, got {len(parts)}")
            return

        try:
            ax, ay, az, gx, gy, gz = map(float, parts)
        except ValueError as e:
            self.get_logger().warn(f"[PARSE] {e}")
            return

        # Convert to SI units
        ax /= 16384.0             # raw → g
        ay /= 16384.0
        az /= 16384.0
        gx = math.radians(gx / 131.0)  # raw → °/s → rad/s
        gy = math.radians(gy / 131.0)
        gz = -math.radians(gz / 131.0)

        # Update Madgwick filter (IMU-only) and save quaternion
        # signature: updateIMU(q, gyr, acc) → ndarray([w, x, y, z])
        self.q = self.filter.updateIMU(
            self.q,
            gyr=np.array([gx, gy, gz]),
            acc=np.array([ax, ay, az])
        )
        w, x, y, z = self.q

        # Build IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.w = float(w)
        msg.orientation.x = float(x)
        msg.orientation.y = float(y)
        msg.orientation.z = float(z)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax * 9.81
        msg.linear_acceleration.y = ay * 9.81
        msg.linear_acceleration.z = az * 9.81

        # Publish IMU data
        self.pub.publish(msg)
        self.get_logger().info("[PUB  ] IMU msg published")

        # Broadcast TF from base_link → imu_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        # If your IMU is offset, adjust the translation here:
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -1.0
        t.transform.rotation.w = float(w)
        t.transform.rotation.x = float(x)
        t.transform.rotation.y = float(y)
        t.transform.rotation.z = float(z)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuMadgwickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
