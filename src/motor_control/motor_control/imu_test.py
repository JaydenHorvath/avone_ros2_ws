#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
import numpy as np

class VN100Node(Node):
    def __init__(self):
        super().__init__('vn100_node')

        # Open serial
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Timer @100Hz
        self.create_timer(0.01, self.read_and_publish)

    def read_and_publish(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line.startswith('$VNYMR'):
            return

        try:
            # Strip leading "$VNYMR," and split before checksum
            body = line.split('*')[0]
            parts = body.split(',')[1:]  # skip $VNYMR
            if len(parts) != 12:
                self.get_logger().warn(f"Got {len(parts)} values instead of 12: {parts}")
                return

            # Parse values
            roll, pitch, yaw = map(float, parts[0:3])
            magx, magy, magz = map(float, parts[3:6])
            accx, accy, accz = map(float, parts[6:9])
            gyrox, gyroy, gyroz = map(float, parts[9:12])

            # Build ROS IMU msg
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            # Orientation → convert Euler (rad) to quaternion
            cr = math.cos(math.radians(roll) * 0.5)
            sr = math.sin(math.radians(roll) * 0.5)
            cp = math.cos(math.radians(pitch) * 0.5)
            sp = math.sin(math.radians(pitch) * 0.5)
            cy = math.cos(math.radians(yaw) * 0.5)
            sy = math.sin(math.radians(yaw) * 0.5)

            msg.orientation.w = cr*cp*cy + sr*sp*sy
            msg.orientation.x = sr*cp*cy - cr*sp*sy
            msg.orientation.y = cr*sp*cy + sr*cp*sy
            msg.orientation.z = cr*cp*sy - sr*sp*cy

            # Gyro (rad/s)
            msg.angular_velocity.x = math.radians(gyrox)
            msg.angular_velocity.y = math.radians(gyroy)
            msg.angular_velocity.z = math.radians(gyroz)

            # Accel (m/s²)
            msg.linear_acceleration.x = accx
            msg.linear_acceleration.y = accy
            msg.linear_acceleration.z = accz

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VN100Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
