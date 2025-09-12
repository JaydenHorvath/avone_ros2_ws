#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math


class VN100Node(Node):
    def __init__(self):
        super().__init__('vn100_node')

        # Try to open serial
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info("Connected to VN100 on /dev/ttyUSB0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open VN100 serial: {e}")
            rclpy.shutdown()
            return

        # Publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # Timer @100Hz
        self.create_timer(0.01, self.read_and_publish)

    def read_and_publish(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith('$VNYMR'):
                return

            # Strip leading "$VNYMR," and split before checksum
            body = line.split('*')[0]
            parts = body.split(',')[1:]  # skip $VNYMR
            if len(parts) != 12:
                self.get_logger().warn(f"Got {len(parts)} values instead of 12: {parts}")
                return

            # Parse values from VN100 (NED convention, deg and g)
            roll_ned, pitch_ned, yaw_ned = map(float, parts[0:3])
            magx, magy, magz = map(float, parts[3:6])
            accx, accy, accz = map(float, parts[6:9])
            gyrox, gyroy, gyroz = map(float, parts[9:12])

            # --- Convert to ROS ENU (x=forward, y=left, z=up) ---

            # Accelerometer
            acc_ros_x = accy
            acc_ros_y = accx
            acc_ros_z = -accz

            # Gyro (deg/s → rad/s)
            gyro_ros_x = math.radians(gyroy)
            gyro_ros_y = math.radians(gyrox)
            gyro_ros_z = math.radians(-gyroz)

            # Orientation (Euler remap so yaw is about z-up)
            roll_ros = yaw_ned
            pitch_ros = pitch_ned
            yaw_ros = -roll_ned

            cr = math.cos(math.radians(roll_ros) * 0.5)
            sr = math.sin(math.radians(roll_ros) * 0.5)
            cp = math.cos(math.radians(pitch_ros) * 0.5)
            sp = math.sin(math.radians(pitch_ros) * 0.5)
            cy = math.cos(math.radians(yaw_ros) * 0.5)
            sy = math.sin(math.radians(yaw_ros) * 0.5)

            q_w = cr * cp * cy + sr * sp * sy
            q_x = sr * cp * cy - cr * sp * sy
            q_y = cr * sp * cy + sr * cp * sy
            q_z = cr * cp * sy - sr * sp * cy

            # --- Build ROS Imu message ---
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            msg.orientation.w = q_w
            msg.orientation.x = q_x
            msg.orientation.y = q_y
            msg.orientation.z = q_z

            msg.angular_velocity.x = gyro_ros_x
            msg.angular_velocity.y = gyro_ros_y
            msg.angular_velocity.z = gyro_ros_z

            msg.linear_acceleration.x = acc_ros_x
            msg.linear_acceleration.y = acc_ros_y
            msg.linear_acceleration.z = acc_ros_z

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
        if hasattr(node, "ser") and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
