#!/usr/bin/env python3
"""
ROS2 Node for reading VectorNav IMU data from serial port (VNYMR format)
Orientation convention: X forward, Y right, Z down
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math


class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'vn100_imu_link')
        self.declare_parameter('parent_frame', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', False)  # NEW PARAM: enable/disable TF broadcast
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to serial port: {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_conn = None
        
        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # TF broadcaster (only if enabled)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        
        # Create timer for reading serial data
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('IMU Serial Node initialized')
        self.get_logger().info(f'Publishing on topic: imu/data')
        self.get_logger().info(f'Frame ID: {self.frame_id}, Parent: {self.parent_frame}')
        self.get_logger().info(f'Broadcast TF: {self.publish_tf}')
        self.get_logger().info('Expecting VectorNav VNYMR format')
    
    def parse_vnymr(self, line):
        """Parse VectorNav VNYMR NMEA sentence."""
        try:
            line = line.strip()
            if not line.startswith('$VNYMR'):
                return None
            
            parts = line.split('*')
            if len(parts) != 2:
                return None
            
            data_parts = parts[0].split(',')
            if len(data_parts) != 13:
                self.get_logger().warn(f'VNYMR: Expected 13 fields, got {len(data_parts)}')
                return None
            
            yaw = float(data_parts[1])
            pitch = float(data_parts[2])
            roll = float(data_parts[3])
            accel_x = -float(data_parts[7])
            accel_y = -float(data_parts[8])
            accel_z = -float(data_parts[9])
            gyro_x = float(data_parts[10])
            gyro_y = float(data_parts[11])
            gyro_z = float(data_parts[12])
            
            yaw_rad = math.radians(yaw)
            pitch_rad = math.radians(pitch)
            roll_rad = math.radians(roll)
            
            qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
            
            return {
                'orientation': (qx, qy, qz, qw),
                'angular_velocity': (gyro_x, gyro_y, gyro_z),
                'linear_acceleration': (accel_x, accel_y, accel_z),
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            }
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse VNYMR data: {e}')
            return None
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw
    
    def timer_callback(self):
        """Read serial data and publish IMU message"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore')
                data = self.parse_vnymr(line)
                
                if not data:
                    return

                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id
                imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = data['orientation']
                imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = data['angular_velocity']
                imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = data['linear_acceleration']
                imu_msg.orientation_covariance[0] = -1.0
                imu_msg.angular_velocity_covariance[0] = -1.0
                imu_msg.linear_acceleration_covariance[0] = -1.0
                
                self.imu_pub.publish(imu_msg)

                if self.publish_tf:
                    self.broadcast_transform(data['orientation'])
                    
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
    
    def broadcast_transform(self, orientation):
        """Broadcast TF transform for RViz visualization"""
        if not self.tf_broadcaster:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        qx, qy, qz, qw = orientation
        # 180° rotation around X-axis
        q_rx, q_rw = math.sin(math.pi / 2.0), math.cos(math.pi / 2.0)
        q_ry = q_rz = 0.0
        
        rotated_qx = q_rw * qx + q_rx * qw + q_ry * qz - q_rz * qy
        rotated_qy = q_rw * qy - q_rx * qz + q_ry * qw + q_rz * qx
        rotated_qz = q_rw * qz + q_rx * qy - q_ry * qx + q_rz * qw
        rotated_qw = q_rw * qw - q_rx * qx - q_ry * qy - q_rz * qz
        
        t.transform.rotation.x = rotated_qx
        t.transform.rotation.y = rotated_qy
        t.transform.rotation.z = rotated_qz
        t.transform.rotation.w = rotated_qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
