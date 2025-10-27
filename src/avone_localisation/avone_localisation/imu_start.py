#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import math
from typing import Optional

class VN100Node(Node):
    def __init__(self):
        super().__init__('vn100_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('gravity_magnitude', 9.80665)
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        self.gravity = self.get_parameter('gravity_magnitude').value
        
        # Serial connection
        self.ser: Optional[serial.Serial] = None
        self.connect_serial(port, baudrate)
        
        # Publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Timer
        self.create_timer(1.0 / rate, self.read_and_publish)
        
        # Statistics
        self.parse_errors = 0
        self.last_error_report = self.get_clock().now()
        
    def connect_serial(self, port: str, baudrate: int) -> bool:
        """Establish serial connection to VN100"""
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.01,  # Non-blocking read
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to VN100 on {port} @ {baudrate} baud")
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open VN100 serial: {e}")
            self.ser = None
            return False
    
    def parse_vnymr(self, line: str) -> Optional[dict]:
        """Parse $VNYMR message from VN100
        
        Format: $VNYMR,yaw,pitch,roll,magX,magY,magZ,accX,accY,accZ,gyroX,gyroY,gyroZ*checksum
        Units: angles in degrees, accel in g, gyro in deg/s
        Convention: NED (North-East-Down)
        """
        if not line.startswith('$VNYMR'):
            return None
        
        # Remove checksum
        body = line.split('*')[0]
        parts = body.split(',')[1:]  # Skip '$VNYMR'
        
        if len(parts) != 12:
            return None
        
        try:
            data = {
                'yaw': float(parts[0]),      # degrees, NED
                'pitch': float(parts[1]),    # degrees, NED
                'roll': float(parts[2]),     # degrees, NED
                'mag': [float(parts[3]), float(parts[4]), float(parts[5])],
                'acc': [float(parts[6]), float(parts[7]), float(parts[8])],  # g
                'gyro': [float(parts[9]), float(parts[10]), float(parts[11])]  # deg/s
            }
            return data
        except (ValueError, IndexError):
            return None
    
    def ned_to_enu_quaternion(self, yaw_deg: float, pitch_deg: float, roll_deg: float) -> tuple:
        """Convert NED Euler angles to ENU quaternion
        
        VN100 outputs: yaw (about down), pitch (about east), roll (about north)
        ROS expects: ENU frame (x=East, y=North, z=Up)
        
        Transformation:
        - NED North → ENU East (x)
        - NED East → ENU North (y)  
        - NED Down → ENU Up (z) [negated]
        
        Returns: (qw, qx, qy, qz)
        """
        # Convert degrees to radians
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)
        roll_rad = math.radians(roll_deg)
        
        # NED to ENU rotation remapping
        # Roll: rotation about North axis → rotation about East (x) axis
        # Pitch: rotation about East axis → rotation about North (y) axis  
        # Yaw: rotation about Down axis → rotation about Up (z) axis (negated)
        roll_enu = roll_rad
        pitch_enu = pitch_rad
        yaw_enu = -yaw_rad
        
        # Convert to quaternion using ZYX Euler convention
        cy = math.cos(yaw_enu * 0.5)
        sy = math.sin(yaw_enu * 0.5)
        cp = math.cos(pitch_enu * 0.5)
        sp = math.sin(pitch_enu * 0.5)
        cr = math.cos(roll_enu * 0.5)
        sr = math.sin(roll_enu * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return (qw, qx, qy, qz)
    
    def ned_to_enu_vector(self, ned_vec: list, is_angular: bool = False) -> Vector3:
        """Convert NED vector to ENU
        
        NED: x=North, y=East, z=Down
        ENU: x=East, y=North, z=Up
        
        For angular rates, sign of z is negated (right-hand rule)
        """
        msg = Vector3()
        msg.x = ned_vec[1]  # East
        msg.y = ned_vec[0]  # North
        msg.z = -ned_vec[2] if is_angular else -ned_vec[2]  # Up
        return msg
    
    def read_and_publish(self):
        """Read from serial and publish IMU message"""
        if self.ser is None or not self.ser.is_open:
            return
        
        try:
            # Read available data
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Parse VNYMR message
                data = self.parse_vnymr(line)
                if data is None:
                    return
                
                # Create IMU message
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                
                # Convert orientation from NED to ENU
                qw, qx, qy, qz = self.ned_to_enu_quaternion(
                    data['yaw'], data['pitch'], data['roll']
                )
                msg.orientation.w = qw
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                
                # Convert angular velocity (deg/s to rad/s, NED to ENU)
                gyro_ned_rad = [math.radians(g) for g in data['gyro']]
                msg.angular_velocity = self.ned_to_enu_vector(gyro_ned_rad, is_angular=True)
                
                # Convert linear acceleration (g to m/s², NED to ENU)
                acc_ned_ms2 = [a * self.gravity for a in data['acc']]
                msg.linear_acceleration = self.ned_to_enu_vector(acc_ned_ms2)
                
                # Set covariances (unknown, so mark as -1 per REP-145)
                msg.orientation_covariance = [-1.0] * 9
                msg.angular_velocity_covariance = [-1.0] * 9
                msg.linear_acceleration_covariance = [-1.0] * 9
                
                # Publish
                self.pub.publish(msg)
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")
            self.ser.close()
            self.ser = None
        except Exception as e:
            self.parse_errors += 1
            # Report errors every 5 seconds
            now = self.get_clock().now()
            if (now - self.last_error_report).nanoseconds > 5e9:
                self.get_logger().warn(
                    f"Parse errors in last 5s: {self.parse_errors} (latest: {e})"
                )
                self.parse_errors = 0
                self.last_error_report = now

def main(args=None):
    rclpy.init(args=args)
    node = VN100Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu
# import serial
# import math


# class VN100Node(Node):
#     def __init__(self):
#         super().__init__('vn100_node')

#         # Try to open serial
#         try:
#             self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
#             self.ser.reset_input_buffer()
#             self.get_logger().info("Connected to VN100 on /dev/ttyUSB0")
#         except serial.SerialException as e:
#             self.get_logger().error(f"Failed to open VN100 serial: {e}")
#             rclpy.shutdown()
#             return

#         # Publisher
#         self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)

#         # Timer @100Hz
#         self.create_timer(0.01, self.read_and_publish)

#     def read_and_publish(self):
#         try:
#             line = self.ser.readline().decode('utf-8', errors='ignore').strip()
#             if not line.startswith('$VNYMR'):
#                 return

#             # Strip leading "$VNYMR," and split before checksum
#             body = line.split('*')[0]
#             parts = body.split(',')[1:]  # skip $VNYMR
#             if len(parts) != 12:
#                 self.get_logger().warn(f"Got {len(parts)} values instead of 12: {parts}")
#                 return

#             # Parse values from VN100 (NED convention, deg and g)
#             roll_ned, pitch_ned, yaw_ned = map(float, parts[0:3])
#             magx, magy, magz = map(float, parts[3:6])
#             accx, accy, accz = map(float, parts[6:9])
#             gyrox, gyroy, gyroz = map(float, parts[9:12])

#             # --- Convert to ROS ENU (x=forward, y=left, z=up) ---

#             # Accelerometer
#             acc_ros_x = accy
#             acc_ros_y = accx
#             acc_ros_z = -accz

#             # Gyro (deg/s → rad/s)
#             gyro_ros_x = math.radians(gyroy)
#             gyro_ros_y = math.radians(gyrox)
#             gyro_ros_z = math.radians(-gyroz)

#             # Orientation (Euler remap so yaw is about z-up)
#             roll_ros = yaw_ned
#             pitch_ros = pitch_ned
#             yaw_ros = -roll_ned

#             cr = math.cos(math.radians(roll_ros) * 0.5)
#             sr = math.sin(math.radians(roll_ros) * 0.5)
#             cp = math.cos(math.radians(pitch_ros) * 0.5)
#             sp = math.sin(math.radians(pitch_ros) * 0.5)
#             cy = math.cos(math.radians(yaw_ros) * 0.5)
#             sy = math.sin(math.radians(yaw_ros) * 0.5)

#             q_w = cr * cp * cy + sr * sp * sy
#             q_x = sr * cp * cy - cr * sp * sy
#             q_y = cr * sp * cy + sr * cp * sy
#             q_z = cr * cp * sy - sr * sp * cy

#             # --- Build ROS Imu message ---
#             msg = Imu()
#             msg.header.stamp = self.get_clock().now().to_msg()
#             msg.header.frame_id = "imu_link"

#             msg.orientation.w = q_w
#             msg.orientation.x = q_x
#             msg.orientation.y = q_y
#             msg.orientation.z = q_z

#             msg.angular_velocity.x = gyro_ros_x
#             msg.angular_velocity.y = gyro_ros_y
#             msg.angular_velocity.z = gyro_ros_z

#             msg.linear_acceleration.x = acc_ros_x
#             msg.linear_acceleration.y = acc_ros_y
#             msg.linear_acceleration.z = acc_ros_z

#             self.pub.publish(msg)

#         except Exception as e:
#             self.get_logger().warn(f"Parse error: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = VN100Node()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if hasattr(node, "ser") and node.ser.is_open:
#             node.ser.close()
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# # ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link imu_link
