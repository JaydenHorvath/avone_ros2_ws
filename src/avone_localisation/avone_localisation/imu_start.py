#!/usr/bin/env python3
# imu_start (VN-100T IMU serial driver → sensor_msgs/Imu)
# --------------------------------------------------------------------------
# ROS 2 driver for the VectorNav VN-100T (VN100) IMU that reads $VNYMR NMEA sentences over a serial port
# and publishes sensor_msgs/Imu on `imu/data`.

# What it does:
#   - Opens a serial port (default /dev/ttyUSB0 @ 115200)
#   - Reads lines until it finds $VNYMR messages
#   - Parses yaw, pitch, roll, accel (m/s^2), gyro (rad/s)
#   - Converts VN heading/pitch/roll into ROS FLU convention and publishes:
#       * orientation (quaternion)
#       * angular_velocity (rad/s)
#       * linear_acceleration (m/s^2)
#   - Applies FRD -> FLU sign flips to accel and gyro vectors
#   - Sets covariance matrices from parameters (tunable)
#   - Optional live console compass output for quick sanity checks

# Coordinate conventions (important):
#   - IMU body convention assumed here: FRD (X forward, Y right, Z down)
#   - ROS REP-103 convention: FLU (X forward, Y left, Z up)
#   - The accel/gyro vectors are transformed by:
#       X_flu =  X_frd
#       Y_flu = -Y_frd
#       Z_flu = -Z_frd

# Yaw conversion logic used:
#   - VN-100T yaw_deg is compass heading: 0°=North, 90°=East, increases clockwise
#   - ROS yaw is about +Z (up), increases counter-clockwise, and 0 aligns with +X (forward)
#   - This code maps: yaw_ros = radians(90 - yaw_deg)

# Parameters:
#   - serial_port (string)              : device path
#   - baud_rate (int)                   : serial baud
#   - frame_id (string)                 : imu frame id in published message
#   - publish_rate (float)              : timer rate (Hz); actual publish depends on serial data arriving
#   - show_compass (bool)               : print heading/pitch/roll to terminal

#   Covariance tuning:
#   - orientation_covariance_roll (float)    : roll variance (rad^2)
#   - orientation_covariance_pitch (float)   : pitch variance (rad^2)
#   - orientation_covariance_yaw (float)     : yaw variance (rad^2)
#   - angular_velocity_covariance (float)    : gyro variance (rad^2/s^2)
#   - linear_acceleration_covariance (float) : accel variance ((m/s^2)^2)

# Notes / gotchas:
#   - The code assumes the VN outputs gyro already in rad/s (some devices output deg/s depending on config).
#   - publish_rate controls how often the node checks the serial buffer, not guaranteed output rate.
#   - If you see inconsistent yaw sign, revisit both:
#       * yaw_ros = radians(90 - yaw_deg)
#       * pitch/roll sign flips

import math
import serial
import numpy as np
import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Force unbuffered output
os.environ["PYTHONUNBUFFERED"] = "1"
sys.stdout.reconfigure(line_buffering=True)


def compass_to_cardinal(compass_deg):
    # Convert compass bearing (0°=North, 90°=East) to cardinal direction.
    dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    ix = int((compass_deg + 22.5) // 45) % 8
    return dirs[ix]


def euler_to_quaternion(roll, pitch, yaw):

    # Convert Euler angles (roll, pitch, yaw) to quaternion.
    # Assumes ZYX rotation order (yaw -> pitch -> roll).

    # Args:
    #     roll: Rotation around X-axis (radians)
    #     pitch: Rotation around Y-axis (radians)
    #     yaw: Rotation around Z-axis (radians)

    # Returns:
    #     (qx, qy, qz, qw): Quaternion components

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class VN100TNode(Node):
    def __init__(self):
        super().__init__("vn100t_imu_node")

        # Declare parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("show_compass", False)

        # Covariance parameters (adjustable based on your IMU specs and testing)
        # VN-100T typical specifications:
        # - Heading accuracy: ~2° RMS (with mag calibration)
        # - Pitch/Roll accuracy: ~0.5° RMS
        # - Gyro noise: ~0.0035 rad/s (0.2 deg/s)
        # - Accel noise: ~0.04 m/s²

        self.declare_parameter(
            "orientation_covariance_roll", 0.0001
        )  # (0.5° = 0.0087 rad)² ≈ 0.0001
        self.declare_parameter(
            "orientation_covariance_pitch", 0.0001
        )  # (0.5° = 0.0087 rad)² ≈ 0.0001
        self.declare_parameter(
            "orientation_covariance_yaw", 0.001
        )  # (2° = 0.035 rad)² ≈ 0.001

        self.declare_parameter(
            "angular_velocity_covariance", 0.00001
        )  # (0.0035 rad/s)² ≈ 0.00001
        self.declare_parameter(
            "linear_acceleration_covariance", 0.0016
        )  # (0.04 m/s²)² = 0.0016

        # Get parameters
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.rate = float(self.get_parameter("publish_rate").value)
        self.show_compass = bool(self.get_parameter("show_compass").value)

        # Get covariance parameters
        cov_roll = self.get_parameter("orientation_covariance_roll").value
        cov_pitch = self.get_parameter("orientation_covariance_pitch").value
        cov_yaw = self.get_parameter("orientation_covariance_yaw").value
        cov_gyro = self.get_parameter("angular_velocity_covariance").value
        cov_accel = self.get_parameter("linear_acceleration_covariance").value

        # Build covariance matrices (row-major 3x3)
        # Format: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
        self.orientation_covariance = [
            cov_roll,
            0.0,
            0.0,
            0.0,
            cov_pitch,
            0.0,
            0.0,
            0.0,
            cov_yaw,
        ]

        self.angular_velocity_covariance = [
            cov_gyro,
            0.0,
            0.0,
            0.0,
            cov_gyro,
            0.0,
            0.0,
            0.0,
            cov_gyro,
        ]

        self.linear_acceleration_covariance = [
            cov_accel,
            0.0,
            0.0,
            0.0,
            cov_accel,
            0.0,
            0.0,
            0.0,
            cov_accel,
        ]

        # Open serial port
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
            self.get_logger().info(f"✅ VN-100T connected on {port} @ {baud} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to open {port}: {e}")
            self.ser = None

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)

        # Create timer
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        # Coordinate frame transformations
        # VN-100T outputs data in NED frame (North-East-Down)
        # IMU body frame: X-forward, Y-right, Z-down (FRD - aligned with NED when pointing North)
        # ROS2 REP-103 standard: X-forward, Y-left, Z-up (FLU)

        # Transformation: FRD -> FLU
        # X_flu = X_frd (forward stays forward)
        # Y_flu = -Y_frd (right becomes left)
        # Z_flu = -Z_frd (down becomes up)

        self.R_frd_to_flu = np.array(
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
        )

        self.get_logger().info("VN-100T IMU driver initialized")
        self.get_logger().info("Body frame: X=forward, Y=right, Z=down")
        self.get_logger().info("Publishing in ROS2 standard: X=forward, Y=left, Z=up")
        self.get_logger().info(
            f"Orientation covariance: roll={cov_roll:.6f}, pitch={cov_pitch:.6f}, yaw={cov_yaw:.6f}"
        )
        self.get_logger().info(f"Angular velocity covariance: {cov_gyro:.6f}")
        self.get_logger().info(f"Linear acceleration covariance: {cov_accel:.6f}")

    def timer_callback(self):
        """Main callback to read and publish IMU data."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            # Read available data
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                # Parse $VNYMR sentence
                if not line.startswith("$VNYMR"):
                    return

                # Split and validate
                parts = line.split("*")[0].split(",")
                if len(parts) < 13:
                    self.get_logger().warn(
                        f"Incomplete VNYMR sentence: {len(parts)} fields"
                    )
                    return

                # Parse VN-100T data
                # Format: $VNYMR,yaw,pitch,roll,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ*checksum
                yaw_deg = float(
                    parts[1]
                )  # Compass heading: 0°=North, 90°=East, 180°=South, 270°=West
                pitch_deg = float(parts[2])  # Pitch (nose up/down)
                roll_deg = float(parts[3])  # Roll (right wing down/up)

                # Linear acceleration in body frame (m/s²)
                ax_body = float(parts[7])
                ay_body = float(parts[8])
                az_body = float(parts[9])

                # Angular velocity in body frame (rad/s)
                gx_body = float(parts[10])
                gy_body = float(parts[11])
                gz_body = float(parts[12])

                # Convert compass heading to ROS2 convention
                # VN-100T: 0° = North (Y-axis in NED), increasing clockwise
                # ROS2: Yaw around Z-up, 0° = X-axis (forward), increasing counter-clockwise
                # When IMU points North: yaw should be +90° (pointing along +Y in ENU/FLU)
                yaw_ros = math.radians(90.0 - yaw_deg)  # Convert to ROS convention
                pitch_ros = math.radians(-pitch_deg)  # Flip pitch for FLU frame
                roll_ros = math.radians(-roll_deg)  # Flip roll for FLU frame

                # Convert to quaternion
                qx, qy, qz, qw = euler_to_quaternion(roll_ros, pitch_ros, yaw_ros)

                # Transform acceleration from FRD to FLU
                accel_frd = np.array([ax_body, ay_body, az_body])
                accel_flu = self.R_frd_to_flu @ accel_frd

                # Transform angular velocity from FRD to FLU
                gyro_frd = np.array([gx_body, gy_body, gz_body])
                gyro_flu = self.R_frd_to_flu @ gyro_frd

                # Create and publish IMU message
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id

                # Orientation (quaternion)
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                msg.orientation.w = qw

                # Angular velocity (rad/s)
                msg.angular_velocity.x = gyro_flu[0]
                msg.angular_velocity.y = gyro_flu[1]
                msg.angular_velocity.z = gyro_flu[2]

                # Linear acceleration (m/s²)
                msg.linear_acceleration.x = accel_flu[0]
                msg.linear_acceleration.y = accel_flu[1]
                msg.linear_acceleration.z = accel_flu[2]

                # Set covariances
                msg.orientation_covariance = self.orientation_covariance
                msg.angular_velocity_covariance = self.angular_velocity_covariance
                msg.linear_acceleration_covariance = self.linear_acceleration_covariance

                self.imu_pub.publish(msg)

                # Display compass heading
                if self.show_compass:
                    compass_bearing = yaw_deg
                    direction = compass_to_cardinal(compass_bearing)
                    yaw_ros_deg = math.degrees(yaw_ros)
                    yaw_ros_deg = (yaw_ros_deg + 360.0) % 360.0

                    print(
                        f"\033[96m🧭 Compass: {compass_bearing:6.2f}° ({direction:2s}) | "
                        f"ROS2 Yaw: {yaw_ros_deg:6.2f}° | "
                        f"Pitch: {pitch_deg:5.1f}° | Roll: {roll_deg:5.1f}°\033[0m",
                        flush=True,
                    )

        except ValueError as e:
            self.get_logger().warn(f"Failed to parse IMU data: {e}")
        except Exception as e:
            self.get_logger().error(f"IMU read error: {e}")

    def destroy_node(self):
        """Clean up serial port on shutdown."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VN100TNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Shutting down VN-100T driver...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
