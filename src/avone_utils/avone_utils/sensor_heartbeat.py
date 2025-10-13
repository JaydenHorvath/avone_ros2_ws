#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# AV.ONE Sensor Heartbeat (Python / rclpy + SocketCAN)
# Payload (1 byte):
#   bit0 LIDAR_OK, bit1 IMU_OK, bit2 GPS_OK, bit7 ALL_OK

import struct
import socket
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import PointCloud2, LaserScan, Imu, NavSatFix
from std_msgs.msg import Bool


class SensorHeartbeatNode(Node):
    def __init__(self):
        super().__init__('sensor_heartbeat_node')

        # ----- parameters -----
        self.can_interface   = self.declare_parameter('can_interface', 'vcan0').get_parameter_value().string_value
        self.status_can_id   = self.declare_parameter('status_can_id', 0x120).get_parameter_value().integer_value
        self.period_ms       = self.declare_parameter('period_ms', 50).get_parameter_value().integer_value

        self.lidar_topic     = self.declare_parameter('lidar_topic', '/lidar/points').get_parameter_value().string_value
        self.lidar_type      = self.declare_parameter('lidar_type', 'PointCloud2').get_parameter_value().string_value  # or LaserScan
        self.imu_topic       = self.declare_parameter('imu_topic', '/imu').get_parameter_value().string_value
        self.gps_topic       = self.declare_parameter('gps_topic', '/navsat1').get_parameter_value().string_value

        self.lidar_timeout   = self.declare_parameter('lidar_timeout_ms', 1000).get_parameter_value().integer_value
        self.imu_timeout     = self.declare_parameter('imu_timeout_ms',   500).get_parameter_value().integer_value
        self.gps_timeout     = self.declare_parameter('gps_timeout_ms',   500).get_parameter_value().integer_value

        self.sensors_ok_topic = self.declare_parameter('sensors_ok_topic', '/av1/sensors_ok').get_parameter_value().string_value

        # ----- subs / pubs -----
        if self.lidar_type.lower() == 'laserscan':
            self.sub_lidar = self.create_subscription(LaserScan, self.lidar_topic, self._cb_lidar_scan, qos_profile_sensor_data)
            self.get_logger().info(f'Listening LiDAR LaserScan on {self.lidar_topic}')
        else:
            self.sub_lidar = self.create_subscription(PointCloud2, self.lidar_topic, self._cb_lidar_pc2, qos_profile_sensor_data)
            self.get_logger().info(f'Listening LiDAR PointCloud2 on {self.lidar_topic}')

        self.sub_imu = self.create_subscription(Imu, self.imu_topic, self._cb_imu, qos_profile_sensor_data)
        self.sub_gps = self.create_subscription(NavSatFix, self.gps_topic, self._cb_gps, qos_profile_sensor_data)

        self.pub_ok = self.create_publisher(Bool, self.sensors_ok_topic, 10)

        # last-seen times (rclpy Time or None)
        self.last_lidar: Optional[rclpy.time.Time] = None
        self.last_imu:   Optional[rclpy.time.Time] = None
        self.last_gps:   Optional[rclpy.time.Time] = None

        # ----- open CAN (SocketCAN raw) -----
        self.can_sock = self._open_can(self.can_interface)

        # periodic timer
        self.timer = self.create_timer(self.period_ms / 1000.0, self._on_timer)

        self.get_logger().info(
            f'Heartbeat(bits only) on {self.can_interface} id=0x{self.status_can_id:X} @ {self.period_ms}ms '
            f'(timeouts lidar={self.lidar_timeout} imu={self.imu_timeout} gps={self.gps_timeout})'
        )

    # --- subs ---
    def _cb_lidar_pc2(self, _msg: PointCloud2): self.last_lidar = self.get_clock().now()
    def _cb_lidar_scan(self, _msg: LaserScan):  self.last_lidar = self.get_clock().now()
    def _cb_imu(self, _msg: Imu):               self.last_imu   = self.get_clock().now()
    def _cb_gps(self, _msg: NavSatFix):         self.last_gps   = self.get_clock().now()

    # --- CAN helpers ---
    def _open_can(self, iface: str) -> socket.socket:
        # Bring iface up externally: sudo ip link set can0 up type can bitrate 500000
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
        return s

    def _send_can(self, can_id: int, data: bytes):
        # struct can_frame { can_id:u32, can_dlc:u8, __pad:u8, __res0:u8, __res1:u8, data[8] }
        dlc = min(len(data), 8)
        frame = struct.pack("=IB3x8s", can_id, dlc, data.ljust(8, b'\x00'))
        try:
            self.can_sock.send(frame)
        except OSError as e:
            self.get_logger().throttle(5.0, f'CAN send failed: {e}')

    # --- timer ---
    def _age_ms(self, t: Optional[rclpy.time.Time]) -> int:
        if t is None:
            return 2**63 - 1
        now = self.get_clock().now()
        return int((now - t).nanoseconds / 1_000_000)

    def _on_timer(self):
        la = self._age_ms(self.last_lidar)
        ia = self._age_ms(self.last_imu)
        ga = self._age_ms(self.last_gps)

        lidar_ok = la <= self.lidar_timeout
        imu_ok   = ia <= self.imu_timeout
        gps_ok   = ga <= self.gps_timeout
        all_ok   = lidar_ok and imu_ok and gps_ok

        # ROS Bool for convenience
        self.pub_ok.publish(Bool(data=all_ok))

        # Build single-byte payload: bit0 LIDAR, bit1 IMU, bit2 GPS, bit7 ALL_OK
        b0 = (0x01 if lidar_ok else 0) | (0x02 if imu_ok else 0) | (0x04 if gps_ok else 0)
        if all_ok:
            b0 |= 0x80

        self._send_can(self.status_can_id, bytes((b0,)))


def main():
    rclpy.init()
    try:
        node = SensorHeartbeatNode()
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
