# AV.ONE Sensor Timeout / Heartbeat Monitor (ROS 2 + SocketCAN)
# File: sensor_timeout.py

# Purpose:
#   - Monitor core perception/localisation sensor topics (LiDAR, IMU, GPS) and detect dropouts.
#   - Publish a single "sensors ok" boolean into ROS for higher-level gating.
#   - Transmit a compact 1 byte health bitfield over CAN so embedded safety nodes can react.

# What it does:
#   1) Subscribes to:
#        - LiDAR topic (either PointCloud2 or LaserScan)
#        - IMU topic (sensor_msgs/Imu)
#        - GPS topic (sensor_msgs/NavSatFix)

#   2) Records the last time each sensor message was seen.

#   3) On a fixed period (period_ms), checks the age of the last message from each sensor:
#        - lidar_ok = last_lidar_age <= lidar_timeout_ms
#        - imu_ok   = last_imu_age   <= imu_timeout_ms
#        - gps_ok   = last_gps_age   <= gps_timeout_ms
#          AND the GPS fix quality meets a minimum NavSatStatus threshold

#   4) Publishes:
#        - /av1/sensors_ok (std_msgs/Bool) = True only if all sensors pass their checks

#   5) Sends a CAN frame (1 byte payload) with bit flags:
#        - bit0 (0x01) = LIDAR_OK
#        - bit1 (0x02) = IMU_OK
#        - bit2 (0x04) = GPS_OK
#        - bit7 (0x80) = ALL_OK (lidar_ok AND imu_ok AND gps_ok)

# Notes:
#   - This node is meant to support safety gating on AV.ONE.
#     Typical downstream usage is for a Teensy or actuator controller to refuse enabling
#     autonomous outputs if ALL_OK is not set, or to trigger a safe fallback.
#   - GPS validity is checked two ways:
#       (a) message freshness (timeout)
#       (b) fix quality via NavSatFix.status.status >= gps_minimum_status
#     This avoids treating "stale but still publishing" or "publishing with NO_FIX" as healthy.
#   - CAN transport is raw SocketCAN, so the OS must provide the interface (can0/vcan0 etc.)
#     and it must be up before this node starts.
#   - This hasnt yet been implemented into the fault checks within the teensy nodes, but does publish CAN
#


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
        super().__init__("sensor_heartbeat_node")

        # ----- parameters -----
        self.can_interface = (
            self.declare_parameter("can_interface", "vcan0")
            .get_parameter_value()
            .string_value
        )
        self.status_can_id = (
            self.declare_parameter("status_can_id", 0x120)
            .get_parameter_value()
            .integer_value
        )
        self.period_ms = (
            self.declare_parameter("period_ms", 50).get_parameter_value().integer_value
        )

        self.lidar_topic = (
            self.declare_parameter("lidar_topic", "/quanergy/points")
            .get_parameter_value()
            .string_value
        )
        self.lidar_type = (
            self.declare_parameter("lidar_type", "PointCloud2")
            .get_parameter_value()
            .string_value
        )
        self.imu_topic = (
            self.declare_parameter("imu_topic", "/imu/data")
            .get_parameter_value()
            .string_value
        )
        self.gps_topic = (
            self.declare_parameter("gps_topic", "/navsat1")
            .get_parameter_value()
            .string_value
        )

        self.lidar_timeout = (
            self.declare_parameter("lidar_timeout_ms", 1000)
            .get_parameter_value()
            .integer_value
        )
        self.imu_timeout = (
            self.declare_parameter("imu_timeout_ms", 500)
            .get_parameter_value()
            .integer_value
        )
        self.gps_timeout = (
            self.declare_parameter("gps_timeout_ms", 500)
            .get_parameter_value()
            .integer_value
        )

        # New parameter: minimum acceptable GPS fix quality
        #  -1 NO_FIX
        #   0 FIX
        #   1 SBAS_FIX
        #   2 GBAS_FIX
        self.gps_min_status = (
            self.declare_parameter("gps_minimum_status", 0)
            .get_parameter_value()
            .integer_value
        )

        self.sensors_ok_topic = (
            self.declare_parameter("sensors_ok_topic", "/av1/sensors_ok")
            .get_parameter_value()
            .string_value
        )

        # ----- subs / pubs -----
        if self.lidar_type.lower() == "laserscan":
            self.sub_lidar = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self._cb_lidar_scan,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Listening LiDAR LaserScan on {self.lidar_topic}")
        else:
            self.sub_lidar = self.create_subscription(
                PointCloud2,
                self.lidar_topic,
                self._cb_lidar_pc2,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Listening LiDAR PointCloud2 on {self.lidar_topic}")

        self.sub_imu = self.create_subscription(
            Imu, self.imu_topic, self._cb_imu, qos_profile_sensor_data
        )
        self.sub_gps = self.create_subscription(
            NavSatFix, self.gps_topic, self._cb_gps, qos_profile_sensor_data
        )

        self.pub_ok = self.create_publisher(Bool, self.sensors_ok_topic, 10)

        # last-seen times (rclpy Time or None)
        self.last_lidar: Optional[rclpy.time.Time] = None
        self.last_imu: Optional[rclpy.time.Time] = None
        self.last_gps: Optional[rclpy.time.Time] = None

        # store latest GPS fix status
        self.last_gps_status: Optional[int] = None

        # ----- open CAN (SocketCAN raw) -----
        self.can_sock = self._open_can(self.can_interface)

        # periodic timer
        self.timer = self.create_timer(self.period_ms / 1000.0, self._on_timer)

        self.get_logger().info(
            f"Heartbeat(bits only) on {self.can_interface} id=0x{self.status_can_id:X} @ {self.period_ms}ms "
            f"(timeouts lidar={self.lidar_timeout} imu={self.imu_timeout} gps={self.gps_timeout} min_gps_status={self.gps_min_status})"
        )

    # --- subs ---
    def _cb_lidar_pc2(self, _msg: PointCloud2):
        self.last_lidar = self.get_clock().now()

    def _cb_lidar_scan(self, _msg: LaserScan):
        self.last_lidar = self.get_clock().now()

    def _cb_imu(self, _msg: Imu):
        self.last_imu = self.get_clock().now()

    def _cb_gps(self, msg: NavSatFix):
        self.last_gps = self.get_clock().now()
        self.last_gps_status = msg.status.status

    # --- CAN helpers ---
    def _open_can(self, iface: str) -> socket.socket:
        s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        s.bind((iface,))
        return s

    def _send_can(self, can_id: int, data: bytes):
        dlc = min(len(data), 8)
        frame = struct.pack("=IB3x8s", can_id, dlc, data.ljust(8, b"\x00"))
        try:
            self.can_sock.send(frame)
        except OSError as e:
            self.get_logger().throttle(5.0, f"CAN send failed: {e}")

    # --- helper ---
    def _age_ms(self, t: Optional[rclpy.time.Time]) -> int:
        if t is None:
            return 2**63 - 1
        now = self.get_clock().now()
        return int((now - t).nanoseconds / 1_000_000)

    # --- timer ---
    def _on_timer(self):
        la = self._age_ms(self.last_lidar)
        ia = self._age_ms(self.last_imu)
        ga = self._age_ms(self.last_gps)

        lidar_ok = la <= self.lidar_timeout
        imu_ok = ia <= self.imu_timeout

        # GPS must pass both timeout and fix-quality checks
        timeout_ok = ga <= self.gps_timeout
        status_ok = (
            self.last_gps_status is not None
            and self.last_gps_status >= self.gps_min_status
        )
        gps_ok = timeout_ok and status_ok

        # final overall status
        all_ok = lidar_ok and imu_ok and gps_ok

        # publish ROS topic
        self.pub_ok.publish(Bool(data=all_ok))

        # CAN byte
        b0 = 0
        if lidar_ok:
            b0 |= 0x01
        if imu_ok:
            b0 |= 0x02
        if gps_ok:
            b0 |= 0x04
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


if __name__ == "__main__":
    main()
