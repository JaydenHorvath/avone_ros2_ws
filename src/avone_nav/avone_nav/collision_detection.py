#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math
import socket
import struct

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class FrontLidarSocketCan(Node):
    def __init__(self):
        super().__init__('Lidar_Collision_Detection')

        # Topics
        self.declare_parameter('cloud_topic', '/quanergy/points')
        self.declare_parameter('filtered_cloud_topic', '/collision_points')

        # Lidar slice parameters
        # Quanergy is mounted facing backwards, so vehicle forward is around 180 deg
        self.declare_parameter('max_distance', 2.0)            # meters
        self.declare_parameter('half_arc_deg', 20.0)           # 40 degree total arc
        self.declare_parameter('center_deg', 180.0)            # arc center in lidar frame
        self.declare_parameter('min_points_for_detection', 3)

        # Z window to avoid ground
        # Tune these based on your frame: for example ground near z=0
        self.declare_parameter('min_z', -0.5)                   # ignore points below 10 cm
        self.declare_parameter('max_z', 2.0)                   # ignore high stuff if you want

        # SocketCAN parameters
        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('can_id', 0x04)
        self.declare_parameter('extended_id', False)
        self.declare_parameter('send_on_every_scan', False)    # if False, send only on change

        cloud_topic = self.get_parameter('cloud_topic').get_parameter_value().string_value
        filtered_topic = self.get_parameter('filtered_cloud_topic').get_parameter_value().string_value

        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.half_arc = self.get_parameter('half_arc_deg').get_parameter_value().double_value
        self.center_deg = self.get_parameter('center_deg').get_parameter_value().double_value
        self.min_points_for_detection = self.get_parameter(
            'min_points_for_detection'
        ).get_parameter_value().integer_value

        self.min_z = self.get_parameter('min_z').get_parameter_value().double_value
        self.max_z = self.get_parameter('max_z').get_parameter_value().double_value

        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.can_id = self.get_parameter('can_id').get_parameter_value().integer_value
        self.extended_id = self.get_parameter('extended_id').get_parameter_value().bool_value
        self.send_on_every_scan = self.get_parameter('send_on_every_scan').get_parameter_value().bool_value

        # Subscribers and publishers
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            cloud_topic,
            self.cloud_cb,
            10
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            filtered_topic,
            10
        )

        # Detection state so we can avoid spamming CAN if not needed
        self.last_detection_state = False

        # Setup SocketCAN
        self.can_socket = None
        self._setup_can_socket()

        self.get_logger().info(
            f"Front lidar socketCAN node ready. "
            f"Sub: {cloud_topic}, Pub: {filtered_topic}, CAN: {self.can_interface}, "
            f"ID: 0x{self.can_id:X}, z in [{self.min_z}, {self.max_z}]"
        )

    def _setup_can_socket(self):
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.can_interface,))
            self.get_logger().info(f"Bound to CAN interface {self.can_interface}")
        except OSError as e:
            self.get_logger().error(f"Failed to open SocketCAN on {self.can_interface}: {e}")
            self.can_socket = None

    def cloud_cb(self, msg: PointCloud2):
        pts_iter = point_cloud2.read_points(
            msg,
            field_names=["x", "y", "z"],
            skip_nans=True
        )

        xs = []
        ys = []
        zs = []

        for p in pts_iter:
            try:
                xs.append(p[0])
                ys.append(p[1])
                zs.append(p[2])
            except Exception:
                xs.append(p['x'])
                ys.append(p['y'])
                zs.append(p['z'])

        if len(xs) == 0:
            self._handle_detection(False)
            return

        x = np.array(xs, dtype=np.float32)
        y = np.array(ys, dtype=np.float32)
        z = np.array(zs, dtype=np.float32)

        # Distance filter
        dist = np.sqrt(x * x + y * y + z * z)
        mask_dist = dist <= self.max_distance

        # Angle filter in lidar frame
        ang = np.degrees(np.arctan2(y, x))
        ang = (ang + 360.0) % 360.0

        left_bound = self.center_deg - self.half_arc
        right_bound = self.center_deg + self.half_arc
        mask_angle = np.logical_and(ang >= left_bound, ang <= right_bound)

        # Z filter to ignore ground etc
        mask_z = np.logical_and(z >= self.min_z, z <= self.max_z)

        # Combine filters
        mask = np.logical_and(mask_dist, np.logical_and(mask_angle, mask_z))

        if not np.any(mask):
            self._handle_detection(False)
            return

        # Extract filtered points
        filtered_pts = np.column_stack((x[mask], y[mask], z[mask]))

        # Publish filtered cloud
        out_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_pts.tolist())
        self.cloud_pub.publish(out_msg)

        # Detection if enough points in the region
        detection = filtered_pts.shape[0] >= self.min_points_for_detection
        self._handle_detection(detection)

    def _handle_detection(self, detection: bool):
        """
        Handle detection logic and send CAN frame if needed.
        Payload:
          byte 0: 1 if obstacle detected, 0 otherwise
          bytes 1..7: zero
        """
        if self.can_socket is None:
            self._setup_can_socket()
            if self.can_socket is None:
                return

        send_can = self.send_on_every_scan or (detection != self.last_detection_state)

        if not send_can:
            self.last_detection_state = detection
            return

        can_id = self.can_id
        if self.extended_id:
            can_id |= socket.CAN_EFF_FLAG

        dlc = 1
        data_bytes = bytes([1 if detection else 0] + [0] * 7)

        frame = struct.pack("=IB3x8s", can_id, dlc, data_bytes)

        try:
            self.can_socket.send(frame)
            if detection:
                self.get_logger().info("Obstacle detected in front lidar sector, CAN frame sent")
            else:
                self.get_logger().info("Obstacle cleared in front lidar sector, CAN frame sent")
        except OSError as e:
            self.get_logger().error(f"Failed to send CAN frame: {e}")

        self.last_detection_state = detection


def main(args=None):
    rclpy.init(args=args)
    node = FrontLidarSocketCan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.can_socket is not None:
            node.can_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
