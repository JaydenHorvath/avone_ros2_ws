#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import socket
import struct
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class FrontLidarSocketCan(Node):
    def __init__(self):
        super().__init__('Lidar_Collision_Detection')

        # Parameters
        self.declare_parameter('cloud_topic', '/quanergy/points')
        self.declare_parameter('filtered_cloud_topic', '/collision_points')

        # Car frame detection arc
        self.declare_parameter('center_deg', 0.0)        # 0 degrees = vehicle forward
        self.declare_parameter('half_arc_deg', 15.0)     # +/- 20 degrees

        # Lidar mounting rotation relative to vehicle frame
        # Example: lidar rotated 90 degrees left -> lidar_yaw_deg = +90
        # Example: lidar rotated 180 degrees (facing backward) -> +180
        self.declare_parameter('lidar_yaw_deg', 0.0)

        # Distance limits
        self.declare_parameter('max_distance', 3.0)

        # Z filtering
        self.declare_parameter('min_z', -0.3)
        self.declare_parameter('max_z', 2.0)

        # Detection logic
        self.declare_parameter('min_points_for_detection', 3)

        # CAN settings
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 0x04)
        self.declare_parameter('extended_id', False)
        self.declare_parameter('send_on_every_scan', False)

        # Resolve params
        cloud_topic = self.get_parameter('cloud_topic').value
        filtered_topic = self.get_parameter('filtered_cloud_topic').value

        self.center_deg = float(self.get_parameter('center_deg').value)
        self.half_arc = float(self.get_parameter('half_arc_deg').value)
        self.lidar_yaw = float(self.get_parameter('lidar_yaw_deg').value)

        self.max_distance = float(self.get_parameter('max_distance').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)

        self.min_points_for_detection = int(self.get_parameter('min_points_for_detection').value)

        self.can_interface = self.get_parameter('can_interface').value
        self.can_id = self.get_parameter('can_id').value
        self.extended_id = self.get_parameter('extended_id').value
        self.send_on_every_scan = self.get_parameter('send_on_every_scan').value

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

        # Internal detection state
        self.last_detection_state = False

        # Setup CAN
        self.can_socket = None
        self._setup_can_socket()

        self.get_logger().info(
            f"Front lidar node ready. Using center {self.center_deg} deg, yaw offset {self.lidar_yaw} deg"
        )

    # ---------------- CAN SETUP ---------------- #

    def _setup_can_socket(self):
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.bind((self.can_interface,))
            self.get_logger().info(f"Bound to CAN interface {self.can_interface}")
        except OSError as e:
            self.get_logger().error(f"Failed to open CAN interface {self.can_interface}: {e}")
            self.can_socket = None

    # ---------------- POINT CLOUD HANDLING ---------------- #

    def cloud_cb(self, msg):

        pts_iter = point_cloud2.read_points(msg, field_names=["x", "y", "z"], skip_nans=True)

        xs = []
        ys = []
        zs = []

        for p in pts_iter:
            # p may be a tuple or numpy structured record
            try:
                xs.append(float(p[0]))
                ys.append(float(p[1]))
                zs.append(float(p[2]))
            except Exception:
                xs.append(float(p['x']))
                ys.append(float(p['y']))
                zs.append(float(p['z']))

        if len(xs) == 0:
            self._handle_detection(False)
            return

        x = np.array(xs, dtype=np.float32)
        y = np.array(ys, dtype=np.float32)
        z = np.array(zs, dtype=np.float32)


        # Distance filter
        dist = np.sqrt(x * x + y * y + z * z)
        mask_dist = dist <= self.max_distance

        # Compute raw lidar angle
        ang = np.degrees(np.arctan2(y, x))   # -180..+180

        # Convert to vehicle frame by removing lidar yaw offset
        ang = ang - self.lidar_yaw

        # Normalize to 0..360
        ang = (ang + 360.0) % 360.0

        # Car forward arc window
        left_bound = (self.center_deg - self.half_arc + 360.0) % 360.0
        right_bound = (self.center_deg + self.half_arc + 360.0) % 360.0

        if left_bound < right_bound:
            mask_angle = np.logical_and(ang >= left_bound, ang <= right_bound)
        else:
            # Window wraps around 0 degrees
            mask_angle = np.logical_or(ang >= left_bound, ang <= right_bound)

        # Z filter
        mask_z = np.logical_and(z >= self.min_z, z <= self.max_z)

        # Combined mask
        mask = np.logical_and(mask_z, np.logical_and(mask_dist, mask_angle))

        if not np.any(mask):
            self._handle_detection(False)
            return

        filtered_pts = np.column_stack((x[mask], y[mask], z[mask]))


        # Publish filtered cloud
        out_msg = point_cloud2.create_cloud_xyz32(msg.header, filtered_pts.tolist())
        self.cloud_pub.publish(out_msg)

        # Detection threshold
        detection = filtered_pts.shape[0] >= self.min_points_for_detection
        self._handle_detection(detection)

    # ---------------- CAN LOGIC ---------------- #

    def _handle_detection(self, detection):

        if self.can_socket is None:
            self._setup_can_socket()
            if self.can_socket is None:
                return

        send_can = self.send_on_every_scan or (detection != self.last_detection_state)

        if not send_can:
            self.last_detection_state = detection
            return

        # CAN ID format
        can_id = self.can_id
        if self.extended_id:
            can_id |= socket.CAN_EFF_FLAG

        data_bytes = bytes([1 if detection else 0] + [0] * 7)

        frame = struct.pack("=IB3x8s", can_id, 1, data_bytes)

        try:
            self.can_socket.send(frame)
            if detection:
                self.get_logger().info("Obstacle detected and CAN sent")
            else:
                self.get_logger().info("Obstacle cleared and CAN sent")
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
        if node.can_socket:
            node.can_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
