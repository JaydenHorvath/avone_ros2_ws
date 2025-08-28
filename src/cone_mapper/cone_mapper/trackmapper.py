#!/usr/bin/env python3
# Record waypoints during a teach lap → YAML for Nav2 follow/replay.

import math
import os
import yaml
from dataclasses import dataclass
from typing import Optional, List

# ---- NumPy & PyYAML shims ---------------------------------------------------
import numpy as _np
if not hasattr(_np, "float"):   # NumPy >= 1.24 removed np.float alias
    _np.float = float

yaml.add_representer(_np.float32, lambda d, v: d.represent_float(float(v)))
yaml.add_representer(_np.float64, lambda d, v: d.represent_float(float(v)))
yaml.add_representer(_np.int32,   lambda d, v: d.represent_int(int(v)))
yaml.add_representer(_np.int64,   lambda d, v: d.represent_int(int(v)))
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float


def yaw_from_quat(q: Quaternion) -> float:
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw


def quat_from_yaw(yaw: float) -> Quaternion:
    x, y, z, w = quaternion_from_euler(0.0, 0.0, yaw)
    q = Quaternion()
    q.x, q.y, q.z, q.w = float(x), float(y), float(z), float(w)
    return q


class WaypointRecorder(Node):
    """
    Teach pass:
      • Samples current pose in world_frame (map/odom) while you RC-drive
      • Adds a point when distance/heading change exceeds thresholds or on joystick button
      • Saves Nav2-compatible YAML on shutdown or via /save_waypoints
    """
    def __init__(self):
        super().__init__("waypoint_recorder")

        # ---------- params ----------
        self.declare_parameter("world_frame", "map")           # set "odom" if no global map/RTK
        self.declare_parameter("base_link_frame", "base_link")
        self.declare_parameter("output_yaml", "waypoints.yaml")
        self.declare_parameter("auto_sample", True)
        self.declare_parameter("sample_period", 2.0)          # seconds
        self.declare_parameter("min_dist", 3.0)                # meters between samples
        self.declare_parameter("min_heading_deg", 6.0)         # deg change to keep
        self.declare_parameter("close_loop", False)            # append first point at end
        self.declare_parameter("joy_button_index", 0)          # press to force a waypoint
        self.declare_parameter("use_tf_first", True)           # prefer TF(world->base_link)
        self.declare_parameter("odometry_topic", "/odometry/global")  # fallback topic
        self.declare_parameter("dedupe_radius", 0.05)          # 5 cm dedupe on save

        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.base_frame  = self.get_parameter("base_link_frame").get_parameter_value().string_value
        self.output_yaml = self.get_parameter("output_yaml").get_parameter_value().string_value
        self.auto_sample = bool(self.get_parameter("auto_sample").value)
        self.sample_period = float(self.get_parameter("sample_period").value)
        self.min_dist = float(self.get_parameter("min_dist").value)
        self.min_head = math.radians(float(self.get_parameter("min_heading_deg").value))
        self.close_loop = bool(self.get_parameter("close_loop").value)
        self.joy_btn_idx = int(self.get_parameter("joy_button_index").value)
        self.use_tf_first = bool(self.get_parameter("use_tf_first").value)
        self.odom_topic = self.get_parameter("odometry_topic").get_parameter_value().string_value
        self.dedupe_radius = float(self.get_parameter("dedupe_radius").value)

        # ---------- TF & subs ----------
        self.tfbuf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        # IMPORTANT: use main executor (no extra thread) to avoid shutdown races
        self.tflist = tf2_ros.TransformListener(self.tfbuf, self, spin_thread=False)

        qos_be = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.last_odom: Optional[Odometry] = None
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_be)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)

        # ---------- timer ----------
        self.timer = self.create_timer(self.sample_period, self._tick)

        # ---------- state ----------
        self.path: List[Waypoint] = []
        self.last_pose: Optional[Waypoint] = None
        self.btn_prev = False

        # ---------- service ----------
        self.srv = self.create_service(Trigger, "save_waypoints", self._save_service_cb)

        self.get_logger().info(
            f"WaypointRecorder ready. world_frame={self.world_frame}, output={self.output_yaml}\n"
            f"Drive the teach lap; press joystick button {self.joy_btn_idx} to force a waypoint.\n"
            f"Call:  ros2 service call /save_waypoints std_srvs/srv/Trigger '{{}}'"
        )

    # ---------- callbacks ----------
    def _odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def _joy_cb(self, msg: Joy):
        pressed = len(msg.buttons) > self.joy_btn_idx and msg.buttons[self.joy_btn_idx] == 1
        if pressed and not self.btn_prev:
            pose = self.current_pose()
            if pose:
                self._add_point(pose, force=True)
                self.get_logger().info(
                    f"Forced waypoint #{len(self.path)} @ ({pose.x:.2f}, {pose.y:.2f})"
                )
        self.btn_prev = pressed

    def _tick(self):
        if not self.auto_sample:
            return
        pose = self.current_pose()
        if pose:
            self._add_point(pose, force=False)

    def _save_service_cb(self, _req, _resp):
        resp = Trigger.Response()
        try:
            n = self.save_yaml()
            resp.success = True
            resp.message = f"Saved {n} waypoints → {os.path.abspath(self.output_yaml)}"
            self.get_logger().info(resp.message)
        except Exception as e:
            resp.success = False
            resp.message = f"Save failed: {e}"
            self.get_logger().error(resp.message)
        return resp

    # ---------- helpers ----------
    def current_pose(self) -> Optional[Waypoint]:
        # Try TF first
        if self.use_tf_first:
            try:
                tf = self.tfbuf.lookup_transform(self.world_frame, self.base_frame, Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return Waypoint(float(t.x), float(t.y), yaw_from_quat(q))
            except TransformException:
                pass

        # Fallback to odometry topic if available and in same frame
        if self.last_odom and (self.last_odom.header.frame_id in (self.world_frame, "", None)):
            p = self.last_odom.pose.pose
            return Waypoint(float(p.position.x), float(p.position.y), yaw_from_quat(p.orientation))

        return None

    def _add_point(self, p: Waypoint, force: bool):
        if not self.path:
            self.path.append(p)
            self.last_pose = p
            self.get_logger().info(
                f"Start waypoint @ ({p.x:.2f}, {p.y:.2f}) yaw={math.degrees(p.yaw):.1f}°"
            )
            return

        dx = p.x - self.last_pose.x
        dy = p.y - self.last_pose.y
        dist = math.hypot(dx, dy)
        dhead = abs((p.yaw - self.last_pose.yaw + math.pi) % (2 * math.pi) - math.pi)

        if force or dist >= self.min_dist or dhead >= self.min_head:
            self.path.append(p)
            self.last_pose = p

    def _dedupe(self, pts: List[Waypoint]) -> List[Waypoint]:
        if not pts:
            return pts
        deduped: List[Waypoint] = []
        last: Optional[Waypoint] = None
        for w in pts:
            if last is None or math.hypot(w.x - last.x, w.y - last.y) > self.dedupe_radius:
                deduped.append(w)
                last = w
        return deduped

    def save_yaml(self) -> int:
        pts = self.path.copy()
        if self.close_loop and len(pts) >= 2:
            pts.append(pts[0])

        # De-dup near-identical adjacency (button spam / jitter)
        pts = self._dedupe(pts)

        data = {"waypoints": []}
        for w in pts:
            q = quat_from_yaw(w.yaw)
            data["waypoints"].append(
                {
                    "header": {"frame_id": str(self.world_frame)},
                    "pose": {
                        "position": {"x": float(w.x), "y": float(w.y), "z": 0.0},
                        "orientation": {
                            "x": float(q.x),
                            "y": float(q.y),
                            "z": float(q.z),
                            "w": float(q.w),
                        },
                    },
                }
            )

        os.makedirs(os.path.dirname(self.output_yaml) or ".", exist_ok=True)
        with open(self.output_yaml, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False)
        return len(pts)


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save on shutdown if enough points; guard rcl shutdown order
        try:
            if len(node.path) >= 2:
                n = node.save_yaml()
                node.get_logger().info(f"Saved {n} waypoints to {os.path.abspath(node.output_yaml)}")
            else:
                node.get_logger().warn("Not enough points to save; need at least 2.")
        except Exception as e:
            # Avoid logging if context is already down
            if rclpy.ok():
                node.get_logger().error(f"Save on shutdown failed: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
