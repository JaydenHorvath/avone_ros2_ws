#!/usr/bin/env python3
# Ground + radius + angular-sector removal with adaptive z_min(r) and per-ring bias.
# Angle filter can follow the vehicle's forward (+X in base frame) using TF so it
# works regardless of LiDAR mounting orientation.

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

def _coerce_list(val, dtype=float):
    if isinstance(val, (list, tuple, np.ndarray)):
        return np.array(val, dtype=dtype)
    if isinstance(val, str):
        s = val.strip()
        if s.startswith('[') and s.endswith(']'):
            s = s[1:-1]
        parts = [p for p in s.replace(',', ' ').split() if p]
        try:
            return np.array([dtype(p) for p in parts], dtype=dtype)
        except Exception:
            pass
    return np.array([], dtype=dtype)

def _quat_to_rot(qx, qy, qz, qw):
    x, y, z, w = qx, qy, qz, qw
    xx = x*x; yy = y*y; zz = z*z
    xy = x*y; xz = x*z; yz = y*z
    wx = w*x; wy = w*y; wz = w*z
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=np.float64)

class GroundRemover(Node):
    def __init__(self):
        super().__init__('ground_remover')

        # I/O
        self.declare_parameter('input_topic', '/quanergy/points')
        self.declare_parameter('output_topic', '/points_no_ground')

        # Base filters
        self.declare_parameter('z_min', -0.4)         # base z threshold at r=0 (m)
        self.declare_parameter('r_min', 0.0)           # inner radius (m)
        self.declare_parameter('r_max', 10.0)          # outer radius (m)
        self.declare_parameter('approx_layout', True)  # x/y/z at offsets 0/4/8

        # Adaptive slope & ring bias
        self.declare_parameter('k_lin', 0.02)         # extra z per meter (m/m)
        self.declare_parameter('use_ring_bias', True)
        self.declare_parameter('ring_bias', [0.00,0.00,0.01,0.01,0.02,0.02,0.03,0.03])

        # Angular sector filter
        # angle_mode: 'manual' (use angle_center_deg) or 'base' (use base_frame +X via TF)
        self.declare_parameter('angle_filter_enabled', True)
        self.declare_parameter('angle_mode', 'base')          # 'base' is robust to mounting
        self.declare_parameter('base_frame', 'quanergy')
        self.declare_parameter('angle_center_deg', 0.0)     # used when angle_mode='manual'
        self.declare_parameter('angle_half_width_deg', 90.0)  # widen to catch legs while pulling
        self.declare_parameter('angle_keep', False)           # False = remove that sector

        # TF setup (for angle_mode='base')
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        in_topic  = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.sub = self.create_subscription(PointCloud2, in_topic, self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)
        self.get_logger().info(f"Ground+range+angle on {in_topic} → {out_topic} | angle_mode={self.get_parameter('angle_mode').value}")

        # Debug counters
        self._dbg_total = 0
        self._dbg_removed_sector = 0

    def cb(self, msg: PointCloud2):
        try:
            n = msg.width * msg.height
            step = msg.point_step
            if n == 0 or step == 0:
                return

            buf = np.frombuffer(msg.data, dtype=np.uint8, count=n*step).reshape((-1, step))

            approx = bool(self.get_parameter('approx_layout').value)
            if approx and self._xyz_layout_ok(msg):
                x = buf[:, 0:4].view('<f4').reshape(-1)
                y = buf[:, 4:8].view('<f4').reshape(-1)
                z = buf[:, 8:12].view('<f4').reshape(-1)
                ring = self._read_ring(msg, buf)
            else:
                x, y, z, ring = self._extract_xyz_ring(msg, buf)

            # Sanitize NaNs
            good = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            if ring is not None:
                good &= np.isfinite(ring)
            if not np.all(good):
                x, y, z = x[good], y[good], z[good]
                buf = buf[good]
                if ring is not None:
                    ring = ring[good]

            r = np.sqrt(x*x + y*y)

            # Range window
            r_min = float(self.get_parameter('r_min').value)
            r_max = float(self.get_parameter('r_max').value)
            keep = (r >= r_min) & (r <= r_max)

            # Angular sector (REMOVE by default)
            if bool(self.get_parameter('angle_filter_enabled').value):
                theta = np.arctan2(y, x)  # [-pi, pi], 0 = +X of LiDAR frame
                half  = np.deg2rad(float(self.get_parameter('angle_half_width_deg').value))
                mode  = self.get_parameter('angle_mode').value
                if mode == 'base':
                    base_frame = self.get_parameter('base_frame').value
                    lidar_frame = msg.header.frame_id
                    stamp = msg.header.stamp
                    # Invert the forward direction
                    center = self._forward_angle_base_in_lidar(lidar_frame, base_frame, stamp, fallback_deg=0.0) + np.pi
                else:
                    # Manual mode — invert direction
                    center = np.deg2rad(float(self.get_parameter('angle_center_deg').value) + 180.0)

                # Signed difference, wrapped to [-pi,pi]
                d = np.arctan2(np.sin(theta - center), np.cos(theta - center))
                in_sector = np.abs(d) <= half

                if bool(self.get_parameter('angle_keep').value):
                    # keep only the sector
                    keep &= in_sector
                else:
                    # remove the sector
                    removed_now = np.count_nonzero(keep & in_sector)
                    self._dbg_removed_sector += int(removed_now)
                    keep &= (~in_sector)

            # Adaptive z threshold
            z0    = float(self.get_parameter('z_min').value)
            k_lin = float(self.get_parameter('k_lin').value)
            z_thr = z0 + k_lin * r

            if bool(self.get_parameter('use_ring_bias').value) and ring is not None:
                rb_param = self.get_parameter('ring_bias').value
                rb = _coerce_list(rb_param, float)
                if rb.size == 0:
                    rb = np.array([0.0], dtype=np.float32)
                max_idx = min(rb.shape[0]-1, int(np.nanmax(ring)) if ring.size else 0)
                ring_idx = np.clip(ring.astype(np.int32), 0, max_idx)
                z_thr = z_thr + rb[ring_idx]

            keep &= (z >= z_thr)

            self._dbg_total += keep.size
            if (self._dbg_total % 100000) < keep.size:  # occasionally log
                self.get_logger().info(
                    f"Angle removed total ~{self._dbg_removed_sector} of {self._dbg_total} points "
                    f"({100.0*self._dbg_removed_sector/max(1,self._dbg_total):.1f}%)"
                )

            kept = buf[keep]
            out = self._build_cloud(msg, kept)
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().warn(f"Filtering failed, passing-through: {e}")
            self.pub.publish(msg)

    # ---------- helpers ----------
    def _forward_angle_base_in_lidar(self, lidar_frame: str, base_frame: str, stamp, fallback_deg: float) -> float:
        """
        Returns the angle (rad) of base_frame +X axis expressed in lidar_frame XY plane.
        Falls back to fallback_deg if TF not available.
        """
        try:
            # TF: lidar_frame <- base_frame (pose of base in lidar)
            can = self.tf_buffer.can_transform(lidar_frame, base_frame, stamp, timeout=Duration(seconds=0.2))
        except Exception:
            can = False
        try:
            if can:
                t: TransformStamped = self.tf_buffer.lookup_transform(lidar_frame, base_frame, stamp, timeout=Duration(seconds=0.2))
            else:
                # try latest
                t = self.tf_buffer.lookup_transform(lidar_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
                self.get_logger().warn(f"Angle: TF at stamp not ready; using latest {base_frame}->{lidar_frame}")
            R = _quat_to_rot(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)
            ex_base = np.array([1.0, 0.0, 0.0], dtype=np.float64)  # +X in base frame
            ex_lidar = R.dot(ex_base)  # expressed in lidar frame
            ang = np.arctan2(ex_lidar[1], ex_lidar[0])  # [-pi,pi]
            return float(ang)
        except Exception as e:
            self.get_logger().warn(f"Angle: TF lookup failed ({e}); using fallback {fallback_deg} deg")
            return np.deg2rad(fallback_deg)

    @staticmethod
    def _xyz_layout_ok(msg: PointCloud2) -> bool:
        try:
            f = {fld.name: fld for fld in msg.fields}
            return (f['x'].datatype == PointField.FLOAT32 and f['x'].offset == 0 and
                    f['y'].datatype == PointField.FLOAT32 and f['y'].offset == 4 and
                    f['z'].datatype == PointField.FLOAT32 and f['z'].offset == 8)
        except KeyError:
            return False

    @staticmethod
    def _read_ring(msg: PointCloud2, buf: np.ndarray):
        f = {fld.name: fld for fld in msg.fields}
        rf = f.get('ring', None)
        if rf is None:
            return None
        try:
            if rf.datatype == PointField.UINT16 and rf.offset <= msg.point_step - 2:
                return buf[:, rf.offset:rf.offset+2].view('<u2').reshape(-1)
            if rf.datatype == PointField.INT32  and rf.offset <= msg.point_step - 4:
                return buf[:, rf.offset:rf.offset+4].view('<i4').reshape(-1).astype(np.int32)
        except Exception:
            return None
        return None

    @staticmethod
    def _extract_xyz_ring(msg: PointCloud2, buf: np.ndarray):
        f = {fld.name: fld for fld in msg.fields}
        for k in ('x', 'y', 'z'):
            if k not in f or f[k].datatype != PointField.FLOAT32:
                raise RuntimeError("Expected float32 x,y,z fields")
        x = buf[:, f['x'].offset:f['x'].offset+4].view('<f4').reshape(-1)
        y = buf[:, f['y'].offset:f['y'].offset+4].view('<f4').reshape(-1)
        z = buf[:, f['z'].offset:f['z'].offset+4].view('<f4').reshape(-1)

        ring = None
        if 'ring' in f:
            rf = f['ring']
            if rf.datatype == PointField.UINT16:
                ring = buf[:, rf.offset:rf.offset+2].view('<u2').reshape(-1)
            elif rf.datatype == PointField.INT32:
                ring = buf[:, rf.offset:rf.offset+4].view('<i4').reshape(-1).astype(np.int32)
        return x, y, z, ring

    @staticmethod
    def _build_cloud(src: PointCloud2, kept_u8_2d: np.ndarray) -> PointCloud2:
        out = PointCloud2()
        out.header = src.header
        out.height = 1
        out.width = kept_u8_2d.shape[0]
        out.fields = src.fields
        out.is_bigendian = src.is_bigendian
        out.point_step = src.point_step
        out.row_step = out.point_step * out.width
        out.is_dense = False
        out.data = kept_u8_2d.reshape(-1).tobytes()
        return out

def main():
    rclpy.init()
    node = GroundRemover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
