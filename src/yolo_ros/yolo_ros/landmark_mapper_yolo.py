#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np

DIST_THRESHOLD = 2.0   # meters to merge duplicates
MAX_RANGE     = 10.0  # maximum distance (m) to accept any cone

def pose_to_numpy(pose: Pose):
    return (
        np.array([pose.position.x,
                  pose.position.y,
                  pose.position.z], dtype=np.float64),
        np.array([pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w], dtype=np.float64),
    )

def quaternion_matrix(q):
    x,y,z,w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-8:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xs,ys,zs = x*s, y*s, z*s
    wx,wy,wz = w*xs, w*ys, w*zs
    xx,xy,xz = x*xs, x*ys, x*zs
    yy,yz,zz = y*ys, y*zs, z*zs
    return np.array([
        [1-(yy+zz),   xy-wz,     xz+wy],
        [xy+wz,       1-(xx+zz), yz-wx],
        [xz-wy,       yz+wx,     1-(xx+yy)]
    ], dtype=np.float64)

def quaternion_multiply(a, b):
    x1,y1,z1,w1 = a; x2,y2,z2,w2 = b
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ], dtype=np.float64)

def transform_pose(pose: Pose, tf) -> Pose:
    pos, quat = pose_to_numpy(pose)
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z], dtype=np.float64)
    q = np.array([tf.transform.rotation.x,
                  tf.transform.rotation.y,
                  tf.transform.rotation.z,
                  tf.transform.rotation.w], dtype=np.float64)

    R = quaternion_matrix(q)
    new_pos = t + R.dot(pos)
    new_q   = quaternion_multiply(q, quat)
    new_q  /= np.linalg.norm(new_q)

    out = Pose()
    out.position.x, out.position.y, out.position.z = map(float, new_pos)
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = map(float, new_q)
    return out


class ConeLandmarksPublisher(Node):
    def __init__(self):
        super().__init__('cone_landmarks_publisher')
        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        # Each entry: {'sum':np.array, 'count':int, 'quat':np.array, 'label':str}
        self.known_cones = []

        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detections_cb,
            10)
        self.marker_pub = self.create_publisher(MarkerArray, '/cone_landmarks', 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(f'Publishing cones in "{self.target_frame}" frame')

    def detections_cb(self, msg: Detection2DArray):
        now = self.get_clock().now()
        updated = False

        for det in msg.detections:
            src = det.results[0].pose.pose

            # Try TF once per detection; skip only that one if unavailable
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.1))
            except Exception:
                self.get_logger().warn('TF lookup failed, skipping this detection')
                continue

            # Transform into world frame
            try:
                world_pose = transform_pose(src, tf)
            except Exception as e:
                self.get_logger().warn(f'Transform error: {e}')
                continue

            pos, quat = pose_to_numpy(world_pose)
            dist = np.linalg.norm(pos)

            # **Filter out anything beyond max range**
            if dist > MAX_RANGE:
                continue

            lbl = det.results[0].hypothesis.class_id

            # Merge into existing if within DIST_THRESHOLD, else add new
            for cone in self.known_cones:
                avg = cone['sum'] / cone['count']
                if np.linalg.norm(pos - avg) < DIST_THRESHOLD:
                    cone['sum']   += pos
                    cone['count'] += 1
                    cone['quat']   = quat
                    cone['label']  = lbl
                    break
            else:
                self.known_cones.append({
                    'sum'  : pos.copy(),
                    'count': 1,
                    'quat' : quat,
                    'label': lbl
                })

            updated = True

        # Always publish (even if updated==False, you'll get an empty array initially)
        self.publish_markers(now)

    def publish_markers(self, now):
        ma = MarkerArray()
        for i, cone in enumerate(self.known_cones):
            avg = cone['sum'] / cone['count']
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp    = now.to_msg()
            m.ns              = 'cones'
            m.id              = i
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD

            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, avg)
            qx, qy, qz, qw = cone['quat']
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
                float(qx), float(qy), float(qz), float(qw)
            )

            m.scale.x = m.scale.y = 0.2
            m.scale.z = 0.5

            lbl = str(cone['label'])
            if lbl in ('0', 'blue'):
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.0, 1.0, 0.8
            elif lbl in ('4', 'yellow'):
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.8
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.8

            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = ConeLandmarksPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
