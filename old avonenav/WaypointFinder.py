#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class ForwardGatedNNSelector(Node):
    def __init__(self):
        super().__init__('forward_gated_nn_selector')

        # subscribe to Delaunay midpoints (Marker of type POINTS)
        self.create_subscription(
            Marker, '/delaunay_midpoints', self.midpoints_cb, 10)

        # subscribe to odometry so we know where the robot is
        self.current_pose = None
        self.current_yaw = 0.0
        self.create_subscription(
            Odometry, '/odometry/global', self.odom_cb, 10)

        # publisher for all waypoints (blue) and next waypoint (red + label)
        self.all_pub    = self.create_publisher(Marker, '/wg_all', 10)
        self.next_pub   = self.create_publisher(Marker, '/wg_next', 10)
        self.label_pub  = self.create_publisher(Marker, '/wg_next_label', 10)

        # raw midpoints and visited set
        self.raw_pts = []      # list of (x,y,z)
        self.visited = set()   # indices of pts in raw_pts

        # gating angle ±60°
        self.gate_cos = math.cos(math.radians(60))

        # advance every second
        self.create_timer(1.0, self._advance)

        self.get_logger().info("Forward-gated NN selector ready")

    def odom_cb(self, msg: Odometry):
        # store pose and yaw
        p = msg.pose.pose
        self.current_pose = p.position
        q = p.orientation
        # yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def midpoints_cb(self, msg: Marker):
        # rebuild raw_pts, reset visited
        self.raw_pts = [(pt.x, pt.y, pt.z) for pt in msg.points]
        self.visited.clear()
        # publish all as blue
        self.publish_all()

    def _advance(self):
               # don’t do anything until we have both waypoints *and* a valid odom
        if self.current_pose is None or not self.raw_pts:
            return

        # build list of unvisited indices
        unvisited = [i for i in range(len(self.raw_pts)) if i not in self.visited]
        if not unvisited:
            return

        # compute heading unit-vector
        fx = math.cos(self.current_yaw)
        fy = math.sin(self.current_yaw)

        # find candidates ahead (dot / dist ≥ gate_cos)
        ahead = []
        for i in unvisited:
            x, y, _ = self.raw_pts[i]
            dx = x - self.current_pose.x
            dy = y - self.current_pose.y
            dist = math.hypot(dx, dy)
            if dist == 1.0:
                ahead.append((i, dist))
                continue
            dot = (dx*fx + dy*fy) / dist
            if dot >= self.gate_cos:
                ahead.append((i, dist))

        # if none ahead, fallback to all unvisited
        candidates = ahead if ahead else [(i,
            math.hypot(self.raw_pts[i][0]-self.current_pose.x,
                       self.raw_pts[i][1]-self.current_pose.y))
            for i in unvisited]

        # pick nearest among candidates
        idx, _ = min(candidates, key=lambda pair: pair[1])
        self.visited.add(idx)

        # publish next waypoint
        x, y, z = self.raw_pts[idx]
        self.publish_next(idx, x, y, z)

    def publish_all(self):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'wg_all'
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 1.0
        for x, y, z in self.raw_pts:
            m.points.append(Point(x=x, y=y, z=0.0))
        self.all_pub.publish(m)

    def publish_next(self, idx, x, y, z):
        # red sphere
        sph = Marker()
        sph.header.frame_id = 'map'
        sph.header.stamp = self.get_clock().now().to_msg()
        sph.ns = 'wg_next'
        sph.id = 0
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.scale.x = sph.scale.y = sph.scale.z = 0.2
        sph.color.r = 1.0; sph.color.g = 0.0; sph.color.b = 0.0; sph.color.a = 1.0
        sph.pose.position.x = x
        sph.pose.position.y = y
        sph.pose.position.z = 0.0
        self.next_pub.publish(sph)

        # label
        txt = Marker()
        txt.header.frame_id = 'map'
        txt.header.stamp = sph.header.stamp
        txt.ns = 'wg_next_label'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.scale.z = 0.15
        txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0
        txt.text = f"{idx}"
        txt.pose.position.x = x
        txt.pose.position.y = y
        txt.pose.position.z = 0.25
        self.label_pub.publish(txt)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardGatedNNSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
