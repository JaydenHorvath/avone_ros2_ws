#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class ForwardNav2Waypoint(Node):
    def __init__(self):
        super().__init__('forward_nav2_waypoint')

        # --- Subscriptions ---
        self.create_subscription(
            Marker, '/delaunay_midpoints', self.midpoints_cb, 10)
        self.create_subscription(
            Odometry, '/odometry/global', self.odom_cb, 10)

        # --- Publishers (visualization only) ---
        self.all_pub    = self.create_publisher(Marker, '/wg_all', 10)
        self.next_pub   = self.create_publisher(Marker, '/wg_next', 10)
        self.label_pub  = self.create_publisher(Marker, '/wg_next_label', 10)
        self.next2_pub  = self.create_publisher(Marker, '/wg_next2', 10)
        self.label2_pub = self.create_publisher(Marker, '/wg_next2_label', 10)

        # --- Nav2 action client ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- State ---
        self.current_pose = None      # geometry_msgs/Point
        self.current_yaw  = 0.0
        self.raw_pts      = []        # list of (x,y,z)
        self.visited      = set()     # indices in raw_pts
        self.current_idx  = None      # index of the waypoint we’re en route to
        self.gate_cos     = math.cos(math.radians(60))

        self.get_logger().info("Forward-nav2-waypoint ready")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        self.current_pose = p.position
        q = p.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def midpoints_cb(self, msg: Marker):
        # update waypoints and viz
        self.raw_pts = [(pt.x, pt.y, pt.z) for pt in msg.points]
        self.visited.clear()
        self.publish_all()
        if self.current_idx is None:
            self.try_advance()

    def try_advance(self):
        # ensure we have odom and no active goal
        if self.current_pose is None or self.current_idx is not None:
            return
        unvisited = [i for i in range(len(self.raw_pts)) if i not in self.visited]
        if not unvisited:
            self.get_logger().info("All waypoints visited ✓")
            return

        # compute heading vector
        fx = math.cos(self.current_yaw)
        fy = math.sin(self.current_yaw)

        # filter ahead (using gate_cos)
        ahead = []
        for i in unvisited:
            x_i, y_i, _ = self.raw_pts[i]
            dx = x_i - self.current_pose.x
            dy = y_i - self.current_pose.y
            dist = math.hypot(dx, dy)
            if dist <= 0.5:
                ahead.append((i, dist))
                continue
            dot = (dx*fx + dy*fy) / dist
            if dot >= self.gate_cos:
                ahead.append((i, dist))
        candidates = ahead if ahead else [
            (i, math.hypot(self.raw_pts[i][0]-self.current_pose.x,
                           self.raw_pts[i][1]-self.current_pose.y))
            for i in unvisited
        ]

        # pick nearest ahead or overall as current
        idx, _ = min(candidates, key=lambda p: p[1])
        self.current_idx = idx
        x, y, _ = self.raw_pts[idx]

        # determine second lookahead waypoint (the "next of the next")
        second_candidates = [i for (i, _) in candidates if i != idx]
        next2_idx = None
        if second_candidates:
            distances = [(j, math.hypot(self.raw_pts[j][0] - x,
                                         self.raw_pts[j][1] - y))
                         for j in second_candidates]
            next2_idx, _ = min(distances, key=lambda p: p[1])
            x2, y2, _ = self.raw_pts[next2_idx]
            # heading for goal orientation
            desired_yaw = math.atan2(y2 - y, x2 - x)
        else:
            desired_yaw = math.atan2(y - self.current_pose.y,
                                     x - self.current_pose.x)

        # visualization of current and next2
        self.publish_next(idx, x, y, 0.0)
        if next2_idx is not None:
            self.publish_next2(next2_idx, x2, y2, 0.0)

        # send Nav2 goal with correct orientation
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        half = desired_yaw * 0.5
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        self._nav_client.wait_for_server()
        send_goal = self._nav_client.send_goal_async(goal)
        send_goal.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal was rejected")
            return
        self.get_logger().info("Nav2 goal accepted, waiting for result…")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached waypoint {self.current_idx} ✔")
            self.visited.add(self.current_idx)
            self.current_idx = None
            self.try_advance()
        else:
            self.get_logger().warn(f"Nav2 failed with status {status}, will not retry until waypoint is achieved")

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
        for x, y, _ in self.raw_pts:
            m.points.append(Point(x=x, y=y, z=0.0))
        self.all_pub.publish(m)

    def publish_next(self, idx, x, y, z):
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
        sph.pose.position.z = z
        self.next_pub.publish(sph)

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
        txt.pose.position.z = z + 0.25
        self.label_pub.publish(txt)

    def publish_next2(self, idx, x, y, z):
        sph = Marker()
        sph.header.frame_id = 'map'
        sph.header.stamp = self.get_clock().now().to_msg()
        sph.ns = 'wg_next2'
        sph.id = 0
        sph.type = Marker.SPHERE
        sph.action = Marker.ADD
        sph.scale.x = sph.scale.y = sph.scale.z = 0.2
        sph.color.r = 0.0; sph.color.g = 1.0; sph.color.b = 0.0; sph.color.a = 1.0
        sph.pose.position.x = x
        sph.pose.position.y = y
        sph.pose.position.z = z
        self.next2_pub.publish(sph)

        txt = Marker()
        txt.header.frame_id = 'map'
        txt.header.stamp = sph.header.stamp
        txt.ns = 'wg_next2_label'
        txt.id = 0
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.scale.z = 0.15
        txt.color.r = txt.color.g = txt.color.b = 1.0; txt.color.a = 1.0
        txt.text = f"{idx}"
        txt.pose.position.x = x
        txt.pose.position.y = y
        txt.pose.position.z = z + 0.25
        self.label2_pub.publish(txt)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardNav2Waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
