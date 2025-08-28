#!/usr/bin/env python3

import sys
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
import numpy as np
from scipy.spatial import Delaunay

class DelaunayPlanner(Node):
    def __init__(self):
        super().__init__('delaunay_planner')

        # storage for incoming cone positions
        self.blue_pts = []    # list of [x, y, z]
        self.yellow_pts = []  # list of [x, y, z]

        # subscribe to your blue/yellow cone topics
        self.create_subscription(
            MarkerArray, '/cone_landmarks_blue', self._blue_cb, 10)
        self.create_subscription(
            MarkerArray, '/cone_landmarks_yellow', self._yellow_cb, 10)

        # publishers for triangles, midpoints, and path
        self.tri_pub  = self.create_publisher(Marker, '/delaunay_triangles', 10)
        self.mid_pub  = self.create_publisher(Marker, '/delaunay_midpoints', 10)
        self.path_pub = self.create_publisher(Path,   '/delaunay_path',      10)

        # timer to recompute at 5 Hz
        self.create_timer(0.2, self._timer_cb)
        self.get_logger().info('DelaunayPlanner ready')

    def _blue_cb(self, msg: MarkerArray):
        self.blue_pts = [
            [m.pose.position.x, m.pose.position.y, m.pose.position.z]
            for m in msg.markers
        ]

    def _yellow_cb(self, msg: MarkerArray):
        self.yellow_pts = [
            [m.pose.position.x, m.pose.position.y, m.pose.position.z]
            for m in msg.markers
        ]

    def _timer_cb(self):
        # merge and check
        n_blue  = len(self.blue_pts)
        n_total = n_blue + len(self.yellow_pts)
        if n_total < 3:
            return  # not enough to triangulate

        pts3d = np.vstack((self.blue_pts, self.yellow_pts))
        pts2d = pts3d[:, :2]
        tri   = Delaunay(pts2d)

        # keep only mixed-color triangles
        mixed = []
        for s in tri.simplices:
            cols = [0 if idx < n_blue else 1 for idx in s]
            if len(set(cols)) > 1:
                mixed.append(s)

        # build unique edges, with max 4 per cone
        seen_edges = set()
        edge_degs  = {i: 0 for i in range(n_total)}
        draw_edges = []
        
        MAX_EDGE_LENGTH = 5.0  # adjust as needed (in meters)

        for s in mixed:
            for i, j in [(0,1),(1,2),(2,0)]:
                e = tuple(sorted((s[i], s[j])))
                if e not in seen_edges:
                    p1 = pts3d[e[0]]
                    p2 = pts3d[e[1]]
                    dist = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
                    if dist <= MAX_EDGE_LENGTH:
                        if edge_degs[e[0]] < 4 and edge_degs[e[1]] < 4:
                            draw_edges.append(e)
                            edge_degs[e[0]] += 1
                            edge_degs[e[1]] += 1
                    seen_edges.add(e)

        # count interior edges (shared by >1 mixed triangle)
        edge_count = {}
        for s in mixed:
            for i, j in [(0,1),(1,2),(2,0)]:
                e = tuple(sorted((s[i], s[j])))
                edge_count[e] = edge_count.get(e, 0) + 1
        internal_edges = [e for e, c in edge_count.items() if c > 1]

        # --- publish triangle edges ---
        tri_m = Marker()
        tri_m.header.frame_id = 'map'
        tri_m.header.stamp    = self.get_clock().now().to_msg()
        tri_m.ns    = 'delaunay'
        tri_m.id    = 0
        tri_m.type  = Marker.LINE_LIST
        tri_m.action = Marker.ADD
        tri_m.scale.x = 0.02
        tri_m.color.r = 0.2; tri_m.color.g = 0.6; tri_m.color.b = 0.8; tri_m.color.a = 1.0
        for i, j in draw_edges:
            p1 = pts3d[i]; p2 = pts3d[j]
            tri_m.points.append(Point(x=p1[0], y=p1[1], z=0.0))
            tri_m.points.append(Point(x=p2[0], y=p2[1], z=0.0))
        self.tri_pub.publish(tri_m)

        # --- publish midpoints of internal edges only ---
        mid_m = Marker()
        mid_m.header.frame_id = 'map'
        mid_m.header.stamp    = tri_m.header.stamp
        mid_m.ns    = 'delaunay'
        mid_m.id    = 1
        mid_m.type  = Marker.POINTS
        mid_m.action= Marker.ADD
        mid_m.scale.x = 0.1; mid_m.scale.y = 0.1
        mid_m.color.r = 0.0; mid_m.color.g = 1.0; mid_m.color.b = 0.0; mid_m.color.a = 1.0

        midpoints = []
        for i, j in internal_edges:
            mid = 0.5 * (pts3d[i] + pts3d[j])
            mid_m.points.append(Point(x=mid[0], y=mid[1], z=0.0))
            midpoints.append((mid[0], mid[1]))
        self.mid_pub.publish(mid_m)

        # --- build & publish Path through midpoints ---
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = tri_m.header.stamp

        if midpoints:
            pts = midpoints.copy()
            ordered = [pts.pop(0)]
            while pts:
                curr = ordered[-1]
                dists = [math.hypot(p[0]-curr[0], p[1]-curr[1]) for p in pts]
                idx = int(np.argmin(dists))
                ordered.append(pts.pop(idx))

            for x, y in ordered:
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.header.stamp    = path.header.stamp
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = DelaunayPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
