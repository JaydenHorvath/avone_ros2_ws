#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from visualization_msgs.msg import (
    MarkerArray, Marker,
    InteractiveMarker, InteractiveMarkerControl
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from geometry_msgs.msg import PointStamped, Pose, Quaternion
from std_msgs.msg import Bool


class ConeSelector(Node):
    def __init__(self):
        super().__init__('cone_selector')

        # ---------------- Params ----------------
        # Topic where RViz "Publish Point" tool publishes
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.clicked_point_topic = self.get_parameter('clicked_point_topic').value

        # Default sizes/colors for newly added cones
        self.declare_parameter('new_cone_radius', 0.20)   # cylinder radius ~ cone footprint
        self.declare_parameter('new_cone_height', 0.50)
        self.declare_parameter('new_cone_color_rgba', [1.0, 0.5, 0.0, 0.9])  # orange-ish

        self.new_cone_radius = float(self.get_parameter('new_cone_radius').value)
        self.new_cone_height = float(self.get_parameter('new_cone_height').value)
        rgba = self.get_parameter('new_cone_color_rgba').value
        self.new_cone_rgba = (
            float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3])
        )

        # ---------------- Interactive marker server ----------------
        self.server = InteractiveMarkerServer(self, "cone_selector")

        # ---------------- Subscriptions ----------------
        # Subscribe to auto-detected cones to mirror them as interactive markers
        self.sub = self.create_subscription(
            MarkerArray,
            '/cones/tracks/markers',
            self.cone_callback,
            10
        )

        # Subscribe to RViz "Publish Point" tool output
        self.click_sub = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.clicked_point_cb,
            10
        )

        # Optional toggle to simulate "Shift" (enable/disable add mode)
        self.shift_sub = self.create_subscription(
            Bool,
            '/cone_selector/add_mode',
            self.shift_mode_callback,
            10
        )
        self.shift_mode = False

        # ---------------- Publisher ----------------
        self.pub = self.create_publisher(MarkerArray, '/cones/tracks/markers_filtered', 10)

        # ---------------- State ----------------
        self.cones = {}               # id -> Marker
        self.next_cone_id = 10000     # manual cones start here (avoid collision with detected IDs)

        self.get_logger().info(
            "Cone selector ready — Left-click cones to remove.\n"
            f"To ADD: select RViz 'Publish Point' tool and click (while /cone_selector/add_mode is True).\n"
            f"Listening to clicked points on: {self.clicked_point_topic}"
        )

    # ---------- Callbacks ----------
    def shift_mode_callback(self, msg: Bool):
        self.shift_mode = msg.data
        self.get_logger().info(f"Add mode {'ENABLED' if msg.data else 'DISABLED'}")

    def cone_callback(self, msg: MarkerArray):
        """Mirror incoming detected cones as interactive markers (if not already present)."""
        for marker in msg.markers:
            if marker.id not in self.cones:
                self._make_interactive_marker(marker)
        self._publish_filtered()

    def clicked_point_cb(self, msg: PointStamped):
        """Handle RViz Publish Point clicks to add cones (only if add_mode is True)."""
        if not self.shift_mode:
            # Acting like "Shift not held": ignore add when toggle is off
            self.get_logger().debug("Clicked point received but add mode is disabled.")
            return

        p = msg.point
        frame = msg.header.frame_id if msg.header.frame_id else "map"

        new_id = self.next_cone_id
        self.next_cone_id += 1

        self.get_logger().info(
            f"Adding cone from Publish Point at ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) in frame '{frame}' "
            f"with ID {new_id}"
        )

        marker = self._build_cone_marker(new_id, frame, p.x, p.y, p.z)
        self._make_interactive_marker(marker)
        self._publish_filtered()

    # ---------- Helpers ----------
    def _build_cone_marker(self, marker_id: int, frame_id: str, x: float, y: float, z: float) -> Marker:
        """Create a cylinder marker representing a cone at the given position."""
        r, g, b, a = self.new_cone_rgba

        m = Marker()
        m.header.frame_id = frame_id
        m.id = marker_id
        m.type = Marker.CYLINDER
        m.action = Marker.ADD

        m.pose = Pose()
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        # Place cylinder so it rests on ground if your clicked z is ground height.
        # If your clicks return ground Z already, leave as is; otherwise adjust (e.g., +height/2).
        m.pose.position.z = float(z + self.new_cone_height * 0.5)
        m.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Cylinder scales: x,y are diameters
        m.scale.x = float(self.new_cone_radius * 2.0)
        m.scale.y = float(self.new_cone_radius * 2.0)
        m.scale.z = float(self.new_cone_height)

        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = a

        return m

    def _make_interactive_marker(self, marker: Marker):
        """Wrap a regular Marker into an InteractiveMarker so clicks remove it."""
        int_marker = InteractiveMarker()
        int_marker.header = marker.header
        int_marker.name = str(marker.id)
        int_marker.description = f"Cone {marker.id}"
        int_marker.pose = marker.pose

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(marker)
        int_marker.controls.append(control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self._process_feedback)

        self.cones[marker.id] = marker
        self.server.applyChanges()

    def _process_feedback(self, feedback):
        """Click on existing interactive marker -> remove it (regardless of add mode)."""
        marker_name = feedback.marker_name.strip()
        if marker_name.isdigit():
            mid = int(marker_name)
            if mid in self.cones:
                self.get_logger().info(f"Removed cone ID {mid}")
                del self.cones[mid]
                self.server.erase(str(mid))
                self.server.applyChanges()
                self._publish_filtered()

    def _publish_filtered(self):
        msg = MarkerArray()
        msg.markers = list(self.cones.values())
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
