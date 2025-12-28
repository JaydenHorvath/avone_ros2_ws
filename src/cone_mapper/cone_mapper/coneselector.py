#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

import csv
import os


class ClickConeMapper(Node):
    def __init__(self):
        super().__init__("click_cone_mapper")

        self.frame_id = "odom"
        self.current_color = "blue"

        # Store cones as list of dicts: {x, y, color}
        self.cones = []

        # RViz click subscriber
        self.sub_click = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

        # Color toggle subscriber ("blue" or "orange")
        self.sub_color = self.create_subscription(
            String,
            "/cone_color_toggle",
            self.color_toggle_callback,
            10
        )

        # Marker publisher
        self.pub_markers = self.create_publisher(
            MarkerArray,
            "/cone_markers",
            10
        )

        # Service to save CSV manually
        self.srv_save = self.create_service(
            Trigger,
            "/save_cone_map",
            self.save_service_callback
        )

        self.get_logger().info("CLICK CONE MAPPER READY")
        self.get_logger().info("Use RViz Publish Point tool to place cones")
        self.get_logger().info("Publish 'blue' or 'orange' to /cone_color_toggle")

    def color_toggle_callback(self, msg: String):
        if msg.data in ["blue", "orange"]:
            self.current_color = msg.data
            self.get_logger().info(f"Switched cone color to: {self.current_color}")
        else:
            self.get_logger().warn("Invalid color. Use 'blue' or 'orange'.")

    def clicked_point_callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y

        cone = {
            "x": x,
            "y": y,
            "color": self.current_color
        }
        self.cones.append(cone)

        self.get_logger().info(
            f"Added cone at ({x:.2f}, {y:.2f}) color={self.current_color}"
        )

        self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, c in enumerate(self.cones):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            m.scale.x = 0.25
            m.scale.y = 0.25
            m.scale.z = 0.5

            m.pose.position.x = c["x"]
            m.pose.position.y = c["y"]
            m.pose.position.z = 0.0

            # Colors (RGBA)
            if c["color"] == "blue":
                m.color.r = 0.0
                m.color.g = 0.3
                m.color.b = 1.0
                m.color.a = 1.0
            else:
                m.color.r = 1.0
                m.color.g = 0.5
                m.color.b = 0.0
                m.color.a = 1.0

            arr.markers.append(m)

        self.pub_markers.publish(arr)

    def save_to_csv(self, filepath=None):
        if filepath is None:
            filepath = os.path.expanduser("~/cones.csv")

        with open(filepath, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y", "color"])
            for c in self.cones:
                writer.writerow([c["x"], c["y"], c["color"]])

        self.get_logger().info(f"Saved CSV: {filepath}")
        return filepath

    def save_service_callback(self, request, response):
        path = self.save_to_csv()
        response.success = True
        response.message = f"Saved to {path}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ClickConeMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Saving cones before shutdown...")
        node.save_to_csv()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
