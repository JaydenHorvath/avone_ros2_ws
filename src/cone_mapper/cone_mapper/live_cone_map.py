#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np

class LiveConeMap(Node):
    def __init__(self):
        super().__init__('live_cone_map')

        # map parameters
        self.resolution = 0.05
        self.width  = 400
        self.height = 400
        self.origin_x = -10.0
        self.origin_y = -10.0

        # prepare the static parts of the OccupancyGrid
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.frame_id = 'map'
        md = MapMetaData()
        md.resolution = self.resolution
        md.width  = self.width
        md.height = self.height
        md.origin.position.x = self.origin_x
        md.origin.position.y = self.origin_y
        self.grid_msg.info = md

        # publisher with transient_local durability
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/live_map', qos)

        # subscribe to cone landmarks
        self.create_subscription(
            PoseArray, '/cone_landmarks',
            self.cb_landmarks, 10)

    def cb_landmarks(self, pa: PoseArray):
        # fresh free grid (0)
        data = np.zeros((self.height, self.width), dtype=np.int8)

        # mark cones as occupied (100)
        for p in pa.poses:
            i = int((p.position.x - self.origin_x) / self.resolution)
            j = int((p.position.y - self.origin_y) / self.resolution)
            if 0 <= i < self.width and 0 <= j < self.height:
                data[self.height - 1 - j, i] = 100

        # flatten and publish
        self.grid_msg.data = data.flatten().tolist()
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.grid_msg)

def main():
    rclpy.init()
    node = LiveConeMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
