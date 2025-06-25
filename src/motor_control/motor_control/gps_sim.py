#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import math, utm

class VS100Sim(Node):
    def __init__(self):
        super().__init__('vs100_sim')
        self.primary = None
        self.secondary = None

        self.sub1 = self.create_subscription(NavSatFix, '/navsat1', self.cb1, 10)
        self.sub2 = self.create_subscription(NavSatFix, '/navsat2', self.cb2, 10)

        # output exactly what the VS100 would:
        # 1) single NavSatFix on /navsat (from primary)
        # 2) heading (deg) on /heading
        self.pub_fix     = self.create_publisher(NavSatFix, '/navsat', 10)
        self.pub_heading = self.create_publisher(Float64,   '/heading', 10)

    def cb1(self, msg):
        self.primary = msg
        self.pub_fix.publish(msg)
        self.try_heading()

    def cb2(self, msg):
        self.secondary = msg
        self.try_heading()

    def try_heading(self):
        if not self.primary or not self.secondary:
            return

        e1, n1, _, _ = utm.from_latlon(self.primary.latitude,   self.primary.longitude)
        e2, n2, _, _ = utm.from_latlon(self.secondary.latitude, self.secondary.longitude)

        dx = e2 - e1
        dy = n2 - n1
        heading = (math.degrees(math.atan2(dx, dy)) + 360) % 360

        msg = Float64()
        msg.data = heading
        self.pub_heading.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VS100Sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
