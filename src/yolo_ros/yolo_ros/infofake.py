#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class FakeCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_info_publisher')
        # Publisher for fake CameraInfo
        self.pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        # Timer at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_caminfo)

        # Fake camera parameters
        self.width = 640
        self.height = 480
        self.fx = 600.0
        self.fy = 600.0
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        self.get_logger().info('Fake CameraInfo publisher up at 30 Hz')

    def publish_caminfo(self):
        msg = CameraInfo()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'camera_link'
        msg.width = int(self.width)
        msg.height = int(self.height)
        # Intrinsic matrix
        msg.k = [self.fx, 0.0, self.cx,
                 0.0, self.fy, self.cy,
                 0.0, 0.0, 1.0]
        # No distortion
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Rectification
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        # Projection matrix
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraInfoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
