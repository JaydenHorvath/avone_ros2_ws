#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import Int32

class SpeedRepublisher(Node):
    def __init__(self):
        super().__init__('kelly_speed_republisher')
        # 1) listen to the CAN frames on /from_can_bus instead of received_messages
        self.sub = self.create_subscription(
            Frame,
            'from_can_bus',
            self.on_can_frame,
            50)
        # 2) republish RPM on /motor_speed
        self.pub = self.create_publisher(Int32, 'motor_speed', 10)

    def on_can_frame(self, frame: Frame):
        # only process the 0x0CF11E05 status frame
        if frame.is_extended and frame.id == 0x0CF11E05:
            # pull bytes 0–1 as a big-endian 16-bit value
            raw = (frame.data[0] << 8) | frame.data[1]
            rpm = int(raw)            # <<< make absolutely sure it’s a Python int
            msg = Int32()
            msg.data = rpm
            self.pub.publish(msg)
            self.get_logger().debug(f"Published RPM: {rpm}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeedRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
