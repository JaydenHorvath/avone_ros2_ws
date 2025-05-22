import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class TriggerToVelocity(Node):
    def __init__(self):
        super().__init__('trigger_to_velocity')
        self.pub = self.create_publisher(Float64, '/cmd_vel_float', 10)
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg):
        rt_axis = msg.axes[5]  # Xbox RT
        # Scale to positive velocity: (1.0 → 0.0), (-1.0 → 1.0)
        trigger_val = (1.0 - rt_axis) / 2.0
        velocity = trigger_val * 10.0  # Max speed = 10 rad/s
        self.pub.publish(Float64(data=velocity))

def main(args=None):
    rclpy.init(args=args)
    node = TriggerToVelocity()
    rclpy.spin(node)
    rclpy.shutdown()
