import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32  # drive_state encoded as uint8

class LedMatrixNode(Node):
    def __init__(self):
        super().__init__('led_matrix_node')

        # open serial to ESP32-S3
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # subscribe to drive state CAN-decoded topic
        self.sub = self.create_subscription(
            Int32,
            '/av1/avone_state/drive_state',   # adjust to your topic name
            self.state_callback,
            10
        )

    def state_callback(self, msg: Int32):
        state = msg.data
        # just send the integer state over serial
        cmd = f"STATE {state}\n"
        self.get_logger().info(f"DriveState={state}")
        self.ser.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = LedMatrixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
