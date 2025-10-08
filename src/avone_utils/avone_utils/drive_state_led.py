import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32

class LedMatrixNode(Node):
    def init(self):
        super().init('led_matrix_node')

        # Serial setup
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Last known drive state
        self.latest_state = None

        # Subscribe to the drive state topic
        self.sub = self.create_subscription(
            Int32,
            '/av1/avone_state/drive_state',
            self.state_callback,
            10
        )

        # ---- Set frequency (Hz) ----
        freq_hz = 5.0   # change this value to your desired frequency
        self.timer = self.create_timer(1.0 / freq_hz, self.publish_state)

        self.get_logger().info(f"LED Matrix Node started at {freq_hz} Hz")

    def state_callback(self, msg: Int32):
        # Store the most recent state (don’t send yet)
        self.latest_state = msg.data

    def publish_state(self):
        if self.latest_state is not None:
            cmd = f"STATE {self.latest_state}\n"
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Sent DriveState={self.latest_state}")

def main(args=None):
    rclpy.init(args=args)
    node = LedMatrixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if name == 'main':
    main()