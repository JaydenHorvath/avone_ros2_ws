#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import serial

class DriveStateLED(Node):
    def __init__(self):
        super().__init__('drive_state_led')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # --- Serial init ---
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f'✅ Connected to ESP32 on {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open serial port: {e}')
            raise SystemExit

        # --- Subscriptions ---
        self.create_subscription(Int32, '/av1/avone_state/drive_state/', self.cb_drive_state, 10)
        self.create_subscription(Bool,  '/av1/fault_status/fault_latched', self.cb_fault_latched, 10)

        # --- Track last values to avoid spam ---
        self.last_state = None
        self.last_fault = None

    # --- Callbacks ---
    def cb_drive_state(self, msg: Int32):
        if msg.data != self.last_state:
            self.last_state = msg.data
            line = f'STATE {msg.data}\n'
            try:
                self.ser.write(line.encode())
                self.get_logger().info(f'Sent → {line.strip()}')
            except Exception as e:
                self.get_logger().warn(f'Write error: {e}')

    def cb_fault_latched(self, msg: Bool):
        if msg.data != self.last_fault:
            self.last_fault = msg.data
            line = f'FAULT {int(msg.data)}\n'
            try:
                self.ser.write(line.encode())
                self.get_logger().info(f'Sent → {line.strip()}')
            except Exception as e:
                self.get_logger().warn(f'Write error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DriveStateLED()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down drive_state_led node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
