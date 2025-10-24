#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
import serial
import serial.tools.list_ports
import time


class ESP32LedController(Node):
    def __init__(self):
        super().__init__('esp32_led_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('auto_detect_port', False)
        self.declare_parameter('debug', True)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        auto_detect = self.get_parameter('auto_detect_port').value
        self.debug = self.get_parameter('debug').value
        
        # State tracking
        self.current_drive_state = -1
        self.fault_latched = False
        self.last_sent_state = None
        self.last_sent_fault = None
        
        # Initialize serial connection
        self.serial_conn = None
        if auto_detect:
            serial_port = self.find_esp32_port()
        
        if serial_port:
            try:
                self.serial_conn = serial.Serial(
                    serial_port, 
                    baud_rate, 
                    timeout=1,
                    write_timeout=1
                )
                time.sleep(2.5)  # Wait for ESP32 to reset after serial connection
                
                # Clear any startup messages
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                
                self.get_logger().info(f'Connected to ESP32 on {serial_port} at {baud_rate} baud')
                
                # Send test command immediately
                time.sleep(0.5)
                test_cmd = "STATE 0 FAULT 0\n"
                self.serial_conn.write(test_cmd.encode('utf-8'))
                self.serial_conn.flush()
                self.get_logger().info(f'Sent test command: {test_cmd.strip()}')
                
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
        else:
            self.get_logger().error('No ESP32 serial port found')
        
        # Create subscriptions
        self.sub_drive_state = self.create_subscription(
            Int32,
            '/av1/avone_state/drive_state',
            self.cb_drive_state,
            10
        )
        
        self.sub_fault = self.create_subscription(
            Int32,
            '/av1/fault_latched/fault_latched',
            self.cb_fault_latched,
            10
        )
        
        # Create timer to read ESP32 responses
        self.create_timer(0.1, self.read_serial)
        
        # Create timer to periodically send combined updates
        self.create_timer(0.2, self.send_update)  # 5 Hz update rate
        
        self.get_logger().info('ESP32 LED Controller node initialized')
    
    def find_esp32_port(self):
        """Auto-detect ESP32-S3 serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # ESP32-S3 typically shows up with these identifiers
            if 'USB' in port.description or 'ESP32' in port.description or \
               'CP210' in port.description or 'CH340' in port.description or \
               '1A86:55D4' in port.hwid:  # CH340 VID:PID
                self.get_logger().info(f'Found potential ESP32 port: {port.device} - {port.description}')
                return port.device
        return None
    
    def read_serial(self):
        """Read and log ESP32 responses"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'ESP32: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')
    
    def cb_drive_state(self, msg):
        """Callback for drive state changes"""
        if self.current_drive_state != msg.data:
            self.get_logger().info(f'Drive state changed: {msg.data}')
            self.current_drive_state = msg.data
            # Force immediate send on state change
            self.send_update()
    
    def cb_fault_latched(self, msg):
        """Callback for fault status changes"""
        if self.fault_latched != msg.data:
            self.get_logger().info(f'Fault latched: {msg.data}')
            self.fault_latched = msg.data
            # Force immediate send on fault change
            self.send_update()
    
    def send_update(self):
        """Send combined state and fault update to ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        # Only send if state changed or periodically
        state_changed = (self.current_drive_state != self.last_sent_state or 
                        self.fault_latched != self.last_sent_fault)
        
        if state_changed or self.debug:
            try:
                fault_val = 1 if self.fault_latched else 0
                cmd = f"STATE {self.current_drive_state} FAULT {fault_val}\n"
                
                self.serial_conn.write(cmd.encode('utf-8'))
                self.serial_conn.flush()
                
                if state_changed or (self.debug and self.get_clock().now().nanoseconds % 10000000000 < 200000000):
                    self.get_logger().info(f'Sent: {cmd.strip()}')
                
                self.last_sent_state = self.current_drive_state
                self.last_sent_fault = self.fault_latched
                
            except serial.SerialException as e:
                self.get_logger().error(f'Serial communication error: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
    
    def destroy_node(self):
        """Cleanup on node shutdown"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Turn off LEDs before closing
                self.serial_conn.write(b"STATE -1 FAULT 0\n")
                self.serial_conn.flush()
                time.sleep(0.1)
            except:
                pass
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32LedController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()