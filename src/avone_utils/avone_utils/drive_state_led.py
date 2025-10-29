#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import serial.tools.list_ports
import time


class ESP32LedController(Node):
    def __init__(self):
        super().__init__('esp32_led_controller')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('auto_detect_port', False)
        self.declare_parameter('debug', True)
        self.declare_parameter('topic_timeout_sec', 1.5)
        
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        auto_detect = self.get_parameter('auto_detect_port').value
        self.debug = self.get_parameter('debug').value
        self.topic_timeout = self.get_parameter('topic_timeout_sec').value
        
        # Internal states
        self.current_drive_state = -1
        self.fault_latched = False
        self.last_sent_state = None
        self.last_sent_fault = None
        self.topics_alive = True  # tracks if topics are publishing
        
        # Watchdog tracking
        now = self.get_clock().now()
        self.last_drive_state_time = now
        self.last_fault_time = now
        
        # Serial setup
        self.serial_conn = None
        if auto_detect:
            serial_port = self.find_esp32_port()
        
        if serial_port:
            try:
                self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1, write_timeout=1)
                time.sleep(2.5)
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.get_logger().info(f'Connected to ESP32 on {serial_port} at {baud_rate} baud')

                # Send startup test command
                test_cmd = "STATE 0 FAULT 0\n"
                self.serial_conn.write(test_cmd.encode('utf-8'))
                self.serial_conn.flush()
                self.get_logger().info(f'Sent test command: {test_cmd.strip()}')

            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {serial_port}: {e}')
        else:
            self.get_logger().error('No ESP32 serial port found')
        
        # ROS topic subscriptions
        self.sub_drive_state = self.create_subscription(
            Int32, '/av1/avone_state/drive_state', self.cb_drive_state, 10
        )
        self.sub_fault = self.create_subscription(
            Int32, '/av1/fault_latched/fault_latched', self.cb_fault_latched, 10
        )
        
        # Timers
        self.create_timer(0.1, self.read_serial)
        self.create_timer(0.2, self.send_update)      # normal sending
        self.create_timer(0.5, self.check_topic_alive)  # watchdog
        
        self.get_logger().info('ESP32 LED Controller node initialized')
    
    def find_esp32_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description or 'ESP32' in port.description or \
               'CP210' in port.description or 'CH340' in port.description or \
               '1A86:55D4' in port.hwid:
                self.get_logger().info(f'Found potential ESP32 port: {port.device}')
                return port.device
        return None
    
    def read_serial(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                while self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'ESP32: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')
    
    def cb_drive_state(self, msg):
        self.current_drive_state = msg.data
        self.last_drive_state_time = self.get_clock().now()
        if not self.topics_alive:
            self.get_logger().info('Topic updates restored — resuming normal operation')
        self.topics_alive = True  # mark alive
        self.get_logger().debug(f'Drive state update: {msg.data}')
    
    def cb_fault_latched(self, msg):
        self.fault_latched = bool(msg.data)
        self.last_fault_time = self.get_clock().now()
        if not self.topics_alive:
            self.get_logger().info('Topic updates restored — resuming normal operation')
        self.topics_alive = True  # mark alive
        self.get_logger().debug(f'Fault latched update: {msg.data}')
    
    def check_topic_alive(self):
        """Check if topics are still publishing; if not, fault and stop sending"""
        now = self.get_clock().now()
        drive_age = (now - self.last_drive_state_time).nanoseconds / 1e9
        fault_age = (now - self.last_fault_time).nanoseconds / 1e9
        
        # Timeout condition
        if drive_age > self.topic_timeout or fault_age > self.topic_timeout:
            if self.topics_alive:
                self.get_logger().warn(
                    f"No topic updates for >{self.topic_timeout}s "
                    f"(drive_age={drive_age:.1f}s, fault_age={fault_age:.1f}s). Entering FAULT HOLD."
                )
                # Send one fault frame, then pause updates
                self.send_cmd(0, 1)
                self.topics_alive = False  # stop sending until updates resume
    
    def send_cmd(self, state, fault):
        """Low-level serial write"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            cmd = f"STATE {state} FAULT {fault}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.serial_conn.flush()
            if self.debug:
                self.get_logger().info(f'Sent: {cmd.strip()}')
        except Exception as e:
            self.get_logger().error(f'Error writing serial: {e}')
    
    def send_update(self):
        """Only send when topics are alive"""
        if not self.topics_alive:
            return  # paused due to timeout
        
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        fault_val = 1 if self.fault_latched else 0
        state_changed = (
            self.current_drive_state != self.last_sent_state or
            fault_val != self.last_sent_fault
        )
        
        if state_changed or self.debug:
            self.send_cmd(self.current_drive_state, fault_val)
            self.last_sent_state = self.current_drive_state
            self.last_sent_fault = fault_val
    
    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(b"STATE -1 FAULT 0\n")
                self.serial_conn.flush()
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
