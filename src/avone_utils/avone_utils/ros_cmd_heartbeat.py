#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import time
from threading import Thread, Event

# -----------------------------
# CAN IDs and setup
# -----------------------------
CAN_INTERFACE = "can0"

# Incoming message IDs
STEER_CMD_ID = 0x009
RMOTOR_CMD_ID = 0x00A
LMOTOR_CMD_ID = 0x00B

# Outgoing CAN ID for status message
SENSOR_STATUS_ID = 0x100

# Timeout (seconds) after last command before setting status=0
CMD_TIMEOUT = 1.0
STATUS_RATE_HZ = 5.0  # send at 5 Hz


class CANStatusNode(Node):
    def __init__(self):
        super().__init__('ros_cmd_status')
        self.get_logger().info("Starting ROS Command Status Node")

        # Open CAN socket
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan')

        # Track last update time for command messages
        self.last_update_time = {
            STEER_CMD_ID: 0.0,
            RMOTOR_CMD_ID: 0.0,
            LMOTOR_CMD_ID: 0.0,
        }

        self.status = 0  # 0 = inactive, 1 = active
        self.last_status = 0
        self.stop_event = Event()

        # Threads
        self.listener_thread = Thread(target=self.listen_can, daemon=True)
        self.listener_thread.start()

        self.status_thread = Thread(target=self.send_status, daemon=True)
        self.status_thread.start()

    # -----------------------------
    # CAN listener
    # -----------------------------
    def listen_can(self):
        self.get_logger().info(f"Listening on {CAN_INTERFACE} for command CAN messages...")
        while rclpy.ok() and not self.stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue

            if msg.arbitration_id in self.last_update_time:
                self.last_update_time[msg.arbitration_id] = time.time()
                self.get_logger().debug(f"Received cmd message: 0x{msg.arbitration_id:03X}")

    # -----------------------------
    # Status sender + zero on drop
    # -----------------------------
    def send_status(self):
        rate = 1.0 / STATUS_RATE_HZ
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.time()
            latest_update = max(self.last_update_time.values())

            # Determine status
            self.status = 1 if (now - latest_update) < CMD_TIMEOUT else 0

            # Send CAN status message
            try:
                status_msg = can.Message(
                    arbitration_id=SENSOR_STATUS_ID,
                    data=[self.status],
                    is_extended_id=False
                )
                self.bus.send(status_msg)
                self.get_logger().debug(f"Sent STATUS={self.status}")
            except can.CanError:
                self.get_logger().warn("CAN send failed (status)")

            # Detect falling edge: went from active → inactive
            if self.last_status == 1 and self.status == 0:
                self.get_logger().info("Command timeout detected → sending zero messages")

                zero_msgs = [
                    can.Message(arbitration_id=RMOTOR_CMD_ID, data=[0x00, 0x00], is_extended_id=False),
                    can.Message(arbitration_id=LMOTOR_CMD_ID, data=[0x00, 0x00], is_extended_id=False),
                    can.Message(arbitration_id=STEER_CMD_ID, data=[0x80], is_extended_id=False),  # 0 deg per DBC
                ]
                for msg in zero_msgs:
                    try:
                        self.bus.send(msg)
                        self.get_logger().info(f"Sent zero on 0x{msg.arbitration_id:03X}")
                    except can.CanError:
                        self.get_logger().warn(f"CAN send failed (zero {msg.arbitration_id:03X})")

            # Update last status
            self.last_status = self.status

            time.sleep(rate)

    # -----------------------------
    # Shutdown handler
    # -----------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down ROS Command Status Node")
        self.stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
