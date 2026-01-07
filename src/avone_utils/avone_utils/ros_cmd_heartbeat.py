# AV.ONE ROS Command Heartbeat (CAN Watchdog)
# File: ros_cmd_heartbeat.py

# Purpose:
#   - Provide a simple watchdog and heartbeat system on the AV.ONE CAN network.
#   - Prove that the NUC is alive, ROS is alive, and command streaming is alive.
#   - Let downstream control nodes (Teensy, motor controllers, steering controller)
#     gate their behaviour based on system health.

# What it does:
#   1) Listens on SocketCAN for recent actuator command frames:
#        - STEER_CMD_ID (0x009)
#        - RMOTOR_CMD_ID (0x00A)
#        - LMOTOR_CMD_ID (0x00B)

#   2) Monitors /joint_states as a proxy for "hardware interface is alive":
#        - If /joint_states is updating recently, ros2_control + hardware interface is assumed healthy.

#   3) Periodically sends three heartbeat frames:
#        - SYSTEM_HEARTBEAT_ID (0x102): always toggles while this node runs.
#          This proves the NUC process is alive and still executing.
#        - ROS_HEARTBEAT_ID (0x101): toggles only while /joint_states is alive.
#          This proves ROS pipeline + hardware interface are healthy.
#        - CMD_HEARTBEAT_ID (0x100): toggles only while command frames are arriving recently.
#          This proves higher-level command streaming is active.

#   4) Command dropout safety behaviour:
#        - If commands were active and then time out, it sends "zero" commands
#          to motors and steering once, to reduce the chance of stale commands
#          holding throttle or steering.

# Notes:
#   - The actual interpretation of these heartbeat IDs is done by the embedded nodes.
#     Typical behaviour is:
#       * require SYSTEM_HEARTBEAT to allow any entering of autonomous mode
#       * require ROS_HEARTBEAT before accepting higher-level control
#       * require CMD_HEARTBEAT before applying actuator commands else, they are zeroed
#   - This node does not implement a full safety state machine.
#     It is a lightweight "still alive" and "commands flowing" signal source.


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import can
import time
from threading import Thread, Event

# -----------------------------
# CAN IDs and setup
# -----------------------------
CAN_INTERFACE = "can0"

# Incoming message IDs (commands from ROS to actuators)
STEER_CMD_ID = 0x009
RMOTOR_CMD_ID = 0x00A
LMOTOR_CMD_ID = 0x00B

# Outgoing CAN IDs for heartbeats
CMD_HEARTBEAT_ID = 0x100  # Commands active heartbeat
ROS_HEARTBEAT_ID = 0x101  # ROS system alive heartbeat
SYSTEM_HEARTBEAT_ID = 0x102  # NUC/Computer alive heartbeat (always on)

# Timeout (seconds) after last command before setting status=0
CMD_TIMEOUT = 1.0
HEARTBEAT_RATE_HZ = 2.0  # Toggle every 500ms (2 Hz)
JOINT_STATES_TIMEOUT = 1.0  # Hardware interface alive timeout


class CANStatusNode(Node):
    def __init__(self):
        super().__init__("ros_cmd_status")
        self.get_logger().info("Starting ROS Command Status Node")

        # Open CAN socket
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype="socketcan")

        # Track last update time for command messages
        self.last_update_time = {
            STEER_CMD_ID: 0.0,
            RMOTOR_CMD_ID: 0.0,
            LMOTOR_CMD_ID: 0.0,
        }

        # State tracking
        self.commands_active = False
        self.last_commands_active = False
        self.cmd_heartbeat_state = 0

        self.hardware_interface_alive = False
        self.last_joint_states_time = time.time()
        self.ros_heartbeat_state = 0

        self.system_heartbeat_state = 0  # Always toggles if node is running

        self.stop_event = Event()

        # Subscribe to joint_states to monitor hardware interface health
        self.joint_states_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_states_callback, 10
        )

        # Threads
        self.listener_thread = Thread(target=self.listen_can, daemon=True)
        self.listener_thread.start()

        self.heartbeat_thread = Thread(target=self.send_heartbeats, daemon=True)
        self.heartbeat_thread.start()

        self.get_logger().info("ROS Command Status Node ready")
        self.get_logger().info(
            f"SYSTEM_HEARTBEAT: 0x{SYSTEM_HEARTBEAT_ID:03X} (always toggles - proves NUC is on)"
        )
        self.get_logger().info(
            f"ROS_HEARTBEAT: 0x{ROS_HEARTBEAT_ID:03X} (toggles when HW interface alive)"
        )
        self.get_logger().info(
            f"CMD_HEARTBEAT: 0x{CMD_HEARTBEAT_ID:03X} (toggles when commands active)"
        )

    # -----------------------------
    # Joint states callback
    # -----------------------------
    def joint_states_callback(self, msg):
        """Called when joint_states received from hardware interface"""
        self.last_joint_states_time = time.time()
        if not self.hardware_interface_alive:
            self.get_logger().info("Hardware interface is alive")
            self.hardware_interface_alive = True

    # -----------------------------
    # CAN listener
    # -----------------------------
    def listen_can(self):
        self.get_logger().info(
            f"Listening on {CAN_INTERFACE} for command CAN messages..."
        )
        while rclpy.ok() and not self.stop_event.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None:
                continue

            if msg.arbitration_id in self.last_update_time:
                self.last_update_time[msg.arbitration_id] = time.time()
                self.get_logger().debug(
                    f"Received cmd message: 0x{msg.arbitration_id:03X}"
                )

    # -----------------------------
    # Dual heartbeat sender + zero on drop
    # -----------------------------
    def send_heartbeats(self):
        rate = 1.0 / HEARTBEAT_RATE_HZ
        while rclpy.ok() and not self.stop_event.is_set():
            now = time.time()

            # Check command activity
            latest_update = max(self.last_update_time.values())
            self.commands_active = (now - latest_update) < CMD_TIMEOUT

            # Check hardware interface activity
            time_since_joint_states = now - self.last_joint_states_time
            hw_interface_active = time_since_joint_states < JOINT_STATES_TIMEOUT

            if hw_interface_active != self.hardware_interface_alive:
                if not hw_interface_active:
                    self.get_logger().error("Hardware interface stopped publishing!")
                self.hardware_interface_alive = hw_interface_active

            # --- System Heartbeat (0x102) ---
            # ALWAYS toggle - proves NUC/computer is powered and running
            self.system_heartbeat_state = 1 - self.system_heartbeat_state
            try:
                sys_hb_msg = can.Message(
                    arbitration_id=SYSTEM_HEARTBEAT_ID,
                    data=[self.system_heartbeat_state],
                    is_extended_id=False,
                )
                self.bus.send(sys_hb_msg)
                self.get_logger().debug(
                    f"SYSTEM_HEARTBEAT={self.system_heartbeat_state}"
                )
            except can.CanError:
                self.get_logger().warn("CAN send failed (SYSTEM heartbeat)")

            # --- ROS System Heartbeat (0x101) ---
            # Always toggle if hardware interface is alive
            if self.hardware_interface_alive:
                self.ros_heartbeat_state = 1 - self.ros_heartbeat_state
                try:
                    ros_hb_msg = can.Message(
                        arbitration_id=ROS_HEARTBEAT_ID,
                        data=[self.ros_heartbeat_state],
                        is_extended_id=False,
                    )
                    self.bus.send(ros_hb_msg)
                    self.get_logger().debug(f"ROS_HEARTBEAT={self.ros_heartbeat_state}")
                except can.CanError:
                    self.get_logger().warn("CAN send failed (ROS heartbeat)")

            # --- Command Heartbeat (0x100) ---
            # Only toggle if commands are active
            if self.commands_active:
                self.cmd_heartbeat_state = 1 - self.cmd_heartbeat_state
                try:
                    cmd_hb_msg = can.Message(
                        arbitration_id=CMD_HEARTBEAT_ID,
                        data=[self.cmd_heartbeat_state],
                        is_extended_id=False,
                    )
                    self.bus.send(cmd_hb_msg)
                    self.get_logger().debug(f"CMD_HEARTBEAT={self.cmd_heartbeat_state}")
                except can.CanError:
                    self.get_logger().warn("CAN send failed (CMD heartbeat)")

            # --- Detect command dropout and send zeros ---
            if self.last_commands_active and not self.commands_active:
                self.get_logger().info(
                    "Command timeout detected → sending zero messages"
                )

                zero_msgs = [
                    can.Message(
                        arbitration_id=RMOTOR_CMD_ID,
                        data=[0x00, 0x00],
                        is_extended_id=False,
                    ),
                    can.Message(
                        arbitration_id=LMOTOR_CMD_ID,
                        data=[0x00, 0x00],
                        is_extended_id=False,
                    ),
                    can.Message(
                        arbitration_id=STEER_CMD_ID, data=[0x80], is_extended_id=False
                    ),  # 0 deg
                ]
                for msg in zero_msgs:
                    try:
                        self.bus.send(msg)
                        self.get_logger().info(
                            f"Sent zero on 0x{msg.arbitration_id:03X}"
                        )
                    except can.CanError:
                        self.get_logger().warn(
                            f"CAN send failed (zero {msg.arbitration_id:03X})"
                        )

            self.last_commands_active = self.commands_active
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


if __name__ == "__main__":
    main()
