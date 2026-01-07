#!/usr/bin/env python3

# can_2_ros (DBC CAN → ROS 2 bridge)
# ---------------------------------
# Bridges raw CAN frames on a SocketCAN interface (default: can0) into individual ROS 2 topics using a DBC file.

# What it does:
#   - Loads a DBC (cantools) and discovers all messages and signals.
#   - Creates one ROS 2 publisher per signal.
#   - Reads CAN frames in a tight loop, decodes them via the DBC, and publishes signal values immediately.

# Topic naming convention:
#   /av1/<message_name_lower>/<signal_name_lower>

# Message types:
#   - Int32 is used for enumerated signals (signal.choices) and some integer-like signals.
#   - Float64 is used for float/scaled signals and most analog values.

# How to run:
#   - Ensure SocketCAN is up: `ip link set can0 up type can bitrate 250000` (bitrate example)
#   - ros2 run avone_cancan_2_ros --ros-args -p can_interface:=can0

# Key assumptions / gotchas:
#   - This node creates a large number of publishers (one per signal) which is convenient but can be heavy.
#   - The main loop is "real-time-ish": it does not sleep, it polls recv(timeout=0.0) and yields briefly to ROS.
#   - Frame IDs not present in the DBC are ignored.


import rclpy
from rclpy.node import Node
import can
import cantools
from std_msgs.msg import Float64, Int32
from ament_index_python.packages import get_package_share_directory
import os


class DbcCanBridge(Node):
    def __init__(self):
        super().__init__("dbc_can_bridge")

        # Hard coded path - change to NUCAN git repository location
        dbc_path = "/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc"
        if not os.path.exists(dbc_path):
            # Early return means node exists but does nothing; logs show why.
            self.get_logger().error(f"DBC file not found: {dbc_path}")
            return

        try:
            # cantools parses the DBC to enable frame_id -> message decoding
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"Loaded DBC file: {dbc_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC file: {e}")
            return

        # -----------------------------
        # CAN interface setup
        # -----------------------------
        # Parameter allows swapping interfaces without editing code:
        # e.g. can0, can1, vcan0, etc.
        can_interface = self.declare_parameter("can_interface", "can0").value
        try:
            # socketcan uses Linux CAN networking (recommended on Ubuntu).
            # receive_own_messages=False prevents echo if you also transmit on the same bus.
            self.bus = can.interface.Bus(
                channel=can_interface, interface="socketcan", receive_own_messages=False
            )
            self.get_logger().info(f"Connected to CAN interface: {can_interface}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN interface: {e}")
            return

        # -----------------------------
        # ROS publishers (one per signal)
        # -----------------------------
        # Mapping: "<MessageName>_<SignalName>" -> {publisher, msg_type}
        self.signal_publishers = {}
        self._create_signal_publishers()

        self.get_logger().info("CAN → ROS bridge running in real-time mode.")
        self.run_bridge_loop()

    # ------------------------------------------------------------------
    def _create_signal_publishers(self):

        # Create ROS 2 publishers for each signal in the DBC file.

        # Design choice:
        #   - One topic per signal is simple for debugging and graphing in rqt_plot.
        #   - Downside: potentially hundreds of topics and publishers.

        # Type selection heuristic:
        #   - If signal has enumerated choices -> Int32 (publish the numeric enum value)
        #   - If signal is float -> Float64
        #   - Else: pick Float64 if scale/offset implies engineering units, otherwise Int32

        for message in self.db.messages:
            for signal in message.signals:
                # Example: /av1/vehicle_state/ami_state
                topic_name = f"/av1/{message.name.lower()}/{signal.name.lower()}"

                # If choices exist, this is almost always an enum. Publish the numeric value.
                if signal.choices:
                    msg_type = Int32
                elif signal.is_float:
                    msg_type = Float64
                else:
                    # Heuristic: scaled/offset values usually represent real-world units (float-like).
                    # Otherwise treat as integer.
                    msg_type = (
                        Float64
                        if (signal.scale != 1.0 or signal.offset != 0.0)
                        else Int32
                    )

                self.signal_publishers[f"{message.name}_{signal.name}"] = {
                    "publisher": self.create_publisher(msg_type, topic_name, 10),
                    "msg_type": msg_type,
                }

        self.get_logger().info(f"Created {len(self.signal_publishers)} publishers.")

    # ------------------------------------------------------------------
    def run_bridge_loop(self):

        # Tight loop that bridges CAN → ROS with minimal added latency.

        # Implementation notes:
        #   - bus.recv(timeout=0.0) is non-blocking. If no frame is available, we yield to ROS briefly.
        #   - rclpy.spin_once keeps the node responsive (so Ctrl+C works cleanly and any callbacks can run).
        #   - Unknown frame IDs (not in DBC) are ignored.

        while rclpy.ok():
            try:
                # Non-blocking read (timeout=0 = immediate return)
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    # Let ROS process other callbacks to stay responsive
                    rclpy.spin_once(self, timeout_sec=0.001)
                    continue

                # -----------------------------
                # Decode CAN frame using DBC
                # -----------------------------
                try:
                    # Lookup message definition by frame ID
                    dbc_message = self.db.get_message_by_frame_id(msg.arbitration_id)

                    # Decode raw bytes -> dict of {signal_name: value}
                    decoded = self.db.decode_message(msg.arbitration_id, msg.data)
                except KeyError:
                    # Frame ID not present in the DBC
                    continue
                except Exception as e:

                    # Decoding errors can occur if DLC/data length mismatches, corrupted frames, etc.
                    self.get_logger().warn(
                        f"Decode error 0x{msg.arbitration_id:X}: {e}"
                    )
                    continue

                # -----------------------------
                # Publish decoded signals
                # -----------------------------
                for signal_name, value in decoded.items():
                    key = f"{dbc_message.name}_{signal_name}"
                    if key not in self.signal_publishers:
                        continue
                    pub = self.signal_publishers[key]
                    ros_msg = pub["msg_type"]()

                    # Match Python type to ROS message type:
                    # - Int32 expects an int
                    # - Float64 expects a float
                    if pub["msg_type"].__name__ == "Int32":
                        # Round to nearest integer to avoid truncation surprises.
                        ros_msg.data = int(round(value))
                    else:
                        ros_msg.data = float(value)

                    pub["publisher"].publish(ros_msg)

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")
                break


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = DbcCanBridge()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
