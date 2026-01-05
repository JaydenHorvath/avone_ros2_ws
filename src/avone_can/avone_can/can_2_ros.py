#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import cantools
from std_msgs.msg import Float64, Int32
import os

class DbcCanBridge(Node):
    def __init__(self):
        super().__init__('dbc_can_bridge')

        # Load DBC
        dbc_path = '/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc'
        if not os.path.exists(dbc_path):
            self.get_logger().error(f"DBC file not found: {dbc_path}")
            return

        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"Loaded DBC file: {dbc_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC file: {e}")
            return

        # Open CAN interface
        can_interface = self.declare_parameter('can_interface', 'can0').value
        try:
            self.bus = can.interface.Bus(channel=can_interface, interface='socketcan', receive_own_messages=False)
            self.get_logger().info(f"Connected to CAN interface: {can_interface}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN interface: {e}")
            return

        # Publishers dictionary
        self.signal_publishers = {}
        self._create_signal_publishers()

        self.get_logger().info("CAN → ROS bridge running in real-time mode.")
        self.run_bridge_loop()

    # ------------------------------------------------------------------
    def _create_signal_publishers(self):
        """Create ROS2 publishers for each signal in the DBC file"""
        for message in self.db.messages:
            for signal in message.signals:
                topic_name = f"/av1/{message.name.lower()}/{signal.name.lower()}"
                if signal.choices:
                    msg_type = Int32
                elif signal.is_float:
                    msg_type = Float64
                else:
                    msg_type = Float64 if (signal.scale != 1.0 or signal.offset != 0.0) else Int32

                self.signal_publishers[f"{message.name}_{signal.name}"] = {
                    'publisher': self.create_publisher(msg_type, topic_name, 10),
                    'msg_type': msg_type
                }

        self.get_logger().info(f"Created {len(self.signal_publishers)} publishers.")

    # ------------------------------------------------------------------
    def run_bridge_loop(self):
        """Tight loop that directly bridges CAN → ROS without delay"""
        while rclpy.ok():
            try:
                # Non-blocking read (timeout=0 = immediate return)
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    # Let ROS process other callbacks to stay responsive
                    rclpy.spin_once(self, timeout_sec=0.001)
                    continue

                try:
                    dbc_message = self.db.get_message_by_frame_id(msg.arbitration_id)
                    decoded = self.db.decode_message(msg.arbitration_id, msg.data)
                except KeyError:
                    continue
                except Exception as e:
                    self.get_logger().warn(f"Decode error 0x{msg.arbitration_id:X}: {e}")
                    continue

                # Publish immediately
                # Publish immediately
                for signal_name, value in decoded.items():
                    key = f"{dbc_message.name}_{signal_name}"
                    if key not in self.signal_publishers:
                        continue
                    pub = self.signal_publishers[key]
                    ros_msg = pub['msg_type']()

                    # FIX: match Python type to ROS message type
                    if pub['msg_type'].__name__ == "Int32":
                        ros_msg.data = int(round(value))
                    else:
                        ros_msg.data = float(value)

                    pub['publisher'].publish(ros_msg)


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

if __name__ == '__main__':
    main()
