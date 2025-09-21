import rclpy
from rclpy.node import Node
import can
import cantools
from std_msgs.msg import Float64, Int32, Bool
import os

class DbcCanBridge(Node):
    def __init__(self):
        super().__init__('dbc_can_bridge')
        
        # Load your specific DBC file
        dbc_path = 'AV1.dbc'
        
        # Check if file exists
        if not os.path.exists(dbc_path):
            self.get_logger().error(f"DBC file not found: {dbc_path}")
            return
            
        try:
            self.db = cantools.database.load_file(dbc_path)
            self.get_logger().info(f"Loaded DBC file: {dbc_path}")
            self.get_logger().info(f"Found {len(self.db.messages)} messages in DBC")
        except Exception as e:
            self.get_logger().error(f"Failed to load DBC file: {e}")
            return
        
        # Set up CAN bus
        can_interface = self.declare_parameter('can_interface', 'can0').value
        
        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
            self.get_logger().info(f"Connected to CAN interface: {can_interface}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CAN interface: {e}")
            return
        
        # Dictionary to store publishers for each signal
        self.signal_publishers = {}
        
        # Create publishers for all signals in DBC
        self._create_signal_publishers()
        
        # Timer to read CAN messages
        self.timer = self.create_timer(0.001, self.read_and_decode_messages)
    
    def _create_signal_publishers(self):
        """Create ROS2 publishers for each signal in the DBC file"""
        for message in self.db.messages:
            self.get_logger().info(f"Processing message: {message.name} (ID: 0x{message.frame_id:x})")
            
            for signal in message.signals:
                topic_name = f"/av1/{message.name.lower()}/{signal.name.lower()}"
                
                # Choose appropriate message type based on signal properties
                if signal.choices:  # Enumerated values
                    msg_type = Int32
                elif signal.scale == 1.0 and signal.offset == 0.0 and not signal.is_float:
                    msg_type = Int32
                else:
                    msg_type = Float64
                
                self.signal_publishers[f"{message.name}_{signal.name}"] = {
                    'publisher': self.create_publisher(msg_type, topic_name, 10),
                    'msg_type': msg_type,
                    'signal': signal,
                    'message_name': message.name
                }
                
                self.get_logger().info(f"Created topic: {topic_name}")
        
        self.get_logger().info(f"Created {len(self.signal_publishers)} signal publishers")
    
    def read_and_decode_messages(self):
        """Read CAN messages and decode using DBC"""
        message = self.bus.recv(timeout=0)
        if message is None:
            return
            
        try:
            # Find the message definition in DBC
            dbc_message = self.db.get_message_by_frame_id(message.arbitration_id)
            
            # Decode the message
            decoded_data = self.db.decode_message(message.arbitration_id, message.data)
            
            # Publish each signal to its topic
            for signal_name, value in decoded_data.items():
                publisher_key = f"{dbc_message.name}_{signal_name}"
                
                if publisher_key in self.signal_publishers:
                    pub_info = self.signal_publishers[publisher_key]
                    
                    # Create and publish message
                    ros_msg = pub_info['msg_type']()
                    ros_msg.data = float(value) if pub_info['msg_type'] == Float64 else int(value)
                    pub_info['publisher'].publish(ros_msg)
                    
        except KeyError:
            # CAN ID not found in DBC - this is normal for unknown messages
            pass
        except Exception as e:
            self.get_logger().warn(f"Error decoding message 0x{message.arbitration_id:x}: {e}")

def main():
    rclpy.init()
    node = DbcCanBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()