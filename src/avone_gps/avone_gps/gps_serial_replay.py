#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped
import math
from typing import Optional

class GPSReplayNode(Node):
    def __init__(self):
        super().__init__('gps_replay_node')
        
        # Declare parameters
        self.declare_parameter('log_file', 'gps_log1.txt')
        self.declare_parameter('frame_id', 'gps_link')
        self.declare_parameter('loop', False)
        self.declare_parameter('sync_topic', '/ackermann_steering_controller/odometry')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('start_delay', 0.0)  # NEW: Add delay parameter
        
        # Get parameters
        self.log_file = self.get_parameter('log_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = self.get_parameter('loop').value
        self.sync_topic = self.get_parameter('sync_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.start_delay = self.get_parameter('start_delay').value  # NEW
        
        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/vel', 10)
        
        # Load and parse all GPS data upfront
        self.gps_data = []
        self.vel_data = []
        self.load_log_file()
        
        self.current_index = 0
        self.start_time = None
        self.last_print_time = None
        self.delay_start_time = None  # NEW: Track when delay started
        self.delay_active = self.start_delay > 0.0  # NEW: Flag for delay state
        
        self.get_logger().info(f"Loaded {len(self.gps_data)} GPS fixes from {self.log_file}")
        self.get_logger().info(f"Publishing at {self.publish_rate} Hz")
        
        # NEW: Log delay information
        if self.start_delay > 0.0:
            self.get_logger().info(f'GPS playback will start after {self.start_delay:.1f} second delay')
        
        # Timer to publish at fixed rate - will check delay in callback
        self.create_timer(1.0 / self.publish_rate, self.publish_next)
    
    def load_log_file(self):
        """Load and parse entire log file"""
        try:
            with open(self.log_file, 'r') as f:
                lines = [line.strip() for line in f if line.strip()]
            
            for line in lines:
                if line.startswith('$GPGGA'):
                    data = self.parse_gpgga(line)
                    if data is not None:
                        self.gps_data.append(data)
                elif line.startswith('$GPVTG'):
                    data = self.parse_gpvtg(line)
                    if data is not None:
                        self.vel_data.append(data)
                        
        except Exception as e:
            self.get_logger().error(f"Failed to load log file: {e}")
    
    def parse_gpgga(self, sentence: str) -> Optional[dict]:
        """Parse GPGGA sentence"""
        if not sentence.startswith('$GPGGA'):
            return None
        
        body = sentence.split('*')[0]
        parts = body.split(',')
        
        if len(parts) < 15:
            return None
        
        try:
            # Parse latitude (DDMM.MMMMM)
            lat_str = parts[2]
            lat_dir = parts[3]
            if not lat_str or not lat_dir:
                return None
            
            lat_deg = float(lat_str[:2])
            lat_min = float(lat_str[2:])
            latitude = lat_deg + (lat_min / 60.0)
            if lat_dir == 'S':
                latitude = -latitude
            
            # Parse longitude (DDDMM.MMMMM)
            lon_str = parts[4]
            lon_dir = parts[5]
            if not lon_str or not lon_dir:
                return None
            
            lon_deg = float(lon_str[:3])
            lon_min = float(lon_str[3:])
            longitude = lon_deg + (lon_min / 60.0)
            if lon_dir == 'W':
                longitude = -longitude
            
            quality = int(parts[6]) if parts[6] else 0
            num_sats = int(parts[7]) if parts[7] else 0
            hdop = float(parts[8]) if parts[8] else 99.99
            altitude = float(parts[9]) if parts[9] else 0.0
            geoid_sep = float(parts[11]) if parts[11] else 0.0
            
            return {
                'latitude': latitude,
                'longitude': longitude,
                'altitude': altitude,
                'quality': quality,
                'num_sats': num_sats,
                'hdop': hdop,
                'geoid_separation': geoid_sep
            }
        except (ValueError, IndexError):
            return None
    
    def parse_gpvtg(self, sentence: str) -> Optional[dict]:
        """Parse GPVTG sentence for velocity"""
        if not sentence.startswith('$GPVTG'):
            return None
        
        body = sentence.split('*')[0]
        parts = body.split(',')
        
        if len(parts) < 9:
            return None
        
        try:
            speed_kph_str = parts[7]
            
            speed_ms = 0.0
            if speed_kph_str and speed_kph_str != 'NaN':
                speed_kph = float(speed_kph_str)
                speed_ms = speed_kph / 3.6
            
            track_str = parts[1]
            track_deg = None
            if track_str and track_str != 'NaN':
                track_deg = float(track_str)
            
            return {
                'speed': speed_ms,
                'track': track_deg
            }
        except (ValueError, IndexError):
            return None
    
    def publish_next(self):
        """Publish next GPS fix"""
        if not self.gps_data:
            return
        
        # NEW: Handle startup delay using ROS clock
        if self.delay_active:
            current_time = self.get_clock().now()
            
            # Initialize delay start time
            if self.delay_start_time is None:
                self.delay_start_time = current_time
                self.get_logger().info(f'Starting {self.start_delay:.1f}s delay before GPS playback...')
                return
            
            # Check if delay period has elapsed
            elapsed_delay = (current_time - self.delay_start_time).nanoseconds / 1e9
            if elapsed_delay < self.start_delay:
                return  # Still in delay period, don't publish
            else:
                # Delay complete, start publishing
                self.delay_active = False
                self.get_logger().info('Delay complete, starting GPS playback now!')
        
        if self.current_index >= len(self.gps_data):
            if self.loop:
                self.current_index = 0
                self.start_time = None
                self.get_logger().info("Looping GPS data")
            else:
                return
        
        # Get current time
        current_time = self.get_clock().now()
        
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = current_time
            self.last_print_time = current_time
        
        # Calculate elapsed time
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Print every second
        if self.last_print_time is None or \
           (current_time - self.last_print_time).nanoseconds >= 1e9:
            
            data = self.gps_data[self.current_index]
            progress = (self.current_index / len(self.gps_data)) * 100
            
            # Get velocity if available
            speed = 0.0
            if self.current_index < len(self.vel_data):
                speed = self.vel_data[self.current_index].get('speed', 0.0)
            
            print(f"\r[GPS Replay] Time: {elapsed:7.2f}s | "
                  f"Fix: {self.current_index:4d}/{len(self.gps_data):4d} ({progress:5.1f}%) | "
                  f"Lat: {data['latitude']:10.6f}° | "
                  f"Lon: {data['longitude']:10.6f}° | "
                  f"Alt: {data['altitude']:6.1f}m | "
                  f"Speed: {speed:5.2f}m/s | "
                  f"Sats: {data['num_sats']:2d} | "
                  f"HDOP: {data['hdop']:4.1f}", 
                  end='', flush=True)
            
            self.last_print_time = current_time
        
        # Publish data
        data = self.gps_data[self.current_index]
        self.publish_fix(data)
        
        # Publish corresponding velocity if available
        if self.current_index < len(self.vel_data):
            self.publish_velocity(self.vel_data[self.current_index])
        
        self.current_index += 1
        
        # Print completion message
        if self.current_index >= len(self.gps_data) and not self.loop:
            print()  # New line after progress
            self.get_logger().info("GPS replay complete")
    
    def publish_fix(self, data: dict):
        """Publish NavSatFix message"""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Set status
        msg.status.status = NavSatStatus.STATUS_NO_FIX
        if data['quality'] == 0:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
        elif data['quality'] == 1:
            msg.status.status = NavSatStatus.STATUS_FIX
        elif data['quality'] == 2:
            msg.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif data['quality'] >= 4:
            msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        msg.latitude = data['latitude']
        msg.longitude = data['longitude']
        msg.altitude = data['altitude']
        
        # Set covariance based on HDOP
        hdop = data['hdop']
        position_variance = (hdop * 1.0) ** 2
        
        msg.position_covariance = [
            position_variance, 0.0, 0.0,
            0.0, position_variance, 0.0,
            0.0, 0.0, position_variance * 2
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        
        self.fix_pub.publish(msg)
    
    def publish_velocity(self, data: dict):
        """Publish velocity as TwistStamped"""
        if data['speed'] is None or data['speed'] == 0.0:
            return
        
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        if data['track'] is not None:
            track_rad = math.radians(data['track'])
            msg.twist.linear.x = data['speed'] * math.cos(track_rad)
            msg.twist.linear.y = data['speed'] * math.sin(track_rad)
        else:
            msg.twist.linear.x = data['speed'] *0.5
            msg.twist.linear.y = 0.0
        
        msg.twist.linear.z = 0.0
        
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSReplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()  # New line after Ctrl+C
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()