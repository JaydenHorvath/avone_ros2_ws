#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D
from sensor_msgs.msg import CameraInfo
import can
import threading
import math

class ConeSteeringController(Node):
    def __init__(self):
        super().__init__('cone_steering_controller')
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/yolo/camera_info',
            self.camera_info_callback,
            10
        )
        
        # CAN bus setup
        self.setup_can_bus()
        
        # Camera parameters (will be updated from camera_info)
        self.image_width = 640  # Default, will be updated
        self.image_center_x = 320  # Default, will be updated
        
        # Control parameters
        self.steering_gain = 0.5  # Adjust this to tune steering sensitivity
        self.max_steering_angle = 45.0  # Maximum steering angle in degrees
        self.target_cone_class = 'orange_cone'  # Updated for orange cones
        
        # Status
        self.camera_info_received = False
        
        self.get_logger().info('Cone Steering Controller initialized for ORANGE cones')
    
    def setup_can_bus(self):
        """Initialize CAN bus connection"""
        try:
            self.can_bus = can.interface.Bus(
                channel='can0',
                bustype='socketcan',
                bitrate=250000
            )
            self.get_logger().info('CAN bus initialized successfully')
            
            # IMMEDIATE TEST - send a message right now
            try:
                test_msg = can.Message(arbitration_id=0x123, data=[0xAA, 0xBB], is_extended_id=False)
                self.can_bus.send(test_msg)
                self.get_logger().info('✓ Test message sent successfully')
            except Exception as e:
                self.get_logger().error(f'✗ Test message failed: {e}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.can_bus = None
    
    def camera_info_callback(self, msg):
        """Update camera parameters from camera_info"""
        self.image_width = msg.width
        self.image_center_x = msg.width / 2.0
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info(f'Camera info received: {msg.width}x{msg.height}')
    
    def detection_callback(self, msg):
        """Process YOLO detections and send steering commands"""
        if not self.camera_info_received:
            self.get_logger().warn('Camera info not received yet, using default parameters')
        
        # DEBUG: Log all detections first
        self.get_logger().info(f'Received {len(msg.detections)} total detections')
        
        # Find orange cone detections
        orange_cones = self.filter_orange_cones(msg.detections)
        
        self.get_logger().info(f'Found {len(orange_cones)} orange cones')
        
        if not orange_cones:
            # No orange cones detected - maintain current steering or go straight
            self.get_logger().info('No orange cones detected, steering straight')
            self.send_steering_command(0.0)
            return
        
        # Find the largest/closest orange cone (assuming larger detection = closer)
        target_cone = self.select_target_cone(orange_cones)
        
        # DEBUG: Log cone position details
        cone_center_x = target_cone.bbox.center.position.x
        cone_center_y = target_cone.bbox.center.position.y
        cone_width = target_cone.bbox.size_x
        cone_height = target_cone.bbox.size_y
        
        self.get_logger().info(
            f'Target cone: center=({cone_center_x:.1f}, {cone_center_y:.1f}), '
            f'size=({cone_width:.1f}x{cone_height:.1f}), '
            f'image_center={self.image_center_x:.1f}, image_width={self.image_width}'
        )
        
        # Calculate steering angle based on cone position
        steering_angle = self.calculate_steering_angle(target_cone)
        
        # DEBUG: Log calculation details
        offset_pixels = cone_center_x - self.image_center_x
        offset_normalized = offset_pixels / (self.image_width / 2.0)
        
        self.get_logger().info(
            f'Steering calc: offset_pixels={offset_pixels:.1f}, '
            f'offset_normalized={offset_normalized:.3f}, '
            f'raw_angle={offset_normalized * self.max_steering_angle * self.steering_gain:.1f}, '
            f'final_angle={steering_angle:.1f}°'
        )
        
        # Send CAN message
        self.send_steering_command(steering_angle)
    
    def filter_orange_cones(self, detections):
        """Filter detections to find orange cones"""
        orange_cones = []
        
        for detection in detections:
            # Check if this detection has results
            if hasattr(detection, 'results') and detection.results:
                for result in detection.results:
                    if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'class_id'):
                        class_id = result.hypothesis.class_id
                        
                        # Check confidence first
                        confidence = result.hypothesis.score if hasattr(result.hypothesis, 'score') else 0.0
                        
                        # Debug log each detection
                        self.get_logger().info(f'Detection: class_id={class_id}, confidence={confidence:.3f}')
                        
                        if self.is_orange_cone(class_id, result):
                            orange_cones.append(detection)
                            self.get_logger().info(f'✓ Accepted orange cone with confidence {confidence:.3f}')
                            break  # Found a valid orange cone in this detection
                        else:
                            self.get_logger().info(f'✗ Rejected: not an orange cone or low confidence')
        
        return orange_cones
    
    def is_orange_cone(self, class_id, result):
        """
        Determine if a detection is an orange cone.
        Based on your YOLO data, class_id '2' appears to be orange cones.
        """
        # Your YOLO model uses class_id '2' for orange cones (as string)
        if class_id == '2':
            return True
        
        # Also check if it's integer 2 (just in case)
        if class_id == 2:
            return True
            
        # Check confidence score - only accept detections above threshold
        if hasattr(result, 'hypothesis') and hasattr(result.hypothesis, 'score'):
            if result.hypothesis.score < 0.5:  # Minimum confidence threshold
                return False
        
        return False
    
    def select_target_cone(self, orange_cones):
        """Select which orange cone to target (e.g., largest/closest)"""
        if not orange_cones:
            return None
        
        # Strategy 1: Select the largest cone (assuming larger = closer)
        largest_cone = max(orange_cones, 
                          key=lambda cone: cone.bbox.size_x * cone.bbox.size_y)
        
        # Strategy 2: Select the cone closest to image center
        # closest_to_center = min(orange_cones,
        #                        key=lambda cone: abs(cone.bbox.center.x - self.image_center_x))
        
        return largest_cone
    
    def calculate_steering_angle(self, detection):
        """Calculate steering angle based on cone position in image"""
        # Get cone center position - note the correct attribute path
        cone_center_x = detection.bbox.center.position.x
        
        # Calculate offset from image center (-1 to 1)
        offset_normalized = (cone_center_x - self.image_center_x) / (self.image_width / 2.0)
        
        # Convert to steering angle
        steering_angle = offset_normalized * self.max_steering_angle * self.steering_gain
        
        # Clamp to maximum steering angle
        steering_angle = max(-self.max_steering_angle, 
                           min(self.max_steering_angle, steering_angle))
        
        return steering_angle
    
    def send_steering_command(self, angle_degrees):
        """Send steering command via CAN bus"""
        if self.can_bus is None:
            self.get_logger().warn(f'CAN bus not available, would send: {angle_degrees:.1f}°')
            return
        
        try:
            # Convert angle to CAN message format
            # Based on your DBC: (0.353, -45) means: value = (raw_value * 0.353) - 45
            # So: raw_value = (value + 45) / 0.353
            raw_value = (angle_degrees + 45) / 0.353
            
            # DEBUG: Log the conversion process
            self.get_logger().info(
                f'CAN conversion: angle={angle_degrees:.1f}° -> '
                f'raw_value_float={raw_value:.2f} -> '
                f'raw_value_int={int(raw_value)} -> '
                f'clamped={max(0, min(255, int(raw_value)))}'
            )
            
            # Convert to integer and clamp to 8-bit range (0-255)
            raw_value_int = int(raw_value)
            raw_value_clamped = max(0, min(255, raw_value_int))
            
            # Create CAN message
            # ID 9 (0x09), 1 byte of data
            message = can.Message(
                arbitration_id=0x09,  # ROS_STEER_ANG_TARGET
                data=[raw_value_clamped],
                is_extended_id=False
            )
            
            # Send message
            self.can_bus.send(message)
            
            # DEBUG: Log what was actually sent
            self.get_logger().info(f'Sent CAN: ID=0x09, data=[{raw_value_clamped:02X}] ({raw_value_clamped})')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    controller = ConeSteeringController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()