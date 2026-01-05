#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import can  # python-can

class SineSteerCanNode(Node):
    """
    Publishes a sine-wave steering command as a CAN frame:
      BO_ 4 ROS_STEER_ANG_TARGET: 1 ROS
       SG_ ROS_STEER_ANG_TARGET : 0|8@1+ (0.7,-90) [-90|90] "deg"

    Encoding: raw = round((angle_deg + 90) / 0.7)
    CAN ID: 0x004 (standard), DLC: 1
    """

    def __init__(self):
        super().__init__('sine_steer_can_node')

        # Sine params (degrees, Hz)
        self.declare_parameter('amplitude_deg', 30.0)
        self.declare_parameter('offset_deg', 0.0)
        self.declare_parameter('frequency_hz', 0.1)
        self.declare_parameter('publish_rate_hz', 50.0)

        # Safety clamp for angle before encoding
        self.declare_parameter('min_deg', -35.0)
        self.declare_parameter('max_deg',  35.0)

        # CAN params
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id', 0x009)          # standard 11-bit
        self.declare_parameter('extended_id', False)     # must be False for BO_ 4
        self.declare_parameter('dbc_scale', 0.7)         # (0.7, -90)
        self.declare_parameter('dbc_offset', -90.0)
        self.declare_parameter('dbc_min_raw', 0)
        self.declare_parameter('dbc_max_raw', 255)

        # Read params
        self.A = float(self.get_parameter('amplitude_deg').value)
        self.y0 = float(self.get_parameter('offset_deg').value)
        self.f  = float(self.get_parameter('frequency_hz').value)
        self.rate = float(self.get_parameter('publish_rate_hz').value)

        self.min_deg = float(self.get_parameter('min_deg').value)
        self.max_deg = float(self.get_parameter('max_deg').value)

        self.can_iface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.can_id = int(self.get_parameter('can_id').value)
        self.extended_id = bool(self.get_parameter('extended_id').value)

        self.scale = float(self.get_parameter('dbc_scale').value)
        self.offset = float(self.get_parameter('dbc_offset').value)
        self.min_raw = int(self.get_parameter('dbc_min_raw').value)
        self.max_raw = int(self.get_parameter('dbc_max_raw').value)

        # Time base
        self.t0 = self.get_clock().now()
        self.dt = 1.0 / max(self.rate, 1.0)

        # Init CAN bus (SocketCAN)
        try:
            self.bus = can.interface.Bus(channel=self.can_iface, bustype='socketcan')
        except Exception as e:
            self.get_logger().fatal(f"Failed to open {self.can_iface}: {e}")
            raise

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"Sine→CAN on {self.can_iface}, ID=0x{self.can_id:03X}, ext={self.extended_id} | "
            f"A={self.A}° f={self.f} Hz offset={self.y0}° rate={self.rate} Hz"
        )

    def encode_deg_to_raw(self, angle_deg: float) -> int:
        """
        Physical → raw for (scale, offset) = (0.7, -90):
          angle = scale*raw + offset  => raw = (angle - offset)/scale
        """
        raw = round((angle_deg - self.offset) / self.scale)
        raw = max(self.min_raw, min(self.max_raw, raw))
        return int(raw)

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Generate sine (deg) and clamp to DBC range
        angle = self.y0 + self.A * math.sin(2.0 * math.pi * self.f * t)
        angle = max(self.min_deg, min(self.max_deg, angle))

        # Encode to 1 byte
        raw = self.encode_deg_to_raw(angle)
        data = bytes([raw])

        # Build and send CAN frame
        msg = can.Message(
            arbitration_id=self.can_id,
            is_extended_id=self.extended_id,
            data=data,
            dlc=1,
        )

        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f"CAN send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SineSteerCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # python-can Bus cleans up on GC; explicit shutdown not mandatory
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import can  # python-can


# class StepSteerCanNode(Node):
#     """
#     Publishes stepped steering commands between -35° and +35°
#     in increments of 10° every 2 seconds, starting at +5°.
#     """

#     def __init__(self):
#         super().__init__('step_steer_can_node')

#         # Params
#         self.declare_parameter('min_deg', -20.0)
#         self.declare_parameter('max_deg', 20.0)
#         self.declare_parameter('step_deg', 10.0)
#         self.declare_parameter('start_deg', 0.0)          # start at 5°
#         self.declare_parameter('step_period_s', 5.0)      # hold each step for 2s
#         self.declare_parameter('publish_rate_hz', 50.0)   # bus send frequency

#         # CAN params
#         self.declare_parameter('can_interface', 'can0')
#         self.declare_parameter('can_id', 0x009)
#         self.declare_parameter('extended_id', False)
#         self.declare_parameter('dbc_scale', 0.7)
#         self.declare_parameter('dbc_offset', -90.0)
#         self.declare_parameter('dbc_min_raw', 0)
#         self.declare_parameter('dbc_max_raw', 255)

#         # Read params
#         self.min_angle = float(self.get_parameter('min_deg').value)
#         self.max_angle = float(self.get_parameter('max_deg').value)
#         self.step = float(self.get_parameter('step_deg').value)
#         self.start_angle = float(self.get_parameter('start_deg').value)
#         self.period = float(self.get_parameter('step_period_s').value)
#         self.rate = float(self.get_parameter('publish_rate_hz').value)

#         self.can_iface = self.get_parameter('can_interface').get_parameter_value().string_value
#         self.can_id = int(self.get_parameter('can_id').value)
#         self.extended_id = bool(self.get_parameter('extended_id').value)

#         self.scale = float(self.get_parameter('dbc_scale').value)
#         self.offset = float(self.get_parameter('dbc_offset').value)
#         self.min_raw = int(self.get_parameter('dbc_min_raw').value)
#         self.max_raw = int(self.get_parameter('dbc_max_raw').value)

#         # Build step sequence
#         up = list(range(int(self.start_angle), int(self.max_angle) + 1, int(self.step)))   # start→+35
#         down = list(range(int(self.max_angle - self.step), int(self.min_angle) - 1, -int(self.step)))  # +25→-35
#         back_up = list(range(int(self.min_angle + self.step), int(self.start_angle) + 1, int(self.step)))  # -25→start

#         self.sequence = up + down + back_up
#         self.index = 0
#         self.current_angle = self.sequence[self.index]

#         # Init CAN
#         try:
#             self.bus = can.interface.Bus(channel=self.can_iface, bustype='socketcan')
#         except Exception as e:
#             self.get_logger().fatal(f"Failed to open {self.can_iface}: {e}")
#             raise

#         # Timers
#         self.publish_dt = 1.0 / max(self.rate, 1.0)
#         self.timer_publish = self.create_timer(self.publish_dt, self.publish_tick)
#         self.timer_step = self.create_timer(self.period, self.step_tick)

#         self.get_logger().info(
#             f"Step→CAN on {self.can_iface}, ID=0x{self.can_id:03X} "
#             f"range=({self.min_angle}° to {self.max_angle}°) step={self.step}° every {self.period}s, starting at {self.start_angle}°"
#         )

#     def encode_deg_to_raw(self, angle_deg: float) -> int:
#         raw = round((angle_deg - self.offset) / self.scale)
#         return int(max(self.min_raw, min(self.max_raw, raw)))

#     def step_tick(self):
#         """Advance to next step every period"""
#         self.index = (self.index + 1) % len(self.sequence)
#         self.current_angle = self.sequence[self.index]

#     def publish_tick(self):
#         """Publish current step at high rate to CAN bus"""
#         raw = self.encode_deg_to_raw(self.current_angle)
#         data = bytes([raw])

#         msg = can.Message(
#             arbitration_id=self.can_id,
#             is_extended_id=self.extended_id,
#             data=data,
#             dlc=1,
#         )

#         try:
#             self.bus.send(msg)
#         except can.CanError as e:
#             self.get_logger().error(f"CAN send failed: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = StepSteerCanNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
