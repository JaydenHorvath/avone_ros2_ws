#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can


class StepRpmCanNode(Node):
    """
    Publishes stepwise wheel RPM commands as CAN frames for both wheels:
      - Each wheel starts at start_rpm_* (default 60)
      - Steps up in step_size_* increments until target_rpm_*
      - Then steps back down to start_rpm_*
      - Repeats indefinitely
    """

    def __init__(self):
        super().__init__('step_rpm_can_node_dual')

        # ---- Shared timing params ----
        self.declare_parameter('publish_rate_hz', 50.0)     # send frequency
        self.declare_parameter('step_interval', 4.0)        # seconds between jumps

        # ---- Right wheel params ----
        self.declare_parameter('start_rpm_right', 60.0)
        self.declare_parameter('target_rpm_right', 60.0)
        self.declare_parameter('step_size_right',20.0)
        self.declare_parameter('rmotor_can_id', 0x00A)      # 11 dec

        # ---- Left wheel params ----
        self.declare_parameter('start_rpm_left', 60.0)
        self.declare_parameter('target_rpm_left', 60.0)
        self.declare_parameter('step_size_left', 20.0)
        self.declare_parameter('lmotor_can_id', 0x00B)      # 12 dec

        # ---- CAN params ----
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('extended_id', False)

        # Read params
        self.rate = float(self.get_parameter('publish_rate_hz').value)
        self.step_interval = float(self.get_parameter('step_interval').value)

        self.start_rpm_r = float(self.get_parameter('start_rpm_right').value)
        self.target_rpm_r = float(self.get_parameter('target_rpm_right').value)
        self.step_size_r = float(self.get_parameter('step_size_right').value)
        self.can_id_r = int(self.get_parameter('rmotor_can_id').value)

        self.start_rpm_l = float(self.get_parameter('start_rpm_left').value)
        self.target_rpm_l = float(self.get_parameter('target_rpm_left').value)
        self.step_size_l = float(self.get_parameter('step_size_left').value)
        self.can_id_l = int(self.get_parameter('lmotor_can_id').value)

        self.can_iface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.extended_id = bool(self.get_parameter('extended_id').value)

        # Time base
        self.t0 = self.get_clock().now()
        self.dt = 1.0 / max(self.rate, 1.0)

        # State per wheel
        self.r_dir = 1
        self.l_dir = 1
        self.rpm_r = self.start_rpm_r
        self.rpm_l = self.start_rpm_l
        self.last_step_time = 0.0  # same step moment for both (can split if desired)

        # Init CAN
        try:
            self.bus = can.interface.Bus(channel=self.can_iface, bustype='socketcan')
        except Exception as e:
            self.get_logger().fatal(f"Failed to open {self.can_iface}: {e}")
            raise

        self.timer = self.create_timer(self.dt, self.tick)

        self.get_logger().info(
            f"StepRPM↕ dual on {self.can_iface}, ext={self.extended_id} | "
            f"Right ID=0x{self.can_id_r:03X} {self.start_rpm_r}↔{self.target_rpm_r} step {self.step_size_r} "
            f"| Left ID=0x{self.can_id_l:03X} {self.start_rpm_l}↔{self.target_rpm_l} step {self.step_size_l} "
            f"every {self.step_interval}s"
        )

    @staticmethod
    def encode_rpm_to_raw(rpm: float) -> int:
        """Physical → raw (scale=1, offset=0, 16 bits)"""
        raw = round(rpm)
        return int(max(0, min(65535, raw)))

    def step_value(self, value: float, direction: int, step_size: float, lo: float, hi: float):
        """Apply a step and flip direction at bounds."""
        value += direction * step_size
        if value >= hi:
            value = hi
            direction = -1
        elif value <= lo:
            value = lo
            direction = 1
        return value, direction

    def send_rpm(self, can_id: int, rpm: float):
        raw = self.encode_rpm_to_raw(rpm)
        data = raw.to_bytes(2, byteorder='little', signed=False)
        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=self.extended_id,
            data=data,
            dlc=2,
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(f"CAN send failed (ID=0x{can_id:03X}): {e}")

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9

        # Step both wheels every step_interval seconds
        if t - self.last_step_time >= self.step_interval:
            self.last_step_time = t
            # Right
            self.rpm_r, self.r_dir = self.step_value(
                self.rpm_r, self.r_dir, self.step_size_r, self.start_rpm_r, self.target_rpm_r
            )
            # Left
            self.rpm_l, self.l_dir = self.step_value(
                self.rpm_l, self.l_dir, self.step_size_l, self.start_rpm_l, self.target_rpm_l
            )

        # Transmit both frames (ROS_RMOTOR_RPM_REQUEST @ 0x00B, ROS_LMOTOR_RPM_REQUEST @ 0x00C)
        self.send_rpm(self.can_id_r, self.rpm_r)
        self.send_rpm(self.can_id_l, self.rpm_l)

        self.get_logger().debug(
            f"t={t:.2f}s R={self.rpm_r:.1f}rpm (ID=0x{self.can_id_r:03X}) "
            f"L={self.rpm_l:.1f}rpm (ID=0x{self.can_id_l:03X})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StepRpmCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




#!/usr/bin/env python3

# # single rpm 
# import rclpy
# from rclpy.node import Node
# import can


# class FixedRpmCanNode(Node):
#     """
#     Publishes a fixed wheel RPM command as a CAN frame:
#       BO_ 3 ROS_RPM_REQUEST: 2 ROS
#        SG_ ROS_RPM_REQUEST : 0|16@1+ (1,0) [0|20000] "rpm"

#     Encoding: raw = round(rpm)
#     CAN ID: 0x003 (standard), DLC: 2
#     """

#     def __init__(self):
#         super().__init__('fixed_rpm_can_node')

#         # Parameters
#         self.declare_parameter('target_rpm', 80.0)       # constant RPM value
#         self.declare_parameter('publish_rate_hz', 20.0)    # how often to send
#         self.declare_parameter('can_interface', 'can0')
#         self.declare_parameter('can_id', 0x003)
#         self.declare_parameter('extended_id', False)

#         # Read params
#         self.target_rpm = float(self.get_parameter('target_rpm').value)
#         self.rate = float(self.get_parameter('publish_rate_hz').value)
#         self.can_iface = self.get_parameter('can_interface').get_parameter_value().string_value
#         self.can_id = int(self.get_parameter('can_id').value)
#         self.extended_id = bool(self.get_parameter('extended_id').value)

#         # Init CAN
#         try:
#             self.bus = can.interface.Bus(channel=self.can_iface, bustype='socketcan')
#         except Exception as e:
#             self.get_logger().fatal(f"Failed to open {self.can_iface}: {e}")
#             raise

#         # Start timer
#         self.timer = self.create_timer(1.0 / max(self.rate, 1.0), self.tick)

#         self.get_logger().info(
#             f"FixedRPM→CAN on {self.can_iface}, ID=0x{self.can_id:03X}, "
#             f"ext={self.extended_id} | target={self.target_rpm} rpm"
#         )

#     def encode_rpm_to_raw(self, rpm: float) -> int:
#         raw = round(rpm)
#         raw = max(0, min(65535, raw))
#         return int(raw)

#     def tick(self):
#         rpm = self.target_rpm
#         raw = self.encode_rpm_to_raw(rpm)
#         data = raw.to_bytes(2, byteorder='little', signed=False)

#         msg = can.Message(
#             arbitration_id=self.can_id,
#             is_extended_id=self.extended_id,
#             data=data,
#             dlc=2,
#         )

#         try:
#             self.bus.send(msg)
#             self.get_logger().debug(f"Sent rpm={rpm} raw={raw} data={data.hex()}")
#         except can.CanError as e:
#             self.get_logger().error(f"CAN send failed: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = FixedRpmCanNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
