#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TwistStamped

# class CmdVelToAckermannSimple(Node):
#     def __init__(self):
#         super().__init__('cmd_vel_to_ackermann_simple')

#         # Subscribe to /cmd_vel
#         self.sub = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_callback,
#             10
#         )

#         # Publish to Ackermann controller
#         self.pub = self.create_publisher(
#             TwistStamped,
#             '/ackermann_steering_controller/reference',
#             10
#         )

#     def cmd_callback(self, msg: Twist):
#         stamped = TwistStamped()
#         stamped.header.stamp = self.get_clock().now().to_msg()
#         stamped.header.frame_id = 'base_link'

#         # Only allow steering if moving forward
#         if msg.linear.x > 1.5:
#             stamped.twist = msg
#         else:
#             # Stop steering when stopped or reversing
#             stamped.twist.linear.x = max(0.0, msg.linear.x)
#             stamped.twist.angular.z = 0.0

#         self.pub.publish(stamped)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CmdVelToAckermannSimple()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TwistStamped

# class CurvatureSafeCmd(Node):
#     def __init__(self):
#         super().__init__('curvature_safe_cmd')

#         # --- Geometry and limits ---
#         self.L = 1.74
#         self.delta_max_deg = 30.0
#         self.kappa_max = math.tan(math.radians(self.delta_max_deg)) / self.L

#         # --- Low-speed policy ---
#         self.v_hold = 0.8      # below this → no steering
#         self.v_release = 0.9   # above this → allow steering (hysteresis)
#         self.v_eps = 0.1

#         # --- Steering rate (limit δ̇, approximated via ω change) ---
#         self.steer_rate_degps = 150.0   # max steering rate
#         self.timeout_s = 0.5            # zero ω if no updates this long

#         # Topics
#         cmd_in = '/cmd_vel'
#         cmd_out = '/ackermann_steering_controller/reference'

#         self.sub = self.create_subscription(Twist, cmd_in, self.cb, 50)
#         self.pub = self.create_publisher(TwistStamped, cmd_out, 50)

#         # State
#         self.last_time = self.get_clock().now()
#         self.last_update = self.last_time
#         self.last_omega = 0.0
#         self.steering_enabled = False

#         self.get_logger().info(
#             f'Curvature clamp: L={self.L:.2f} m, δ_max={self.delta_max_deg:.1f}°, '
#             f'κ_max={self.kappa_max:.3f} 1/m, v_hold={self.v_hold:.2f} m/s'
#         )

#     def cb(self, msg: Twist):
#         v = float(msg.linear.x)
#         omega = float(msg.angular.z)
#         now = self.get_clock().now()

#         # --- Disable steering below v_hold, enable above v_release ---
#         if self.steering_enabled:
#             if abs(v) < self.v_hold:
#                 self.steering_enabled = False
#         else:
#             if abs(v) > self.v_release:
#                 self.steering_enabled = True

#         if not self.steering_enabled:
#             omega_cmd = 0.0
#         else:
#             omega_cmd = omega

#         # --- Curvature clamp ---
#         v_for_kappa = max(abs(v), self.v_eps)
#         kappa_des = omega_cmd / v_for_kappa
#         kappa_clamped = max(-self.kappa_max, min(self.kappa_max, kappa_des))
#         omega_out = kappa_clamped * v

#         # --- ω rate limit to emulate δ̇ limit ---
#         dt = (now - self.last_time).nanoseconds * 1e-9
#         if dt > 0.0:
#             v_for_rate = max(abs(v), self.v_eps)
#             domega_max = (math.radians(self.steer_rate_degps) * v_for_rate / self.L) * dt
#             d = omega_out - self.last_omega
#             if d > domega_max:
#                 omega_out = self.last_omega + domega_max
#             elif d < -domega_max:
#                 omega_out = self.last_omega - domega_max
#         self.last_omega = omega_out
#         self.last_time = now

#         # --- Timeout: zero ω if no new input for too long ---
#         if (now - self.last_update).nanoseconds * 1e-9 > self.timeout_s:
#             omega_out = 0.0
#         self.last_update = now

#         # --- Publish ---
#         out = TwistStamped()
#         out.header.stamp = now.to_msg()
#         out.header.frame_id = 'base_link'
#         out.twist.linear.x = v
#         out.twist.angular.z = omega_out
#         self.pub.publish(out)

# def main():
#     rclpy.init()
#     rclpy.spin(CurvatureSafeCmd())
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelToAckermannRampHardCutoff(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_ramp_hard')

        # --- Parameters ---
        self.linear_threshold = 0.9      # [m/s] must exceed before steering allowed
        self.angular_ramp_rate = 0.1      # [rad/s²] ramp rate toward target
        self.update_hz = 50.0              # control loop frequency [Hz]

        # --- State ---
        self.current_linear = 0.0
        self.target_omega = 0.0
        self.current_omega = 0.0

        # --- ROS I/O ---
        self.sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.pub = self.create_publisher(
            TwistStamped, '/ackermann_steering_controller/reference', 10
        )
        self.timer = self.create_timer(1.0 / self.update_hz, self.update)

        self.get_logger().info(
            f"CmdVelToAckermannRampHardCutoff active "
            f"(threshold={self.linear_threshold} m/s, ramp={self.angular_ramp_rate} rad/s²)"
        )

    # ------------------------------------------------------------------
    def cmd_callback(self, msg: Twist):
        """Store latest commanded twist from /cmd_vel."""
        self.current_linear = msg.linear.x
        self.target_omega = msg.angular.z

    # ------------------------------------------------------------------
    def update(self):
        """Ramp ω only when above threshold; else force 0 immediately."""
        dt = 1.0 / self.update_hz

        if self.current_linear < self.linear_threshold:
            # 🚫 Below threshold → full lockout
            self.current_omega = 0.0
            omega_out = 0.0
        else:
            # ✅ Above threshold → ramp smoothly toward target
            delta = self.target_omega - self.current_omega
            step = self.angular_ramp_rate * dt
            if delta > step:
                self.current_omega += step
            elif delta < -step:
                self.current_omega -= step
            else:
                self.current_omega = self.target_omega
            omega_out = self.current_omega

        # --- Publish filtered command ---
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist.linear.x = max(0.0, self.current_linear)
        out.twist.angular.z = omega_out
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannRampHardCutoff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
