#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math

class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')

        # --- parameters ---
        self.wheel_radius = 0.203       # meters
        self.kp = 2.0                   # P gain
        self.max_throttle = 100.0       # %
        self.max_regen = 100.0          # %

        # --- state ---
        self.v_des = 0.0                # desired speed (m/s)
        self.v_meas = 0.0               # measured speed (m/s)

        # --- subscriptions ---
        # desired forward speed
        self.create_subscription(
            Float64,
            '/desired_velocity',
            self.velocity_callback,
            10)
        # joint_states for actual motor RPM → v_meas
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # --- publishers ---
        self.throttle_pub = self.create_publisher(Float64, '/throttle_cmd', 10)
        self.regen_pub    = self.create_publisher(Float64, '/regen_cmd',    10)
        self.meas_pub     = self.create_publisher(Float64, '/measured_velocity', 10)

        # --- timer ---
        self.create_timer(0.1, self.control_loop)  # 10 Hz

    def velocity_callback(self, msg: Float64):
        self.v_des = msg.data

    def joint_state_callback(self, msg: JointState):
        # Now read from the actual driven joints:
        #   'RLMotor' and 'RRMotor' are your hub‐motor joints.
        try:
            i_rl = msg.name.index('RLMotor')
            i_rr = msg.name.index('RRMotor')
        except ValueError:
            return

        # velocities are in rad/s; convert to m/s with wheel radius
        vel_rl = msg.velocity[i_rl] * self.wheel_radius
        vel_rr = msg.velocity[i_rr] * self.wheel_radius

        # average the two motors
        self.v_meas = 0.5 * (vel_rl + vel_rr)

    def control_loop(self):
        # 1) Publish measured velocity
        self.meas_pub.publish(Float64(data=self.v_meas))

        # 2) P‐controller
        error = self.v_des - self.v_meas
        u = self.kp * error

        # 3) Split into throttle/regen
        throttle = max(0.0, min(u,  self.max_throttle))
        regen    = max(0.0, min(-u, self.max_regen))

        # 4) Publish actuator commands
        self.throttle_pub.publish(Float64(data=throttle))
        self.regen_pub   .publish(Float64(data=regen))

def main(args=None):
    rclpy.init(args=args)
    node = MotorSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
