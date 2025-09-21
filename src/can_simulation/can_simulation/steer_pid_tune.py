#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can, time, struct
import numpy as np
from scipy.optimize import minimize, differential_evolution


class SteerPidOptimizer(Node):
    def __init__(self):
        super().__init__('steer_pid_optimizer')

        # -------------------------------
        # CAN setup
        # -------------------------------
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.steer_cmd_id = 0x009        # steering command
        self.steer_feedback_id = 0x00D   # feedback (STEER_ANG_ACTUAL)
        self.pid_tune_id = 0x180         # tuning gains

        # -------------------------------
        # Test parameters
        # -------------------------------
        self.rate = 50.0  # Hz loop rate

        # Step test params
        self.test_angles = [10.0, -10.0, 20.0, -20.0]
        self.hold_time = 4.0

        # Ramp test params
        self.ramp_start = -30.0
        self.ramp_end = 30.0
        self.ramp_duration = 6.0  # seconds

        # Mode (step or ramp)
        self.mode = "step"  # change to "ramp" for ramp mode

    # -------------------------------
    # CAN helpers
    # -------------------------------
    def encode_deg_to_raw(self, angle):
        raw = round((angle + 90.0) / 0.7)
        return max(0, min(255, raw))

    def send_steer(self, angle):
        raw = self.encode_deg_to_raw(angle)
        msg = can.Message(arbitration_id=self.steer_cmd_id,
                          is_extended_id=False,
                          data=bytes([raw]), dlc=1)
        self.bus.send(msg)

    def send_pid(self, kp, ki, kd):
        data = struct.pack('<hhh', int(kp*100), int(ki*100), int(kd*100))
        msg = can.Message(arbitration_id=self.pid_tune_id,
                          is_extended_id=False,
                          data=data, dlc=6)
        self.bus.send(msg)
        self.get_logger().info(f"Applied PID → Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")

    # -------------------------------
    # Step test
    # -------------------------------
    def run_step_test(self, kp, ki, kd):
        self.send_pid(kp, ki, kd)
        feedback, reference = [], []

        for step in self.test_angles:
            self.get_logger().info(f"Step → {step:.1f}°")
            t0 = time.time()
            while time.time() - t0 < self.hold_time:
                self.send_steer(step)
                msg = self.bus.recv(timeout=0.01)
                if msg and msg.arbitration_id == self.steer_feedback_id:
                    angle = msg.data[0] * 0.7 - 90
                    feedback.append(angle)
                    reference.append(step)
                time.sleep(1.0 / self.rate)

        self.send_steer(0.0)

        if len(feedback) < len(reference) // 2:
            return 1e6

        feedback = np.array(feedback)
        reference = np.array(reference)
        error = reference - feedback

        mse = np.mean(error**2)
        overshoot = max(0, (np.max(np.abs(feedback)) - np.max(np.abs(reference))) / np.max(np.abs(reference)))
        steady_err = np.mean(np.abs(error[-int(self.rate*self.hold_time/2):]))

        cost = mse + 5.0*overshoot + 2.0*steady_err
        self.get_logger().info(
            f"[Step Test] mse={mse:.3f} overshoot={overshoot:.2f} steady_err={steady_err:.2f} cost={cost:.3f}"
        )
        return cost

    # -------------------------------
    # Ramp test
    # -------------------------------
    def run_ramp_test(self, kp, ki, kd):
        self.send_pid(kp, ki, kd)
        n_steps = int(self.ramp_duration * self.rate)
        ramp = np.linspace(self.ramp_start, self.ramp_end, n_steps)

        feedback = []
        t0 = time.time()
        for i, cmd in enumerate(ramp):
            self.send_steer(float(cmd))
            msg = self.bus.recv(timeout=0.01)
            if msg and msg.arbitration_id == self.steer_feedback_id:
                angle = msg.data[0] * 0.7 - 90
                feedback.append(angle)
            while time.time() - t0 < (i+1)/self.rate:
                time.sleep(0.001)

        self.send_steer(0.0)

        if len(feedback) < n_steps // 2:
            return 1e6

        feedback = np.array(feedback)
        ramp = ramp[:len(feedback)]
        error = ramp - feedback

        mse = np.mean(error**2)
        lag = np.mean(np.abs(np.gradient(error)))
        cost = mse + 3.0*lag

        self.get_logger().info(
            f"[Ramp Test] mse={mse:.3f} lag={lag:.3f} cost={cost:.3f}"
        )
        return cost

    # -------------------------------
    # Optimization pipeline
    # -------------------------------
    def optimize(self):
        bounds = [(3.0, 4.0), (3.0, 4.0), (1.0, 1.5)]

        def cost_fn(params):
            kp, ki, kd = params
            if self.mode == "step":
                return self.run_step_test(kp, ki, kd)
            elif self.mode == "ramp":
                return self.run_ramp_test(kp, ki, kd)
            else:
                self.get_logger().error(f"Unknown mode {self.mode}")
                return 1e6

        self.get_logger().info(f"🌍 Starting optimization in {self.mode} mode...")
        result_de = differential_evolution(cost_fn, bounds, maxiter=10, polish=False)
        best_global = result_de.x
        self.get_logger().info(f"Global best → {best_global}")

        self.get_logger().info("🔍 Refining with Nelder–Mead...")
        result_nm = minimize(cost_fn, best_global, method='Nelder-Mead',
                             options={'maxiter': 20, 'disp': True})
        best = result_nm.x

        self.get_logger().info(f"✅ Final optimized PID: {best}")
        self.send_pid(*best)


def main(args=None):
    rclpy.init(args=args)
    node = SteerPidOptimizer()
    node.optimize()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
