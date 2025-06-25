#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class WheelStateToJointState(Node):
    def __init__(self):
        super().__init__('wheelspeedtest')

        # Must exactly match your URDF joint names:
        self.joint_names = [
            'FLWheel',
            'FRWheel',
            'RLMotor',
            'RRMotor'
        ]
        self.js = JointState()
        self.js.name = self.joint_names
        self.js.position = [0.0]*4
        self.js.velocity = [0.0]*4

        self.last_time = self.get_clock().now()

        # publisher of JointState
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # subscribe to your four wheel-speed topics
          # Subscribe to the Int32 motor_speed and forward into cb()
        self.subs = []
        for i in range(4):
            sub = self.create_subscription(
                Int32,                      # <<— Int32 here
                '/motor_speed',
                lambda msg, i=i: self.cb_int(msg, i),
                10
            )
            self.subs.append(sub)

    def cb(self, msg: Float32, idx: int):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # velocity is wheel angular velocity
        self.js.velocity[idx] = msg.data
        # integrate to get joint angle
        self.js.position[idx] += msg.data * dt

        # stamp & publish
        self.js.header.stamp = now.to_msg()
        self.pub.publish(self.js)

    def cb_int(self, msg: Int32, idx: int):
        # Cast Int32 → Float32 and reuse your cb()
        fake = Float32(data=float(msg.data))
        self.cb(fake, idx)

def main():
    rclpy.init()
    node = WheelStateToJointState()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
