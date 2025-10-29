
#!/usr/bin/env python3
# file: gps_odom_fix.py
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

class GPSOdomFix(Node):
    def __init__(self):
        super().__init__('gps_odom_fix')
        self.declare_parameter('in_topic',  '/odometry/gps')
        self.declare_parameter('out_topic', '/odometry/gps_fixed')
        self.declare_parameter('sigma_xy',  3.0)
        self.declare_parameter('use_local_yaw', True)
        self.declare_parameter('local_odom_topic', '/odometry/local')

        self.in_topic  = self.get_parameter('in_topic').get_parameter_value().string_value
        self.out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        self.sigma_xy  = float(self.get_parameter('sigma_xy').value)
        self.use_local_yaw = bool(self.get_parameter('use_local_yaw').value)
        self.local_yaw = None

        self.sub_gps = self.create_subscription(Odometry, self.in_topic, self.on_gps, 10)
        if self.use_local_yaw:
            self.sub_loc = self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self.on_local, 10)
        self.pub = self.create_publisher(Odometry, self.out_topic, 10)

        var_xy = self.sigma_xy ** 2
        BIG = 9999.0
        self.pose_cov = [
            var_xy,0.0,  0.0,  0.0,0.0,0.0,
            0.0,  var_xy,0.0,  0.0,0.0,0.0,
            0.0,  0.0,   0.0,  0.0,0.0,0.0,
            0.0,  0.0,   0.0,  0.0, 0.0,0.0,
            0.0,  0.0,   0.0,  0.0,0.0, 0.0,
            0.0,  0.0,   0.0,  0.0,0.0,0.0
        ]
        self.twist_cov = [BIG if i % 7 == 0 else 0.0 for i in range(36)]

    def on_local(self, msg: Odometry):
        q = msg.pose.pose.orientation
        s = 2.0*(q.w*q.z + q.x*q.y)
        c = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.local_yaw = math.atan2(s, c)

    def on_gps(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'      # ensure odom
        out.child_frame_id  = 'base_link' # fill child frame
        out.pose.pose = msg.pose.pose
        if self.use_local_yaw and self.local_yaw is not None:
            out.pose.pose.orientation = yaw_to_quat(self.local_yaw)  # optional
        out.pose.covariance  = self.pose_cov
        out.twist.twist      = msg.twist.twist
        out.twist.covariance = self.twist_cov
        self.pub.publish(out)

def main():
    rclpy.init(); node = GPSOdomFix(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
