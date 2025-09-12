#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool

ABORTED = 4  # action_msgs/GoalStatus.STATUS_ABORTED

class FollowPathAbortFlag(Node):
    def __init__(self):
        super().__init__('follow_path_abort_flag')
        self.pub = self.create_publisher(Bool, '/avone/collision_detected', 1)
        self.sub = self.create_subscription(
            GoalStatusArray,
            '/follow_path/_action/status',
            self.on_status,
            10
        )
        self.last_state = False

    def on_status(self, msg: GoalStatusArray):
        collided = any(s.status == ABORTED for s in msg.status_list)
        if collided != self.last_state:
            self.last_state = collided
            self.pub.publish(Bool(data=collided))
            self.get_logger().info(f'collision_detected={collided}')

def main():
    rclpy.init()
    rclpy.spin(FollowPathAbortFlag())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
