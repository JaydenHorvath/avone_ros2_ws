#!/usr/bin/env python3


# AV.ONE Nav2 Goal Gate and Cancel Controller
# Package: avone_nav
# Node: nav2_cancel (Nav2ControlNode)

# Purpose:
#   - Gate Nav2 NavigateToPose goals behind an explicit "Start" command from the remote controller
#   - Allow the remote controller to cancel any active Nav2 goal at any time
#   - Support a workflow where operators can:
#       1) Set a goal in RViz (/goal_pose)
#       2) Confirm safety / readiness
#       3) Press Start to begin moving
#       4) Press Cancel to stop and abandon the goal

# How it works:
#   - Subscribes to:
#       - /goal_pose (PoseStamped) from RViz "2D Goal Pose"
#       - /av1/nav2_start_cmd/nav2_start_cmd (Int32) remote start button
#       - /av1/nav2_cancel_cmd/nav2_cancel_cmd (Int32) remote cancel button
#   - When a new /goal_pose arrives:
#       - Stores it as last_goal
#       - Sets goal_pending = True (holding state)
#       - Immediately cancels any currently running Nav2 goals so the robot does not move
#   - When Start is pressed (rising edge 0->1):
#       - If a goal is pending, it sends the stored goal to NavigateToPose
#   - When Cancel is pressed (rising edge 0->1):
#       - Calls the Nav2 CancelGoal service to cancel active goals

# Notes:
#   - Uses edge detection on Start/Cancel so holding the button does not repeatedly trigger actions.
#   - Cancel uses CancelGoal service with stamp=0 to cancel "all goals" behaviour in Nav2 action cancel.
#   - This node does not check navigation state feedback; it only gates when goals are allowed to begin.


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient


class Nav2ControlNode(Node):
    def __init__(self):
        super().__init__("nav2_control_node")

        # Cancel service client
        self._cancel_client = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )

        # NavigateToPose action client
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Last goal storage
        self.last_goal = None
        self.goal_pending = False  # flag: new goal waiting for start

        # Track previous states for edge detection
        self.prev_cancel = 0
        self.prev_start = 0

        # Subscriptions
        self.create_subscription(
            Int32, "/av1/nav2_cancel_cmd/nav2_cancel_cmd", self.cancel_callback, 10
        )

        self.create_subscription(
            Int32, "/av1/nav2_start_cmd/nav2_start_cmd", self.start_callback, 10
        )

        self.create_subscription(PoseStamped, "/goal_pose", self.goal_tap_callback, 10)

        self.get_logger().info(
            "Nav2 gated control node ready (requires start button to move)."
        )

    def goal_tap_callback(self, msg: PoseStamped):
        # Wrap PoseStamped into a NavigateToPose.Goal
        goal = NavigateToPose.Goal()
        goal.pose = msg
        self.last_goal = goal
        self.goal_pending = True

        self.get_logger().warn(
            "New goal received → holding until Start button is pressed."
        )

        # Cancel any active goals right away
        if self._cancel_client.service_is_ready():
            request = CancelGoal.Request()
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0
            self._cancel_client.call_async(request)

    def send_goal(self, goal_pose: NavigateToPose.Goal):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available.")
            return

        self.get_logger().info("Sending NavigateToPose goal to Nav2...")
        future = self._action_client.send_goal_async(goal_pose)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected.")
            return
        self.get_logger().info("Nav2 goal accepted.")

    def cancel_callback(self, msg: Int32):
        if msg.data == 1 and self.prev_cancel == 0:  # edge detect
            if not self._cancel_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("CancelGoal service not available.")
                return

            request = CancelGoal.Request()
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0

            self.get_logger().warn("Cancel button pressed → cancelling Nav2 goals...")
            self._cancel_client.call_async(request)

        self.prev_cancel = msg.data

    def start_callback(self, msg: Int32):
        if msg.data == 1 and self.prev_start == 0:  # edge detect
            if self.goal_pending and self.last_goal is not None:
                self.get_logger().warn("Start button pressed → resending held goal.")
                self.send_goal(self.last_goal)
                self.goal_pending = False
            else:
                self.get_logger().info("Start pressed but no pending goal.")
        self.prev_start = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
