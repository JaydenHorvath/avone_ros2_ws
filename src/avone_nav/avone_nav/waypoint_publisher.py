#!/usr/bin/env python3

import math
import numpy as np
# patch deprecated numpy float alias
np.float = float
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Create action client for Nav2 NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # List of (x, y) waypoints; yaw will be computed to face next waypoint
        self.waypoints = [
            #(10.7756, -6.9223),
            #(10.6788, -4.02486),
            (10.683,  -0.170739),
            (10.9931, 3.22278),
            (12.5259, 7.77802),
            (14.0978, 12.4749),
            (13.2057, 15.8263),
            (10.8683, 18.3848),
            (8.88971, 18.969),
            (5.90648, 19.137),
            (2.80367, 17.5369),
            (0.382189,16.9477),
            (-1.76519,15.4528),
            (-4.87451,10.5713),
            (-6.31437,7.85892),
            (-7.88561,5.13401),
            (-9.3778,  1.7579),
            (-11.3013,-2.04985),
            (-13.8402,-6.50237),
            (-14.9525,-9.52483),
            (-13.2456,-13.492),
            (-10.3439,-16.1279),
            (-4.76384,-16.6025),
            (5.88574, -15.6553),
            (9.01311, -14.3446)
        ]
        self.current_index = 0

        # Wait for action server
        self.get_logger().info('Waiting for "navigate_to_pose" action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available, starting navigation.')

        # Start navigation
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            return

        x, y = self.waypoints[self.current_index]
        # compute yaw: default 0 for first and last waypoint
        if self.current_index == 0 or self.current_index == len(self.waypoints) - 1:
            yaw = 0.0
        else:
            nx, ny = self.waypoints[self.current_index + 1]
            yaw = math.atan2(ny - y, nx - x)
            # add 180 degrees to face opposite direction
            yaw = yaw + math.pi
            # normalize to [-pi, pi]
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        self.get_logger().info(
            f'Sending goal {self.current_index+1}/{len(self.waypoints)}: x={x}, y={y}, yaw={yaw}')

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Explicitly set z to zero for 2D navigation
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Waypoint {self.current_index+1} reached successfully.')
            self.current_index += 1
            self.send_next_goal()
        else:
            self.get_logger().error(
                f'Failed to reach waypoint {self.current_index+1}, status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
