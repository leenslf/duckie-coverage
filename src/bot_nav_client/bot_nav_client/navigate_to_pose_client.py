#!/usr/bin/env python3

"""
Bot navigation client for Nav2.

This node sends a single target pose to Nav2's /navigate_to_pose action server.

Example:
    ros2 run bot_nav_client navigate_to_pose_client 1.0 2.0 0.0

Arguments:
    x    Target x position in map frame, meters
    y    Target y position in map frame, meters
    yaw  Target heading in radians

CHANGED/ADDED:
- New standalone client node in a separate package.
- Uses Nav2's NavigateToPose action server instead of directly publishing /cmd_vel.
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    """
    Convert planar yaw angle to quaternion.

    For a ground robot, roll and pitch are zero.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__("navigate_to_pose_client")

        # CHANGED/ADDED: Action client that talks to Nav2's navigation server.
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

    def send_goal(self, x: float, y: float, yaw: float):
        self.get_logger().info("Waiting for Nav2 /navigate_to_pose action server...")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Could not find /navigate_to_pose. "
                "Check that Nav2 is launched and bt_navigator is active."
            )
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()

        # CHANGED/ADDED: Build goal pose in the map frame.
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg.pose = pose

        self.get_logger().info(
            f"Sending goal to Nav2: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by Nav2.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by Nav2.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        distance_remaining = getattr(feedback, "distance_remaining", None)

        if distance_remaining is not None:
            self.get_logger().info(
                f"Distance remaining: {distance_remaining:.2f} m"
            )
        else:
            self.get_logger().info("Navigation feedback received.")

    def result_callback(self, future):
        wrapped_result = future.result()

        self.get_logger().info(
            f"Navigation finished with status code: {wrapped_result.status}"
        )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 4:
        print("Usage:")
        print("  ros2 run bot_nav_client navigate_to_pose_client <x> <y> <yaw>")
        print("")
        print("Example:")
        print("  ros2 run bot_nav_client navigate_to_pose_client 1.0 2.0 0.0")
        rclpy.shutdown()
        return

    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
    except ValueError:
        print("Error: x, y, and yaw must be numbers.")
        rclpy.shutdown()
        return

    node = NavigateToPoseClient()
    node.send_goal(x, y, yaw)

    rclpy.spin(node)


if __name__ == "__main__":
    main()