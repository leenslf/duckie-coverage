#!/usr/bin/env python3

"""
Named waypoint client for Nav2.

Usage:
    ros2 run bot_nav_client named_waypoint_client gate

This loads waypoint names from config/waypoints.yaml and sends the selected
pose to Nav2's /navigate_to_pose action server.

CHANGED/ADDED:
- Added named waypoint lookup from YAML.
- Reuses Nav2's NavigateToPose action server.
- Keeps user/bot commands simple: "go to gate" instead of raw x/y/yaw.
"""

import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import yaml

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    """
    Convert planar yaw to quaternion.

    For ground robots, roll and pitch are zero.
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class NamedWaypointClient(Node):
    def __init__(self):
        super().__init__("named_waypoint_client")

        # CHANGED/ADDED: Load waypoints from installed package config.
        package_share = get_package_share_directory("bot_nav_client")
        self.waypoints_path = os.path.join(package_share, "config", "waypoints.yaml")
        self.waypoints = self.load_waypoints(self.waypoints_path)

        # CHANGED/ADDED: Nav2 action client.
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

    def load_waypoints(self, path: str) -> dict:
        """
        Load waypoint dictionary from YAML file.
        """
        if not os.path.exists(path):
            self.get_logger().error(f"Waypoint file not found: {path}")
            return {}

        with open(path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        if not data or "waypoints" not in data:
            self.get_logger().error(
                "Invalid waypoint file. Expected top-level key: waypoints"
            )
            return {}

        return data["waypoints"]

    def send_named_waypoint(self, waypoint_name: str):
        """
        Look up a named waypoint and send it to Nav2.
        """
        if waypoint_name not in self.waypoints:
            available = ", ".join(sorted(self.waypoints.keys()))
            self.get_logger().error(
                f"Unknown waypoint '{waypoint_name}'. Available: {available}"
            )
            rclpy.shutdown()
            return

        waypoint = self.waypoints[waypoint_name]

        try:
            x = float(waypoint["x"])
            y = float(waypoint["y"])
            yaw = float(waypoint["yaw"])
        except KeyError as exc:
            self.get_logger().error(
                f"Waypoint '{waypoint_name}' is missing required field: {exc}"
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"Selected waypoint '{waypoint_name}': x={x}, y={y}, yaw={yaw}"
        )

        self.send_goal(x, y, yaw)

    def send_goal(self, x: float, y: float, yaw: float):
        """
        Send a map-frame pose goal to Nav2.
        """
        self.get_logger().info("Waiting for /navigate_to_pose action server...")

        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Could not find /navigate_to_pose. "
                "Check that Nav2 is launched and bt_navigator is active."
            )
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()

        # CHANGED/ADDED: Build map-frame target pose.
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

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2.")
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

    if len(sys.argv) != 2:
        print("Usage:")
        print("  ros2 run bot_nav_client named_waypoint_client <waypoint_name>")
        print("")
        print("Example:")
        print("  ros2 run bot_nav_client named_waypoint_client gate")
        rclpy.shutdown()
        return

    waypoint_name = sys.argv[1]

    node = NamedWaypointClient()
    node.send_named_waypoint(waypoint_name)

    rclpy.spin(node)


if __name__ == "__main__":
    main()
