#!/usr/bin/env python3
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from opennav_coverage_msgs.action import NavigateCompleteCoverage

# 4x4 m square centred near the robot spawn at (0.67, -0.01).
# Robot must be clearly inside the polygon; previous polygon had the robot
# at y=-0.01 which is outside the y=0.0 boundary, causing goal rejection.
FIELD_POLYGON = [
    [-1.0, -2.0],
    [ 3.0, -2.0],
    [ 3.0,  2.0],
    [-1.0,  2.0],
    [-1.0, -2.0],  # close the ring (GML requirement)
]


def _build_polygon(points):
    poly = Polygon()
    for x, y in points:
        pt = Point32()
        pt.x = float(x)
        pt.y = float(y)
        poly.points.append(pt)
    return poly


def main():
    rclpy.init()
    node = Node('coverage_client')
    action_client = ActionClient(node, NavigateCompleteCoverage, 'navigate_complete_coverage')

    node.get_logger().info('Waiting for action server...')
    while not action_client.wait_for_server(timeout_sec=1.0):
        if not rclpy.ok():
            node.get_logger().info('Interrupted while waiting for server.')
            action_client.destroy()
            node.destroy_node()
            rclpy.shutdown()
            return

    goal = NavigateCompleteCoverage.Goal()
    goal.frame_id = 'map'
    goal.polygons.append(_build_polygon(FIELD_POLYGON))

    node.get_logger().info(f'Sending goal with {len(FIELD_POLYGON)}-point polygon...')

    done = threading.Event()

    def on_goal_response(future):
        goal_handle = future.result()
        if goal_handle.accepted:
            node.get_logger().info('Goal accepted.')
        else:
            node.get_logger().info('Goal rejected.')
        done.set()

    send_future = action_client.send_goal_async(goal)
    send_future.add_done_callback(on_goal_response)

    while rclpy.ok() and not done.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
