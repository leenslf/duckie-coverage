#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

# Must match nav_params_sim.yaml
_OP_WIDTH = 0.47   # operation_width (m)
_HEADLAND = 0.85   # default_headland_width (m)


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

        # Path metrics
        self._dist    = 0.0
        self._last_xy = None
        self._poses   = []   # (x, y, yaw) for turn detection
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y = p.x, p.y
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        if self._last_xy is not None:
            self._dist += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
        self._last_xy = (x, y)
        self._poses.append((x, y, yaw))

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field):
        """Send a NavigateCompleteCoverage action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.append(self.toPolygon(field))

        print('Navigating field of size: ' + str(len(field)) + ' vertices...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            return True
        # 1.0 s spin window — processes ~50 odom callbacks per iteration
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=1.0)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task failed with status code: {self.status}')
                return True
        else:
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _polygon_area(coords):
    """Shoelace formula — works for any simple polygon."""
    n = len(coords)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += coords[i][0] * coords[j][1]
        area -= coords[j][0] * coords[i][1]
    return abs(area) / 2.0


def _count_turns(poses):
    """Count E↔W heading reversals — each reversal is one headland turn."""
    turns, prev_dir = 0, None
    for _, _, yaw in poses:
        if abs(yaw) < math.pi / 4:
            d = 'E'
        elif abs(yaw) > 3 * math.pi / 4:
            d = 'W'
        else:
            continue   # mid-turn heading, skip
        if prev_dir is not None and d != prev_dir:
            turns += 1
        prev_dir = d
    return turns


def _print_metrics(field, nav, elapsed_s):
    xs = [c[0] for c in field]
    ys = [c[1] for c in field]
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)

    field_area = _polygon_area(field)
    sw_w   = (x1 - x0) - 2 * _HEADLAND   # E-W row length
    sw_h   = (y1 - y0) - 2 * _HEADLAND   # N-S swath extent
    n_rows = max(1, int(sw_h / _OP_WIDTH))

    total_dist = nav._dist
    # Theoretical minimum = rows only, no turns, no approach
    # (NOT field_area/op_width — that assumes full field coverage incl. headlands,
    #  which would exceed the actual path length and give efficiency > 100%)
    row_dist   = n_rows * sw_w
    n_turns    = _count_turns(nav._poses)
    hl_dist    = total_dist - row_dist

    print('\n=== Coverage Metrics ===')
    print(f'  Path length        : {total_dist:.1f} m')
    print(f'  Theoretical min    : {row_dist:.1f} m'
          f'  ({n_rows} rows × {sw_w:.1f} m, zero turns)')
    print(f'  Path efficiency    : {row_dist / total_dist * 100:.1f}%'
          f'  (100% = straight rows with no turns)')
    print()
    print(f'  Turns detected     : {n_turns}')
    print(f'  Headland overhead  : {hl_dist / total_dist * 100:.1f}%'
          f'  ({hl_dist:.1f} m of {total_dist:.1f} m is non-row travel)')
    print(f'  Avg turn cost      : {hl_dist / n_turns:.1f} m'
          if n_turns > 0 else '  Avg turn cost      : n/a')
    print()
    print(f'  Field area         : {field_area:.2f} m²')
    print(f'  Total time         : {elapsed_s:.1f} s  ({elapsed_s / 60:.2f} min)')
    print(f'  Area rate          : {field_area / (elapsed_s / 60):.2f} m²/min')
    print('========================')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()

    navigator = CoverageNavigatorTester()
    navigator.startup()

    # Field in map frame (= Gazebo world frame, map→odom TF is identity).
    # Walls: west inner x=-0.1, east inner x=9.0, north inner y=-3.3, south inner y=-8.8
    # x∈[1.45, 7.45]: 6m wide, 1.55m clearance from E/W walls for Reeds-Shepp turns.
    # y∈[-8.8, -3.3]: 5.5m tall → headland=0.85m → swath y∈[-7.95,-4.15] = 3.8m → 8 rows.
    # Row centres (N→S): -4.4, -4.87, -5.34, -5.81, -6.28, -6.75, -7.22, -7.69
    # Robot spawns at (2.3, -3.5), drives ~0.9m south to reach first row.
    field = [[1.45, -8.8], [1.45, -3.3], [7.45, -3.3], [7.45, -8.8], [1.45, -8.8]]

    print(f'Field area: {_polygon_area(field):.2f} m²')

    navigator.navigateCoverage(field)
    t_start = time.monotonic()  # goal accepted — robot starts moving now

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            elapsed = time.monotonic() - t_start
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            print(f'Elapsed: {elapsed:.0f}s  ETA: {eta:.0f}s')

    elapsed_total = time.monotonic() - t_start

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        _print_metrics(field, navigator, elapsed_total)
    elif result == TaskResult.CANCELED:
        print(f'Goal was canceled after {elapsed_total:.1f} s')
    elif result == TaskResult.FAILED:
        print(f'Goal failed after {elapsed_total:.1f} s')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
