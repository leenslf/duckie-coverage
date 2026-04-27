#!/usr/bin/env python3
# tester.py 
from enum import Enum
import time

from numpy import float32
from std_msgs.msg import Float32
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class CoverageTester(Node):
    def __init__(self):
        super().__init__(node_name='_coverage_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        # Coverage action client
        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')
        
        # these are optional things we can show at the demo, we will add this implemetation in voverage_state
        # # State control service clients
        # self.start_planning_client = self.create_client(Trigger, '/start_planning')
        # self.start_cleaning_client = self.create_client(Trigger, '/start_cleaning')
        # self.stop_client = self.create_client(Trigger, '/stop')
        
        # # Progress publisher (for state machine to track percentage)
        # self.progress_publisher = self.create_publisher(Float32, 'coverage_distance_remaining', 10)
        
        self.get_logger().info("Coverage tester initialized - service-driven mode")

    def call_state_service(self, client, service_name):
        """Helper to call state services"""
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f'{service_name} service not available')
            return False
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            self.get_logger().info(f'{service_name}: {future.result().message}')
            return future.result().success
        return False

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = float(coord[0])
            pt.y = float(coord[1])
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field):
        """Send a coverage navigation request with state management"""
        
        # 1. Trigger planning state
        self.call_state_service(self.start_planning_client, 'start_planning')
        
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.polygons.append(self.toPolygon(field))

        print(f'Starting coverage of field with {len(field)} points...')
        
        send_goal_future = self.coverage_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Coverage navigation request was rejected!')
            return False

        print("Coverage goal accepted, planning...")
        
        # Small delay to let planning happen, then trigger cleaning
        time.sleep(2.0)  # Give planning time
        
        # 2. Trigger cleaning state
        self.call_state_service(self.start_cleaning_client, 'start_cleaning')
        
        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the coverage task is complete"""
        if not self.result_future:
            return True
            
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        
        if self.result_future.result():
            self.status = self.result_future.result().status
            
            # 3. Trigger stop state when done
            self.call_state_service(self.stop_client, 'stop')
            
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Coverage task failed with status code: {self.status}')
            else:
                print('Coverage task succeeded!')
            return True
        
        return False

    def _feedbackCallback(self, msg):
        """Publish progress for state machine"""
        self.feedback = msg.feedback
        if self.feedback:
            distance_msg = Float32()
            distance_msg.data = self.feedback.distance_remaining
            # current_pose = self.feedback.current_pose
            # print(f'Position: ({current_pose.pose.position.x:.2f}, {current_pose.pose.position.y:.2f}), ')
            self.progress_publisher.publish(distance_msg)

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Wait for the navigation stack to become active
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

def main():
    rclpy.init()

    navigator = CoverageTester()
    navigator.startup()
    
    # test_field = [ 
        # technially these are the correct measurements of the field however I decided to add a little margin to the 
        # Y since the headland width is fixed for both sides of the rectangle so we get better control with this setup 
    #     [22.59, 16.58],
    #     [7.72, 16.58],
    #     [7.72, 8.92],
    #     [22.59, 8.92],
    #     [22.59, 16.58]
    # ]
    test_field = [
        [22.59, 17.1],
        [7.72, 17.1],
        [7.72, 9.0],
        [22.59, 9.0],
        [22.59, 17.1]
    ]
    print(f"Starting coverage mission for field: {test_field}")
    
    if not navigator.navigateCoverage(test_field):
        print("Failed to start coverage mission!")
        return

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        rclpy.spin_once(navigator, timeout_sec=0.1)
        
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            try:
                time_remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f'Estimated time remaining: {time_remaining:.0f} seconds')
            except:
                print('Coverage in progress...')
            
        time.sleep(1)

    # Check final result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Coverage mission completed successfully!')
    elif result == TaskResult.CANCELED:
        print('Coverage mission was canceled!')
    elif result == TaskResult.FAILED:
        print('Coverage mission failed!')
    else:
        print('Coverage mission has an invalid return status!')

if __name__ == '__main__':
    main()