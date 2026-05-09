#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from gazebo_msgs.msg import ModelStates


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states',
                                 self.model_states_cb, 10)
        self.get_logger().info('Ground truth odometry node started')

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index('moborobo_robot')
        except ValueError:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]
        now = self.get_clock().now().to_msg()
        if now.sec == 0:
            return  # sim clock not yet initialized; skip to avoid zero-timestamp TF

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose = pose
        odom.twist.twist = twist
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = GroundTruthOdom()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
