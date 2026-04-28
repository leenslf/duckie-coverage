import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs

from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._odom_pub = self.create_publisher(Odometry, '/ground_truth/odometry', 10)

        self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self._on_detections,
            10,
        )

    def _on_detections(self, msg: AprilTagDetectionArray):
        for detection in msg.detections:
            if detection.hamming != 0 or detection.decision_margin <= 80.0:
                continue

            tag_frame = f'36h11:{detection.id}'
            det_stamp = msg.header.stamp
            timeout = Duration(seconds=0.1)

            try:
                # world → tag (static, externally published)
                world_to_tag = self._tf_buffer.lookup_transform(
                    'world', tag_frame, Time(), timeout
                )
                # tag → camera (inverse of camera→tag detection, dynamic)
                tag_to_camera = self._tf_buffer.lookup_transform(
                    tag_frame, 'oak_rgb_camera_optical_frame',
                    Time.from_msg(det_stamp), timeout
                )
                # camera → base_link (static, driver chain)
                camera_to_base = self._tf_buffer.lookup_transform(
                    'oak_rgb_camera_optical_frame', 'base_link', Time(), timeout
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().debug(f'TF lookup failed for {tag_frame}: {e}')
                continue

            # Compose: express base_link origin in world frame.
            # do_transform_pose(pose_in_src, lookup(target, src)) → pose_in_target
            base_origin = Pose()
            base_origin.orientation.w = 1.0

            in_camera = tf2_geometry_msgs.do_transform_pose(base_origin, camera_to_base)
            in_tag = tf2_geometry_msgs.do_transform_pose(in_camera, tag_to_camera)
            in_world = tf2_geometry_msgs.do_transform_pose(in_tag, world_to_tag)

            odom = Odometry()
            odom.header.stamp = det_stamp
            odom.header.frame_id = 'world'
            odom.child_frame_id = 'base_link'
            odom.pose.pose = in_world
            self._odom_pub.publish(odom)
            break  # one valid detection per message is sufficient


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
