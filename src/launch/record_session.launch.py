#!/usr/bin/env python3
"""
Record Session — OAK-D Pro sensor stack (ROS 2 Humble).

What this file starts
─────────────────────
  1. depthai_ros_driver_v3  — OAK-D Pro driver (stereo depth enabled)
  2. depth_image_proc/point_cloud_xyz_node  — depth → /oak/points
  3. tf2_ros/static_transform_publisher  — base_link → oak_parent_frame (identity)

Known driver quirks handled here
─────────────────────────────────
  • /oak/stereo/camera_info reports width=0/height=0 → remap camera_info in
    point_cloud_xyz_node to /oak/rgb/camera_info, which has valid 640×400
    intrinsics and matches the aligned depth frame.

Run
───
  # All nodes (default)
  ros2 launch src/launch/record_session.launch.py

  # Skip individual nodes
  ros2 launch src/launch/record_session.launch.py launch_driver:=false
  ros2 launch src/launch/record_session.launch.py launch_point_cloud:=false
  ros2 launch src/launch/record_session.launch.py launch_static_tf:=false

Then start the bag recorder in a second terminal:
  ros2 launch src/launch/record_bag.launch.py
  ros2 launch src/launch/record_bag.launch.py bag_prefix:=my_run_01
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────
    launch_driver_arg = DeclareLaunchArgument(
        'launch_driver',
        default_value='true',
        description='Launch the depthai_ros_driver_v3 OAK-D Pro driver',
    )
    launch_point_cloud_arg = DeclareLaunchArgument(
        'launch_point_cloud',
        default_value='true',
        description='Launch depth_image_proc/point_cloud_xyz_node (depth → /oak/points)',
    )
    launch_static_tf_arg = DeclareLaunchArgument(
        'launch_static_tf',
        default_value='true',
        description='Publish static TF base_link → oak_parent_frame',
    )

    log_start = LogInfo(msg='[record_session] ── Starting OAK-D sensor stack ──')

    # ── 1. OAK-D Pro driver ────────────────────────────────────────────────
    driver_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_driver')),
        actions=[
            LogInfo(msg='[record_session] Step 1: launching depthai_ros_driver_v3 …'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('depthai_ros_driver_v3'),
                    '/launch/driver.launch.py',
                ]),
                launch_arguments={
                    'stereo.enableDepth': 'true',
                }.items(),
            ),
        ],
    )

    # ── 2. Point-cloud node ────────────────────────────────────────────────
    # camera_info is remapped from stereo to RGB: the stereo camera_info
    # publishes width=0/height=0 and is unusable; depth is already aligned
    # to the RGB frame so RGB intrinsics are correct here.
    point_cloud_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_point_cloud')),
        actions=[
            LogInfo(msg='[record_session] Step 2: starting depth_image_proc/point_cloud_xyz_node …'),
            Node(
                package='depth_image_proc',
                executable='point_cloud_xyz_node',
                name='point_cloud_xyz',
                output='screen',
                remappings=[
                    ('image_rect',  '/oak/stereo/image_raw'),
                    ('camera_info', '/oak/rgb/camera_info'),
                    ('points',      '/oak/points'),
                ],
            ),
        ],
    )

    # ── 3. Static TF: base_link → oak_parent_frame ─────────────────────────
    # Identity placeholder (x y z yaw pitch roll).
    # Replace with real extrinsics once the camera is calibrated on the robot.
    static_tf = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_static_tf')),
        actions=[
            LogInfo(msg='[record_session] Step 3: publishing static TF base_link → oak_parent_frame …'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_oak_parent',
                arguments=[
                    '0', '0', '0',      # translation  x y z  (metres)
                    '0', '0', '0',      # rotation     yaw pitch roll  (radians)
                    'base_link',        # parent frame
                    'oak_parent_frame', # child frame
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        launch_driver_arg,
        launch_point_cloud_arg,
        launch_static_tf_arg,
        log_start,
        driver_launch,
        point_cloud_node,
        static_tf,
    ])


if __name__ == '__main__':
    print(
        '\n[record_session] This is a ROS 2 launch file — run it with:\n'
        '  ros2 launch src/launch/record_session.launch.py\n'
    )
