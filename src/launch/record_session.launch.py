#!/usr/bin/env python3
"""
Record Session — OAK-D Pro one-shot launch file (ROS 2 Humble).

What this file starts
─────────────────────
  1. depthai_ros_driver_v3  — OAK-D Pro driver (stereo depth enabled)
  2. depth_image_proc/point_cloud_xyz_node  — depth → /oak/points
  3. tf2_ros/static_transform_publisher  — base_link → oak_parent_frame (identity)
  4. ros2 bag record  — all key topics, delayed 3 s to let the driver come up

Known driver quirks handled here
─────────────────────────────────
  • /oak/stereo/camera_info reports width=0/height=0 → remap camera_info in
    point_cloud_xyz_node to /oak/rgb/camera_info, which has valid 640×400
    intrinsics and matches the aligned depth frame.

Run
───
  ros2 launch src/launch/record_session.launch.py
  ros2 launch src/launch/record_session.launch.py bag_prefix:=my_run_01
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────
    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix',
        default_value='oak_session',
        description='Output bag name (written to the current working directory)',
    )

    # ── 1. OAK-D Pro driver ────────────────────────────────────────────────
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('depthai_ros_driver_v3'),
            '/launch/driver.launch.py',
        ]),
        launch_arguments={
            'stereo.enableDepth': 'true',
        }.items(),
    )

    # ── 2. Point-cloud node ────────────────────────────────────────────────
    # Converts the 16UC1 depth image (mm) to a PointCloud2 on /oak/points.
    # camera_info is remapped from stereo to RGB: the stereo camera_info
    # publishes width=0/height=0 and is unusable; depth is already aligned
    # to the RGB frame so RGB intrinsics are correct here.
    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz',
        output='screen',
        remappings=[
            ('image_rect',  '/oak/stereo/image_raw'),
            ('camera_info', '/oak/rgb/camera_info'),
            ('points',      '/oak/points'),
        ],
    )

    # ── 3. Static TF: base_link → oak_parent_frame ─────────────────────────
    # Identity placeholder (x y z yaw pitch roll).
    # Replace with real extrinsics once the camera is calibrated on the robot.
    static_tf = Node(
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
    )

    # ── 4. Bag recorder (delayed 3 s to let topics become active) ──────────
    bag_record = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '--output', LaunchConfiguration('bag_prefix'),
                    '/oak/rgb/image_raw',
                    '/oak/rgb/camera_info',
                    '/oak/stereo/image_raw',
                    '/oak/imu/data',
                    '/tf',
                    '/tf_static',
                    '/oak/points',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        bag_prefix_arg,
        driver_launch,
        point_cloud_node,
        static_tf,
        bag_record,
    ])
