#!/usr/bin/env python3
"""
RTAB-Map RGB-D Odometry — OAK-D Pro launch file (ROS 2 Humble).

What this file starts
─────────────────────
  1. static_transform_publisher  base_link → oak_parent_frame  (identity)
     Completes the TF chain so rtabmap can look up sensor poses relative
     to the robot base.

  2. rgbd_odometry (rtabmap_ros)
     Visual-inertial odometry using OAK-D Pro RGB + aligned depth + IMU.
     Publishes /odom and the TF edge  odom → base_link  at ~20 Hz.

Known driver quirks handled here
─────────────────────────────────
  • /oak/stereo/camera_info reports width=0/height=0  →  depth/camera_info
    is remapped to /oak/rgb/camera_info, which has valid 640×400 intrinsics
    and matches the aligned depth frame.

  • /oak/imu/data covariance matrices are all-zero  →  set
    Imu/IgnoreAccCovariance and Imu/IgnoreGyroCovariance to "true" so
    rtabmap accepts the data without rejecting it as invalid.

  • /oak/imu/data orientation is a placeholder identity quaternion (not
    estimated)  →  subscribe_imu_orientation=False; treat as raw 6-DOF.

Run
───
  ros2 launch vio rtabmap_odom.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── 1. Static TF: base_link → oak_parent_frame ────────────────────────
    # Identity transform (x y z yaw pitch roll).  Replace with real extrinsics
    # once a proper calibration is available.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_oak_parent',
        arguments=[
            '0', '0', '0',          # translation  x y z  (metres)
            '0', '0', '0',          # rotation     yaw pitch roll  (radians)
            'base_link',            # parent frame
            'oak_parent_frame',     # child  frame
        ],
        output='screen',
    )

    # ── 2. RGB-D Odometry node (rtabmap_ros, no SLAM back-end) ────────────
    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odom',
        output='screen',
        parameters=[{
            # Frames ──────────────────────────────────────────────────────
            'frame_id':      'base_link',   # robot base / IMU integration frame
            'odom_frame_id': 'odom',        # odometry origin
            'publish_tf':    True,          # emit TF edge odom → base_link

            # Input topology ──────────────────────────────────────────────
            'subscribe_depth':           True,
            'subscribe_imu':             True,   # fuse IMU with visual odometry
            # OAK-D Pro IMU does not estimate orientation — use raw accel/gyro.
            'subscribe_imu_orientation': False,

            # Synchronisation ─────────────────────────────────────────────
            # Camera (20 Hz) and IMU run at different rates; approximate sync
            # is required to align their messages.
            'approx_sync':              True,
            'approx_sync_max_interval': 0.0,   # 0 = auto (≈ half camera period)
            'queue_size':               30,

            # IMU quirk workarounds ────────────────────────────────────────
            # The driver publishes all-zero covariance matrices; without these
            # flags rtabmap discards every IMU message as invalid.
            'Imu/IgnoreAccCovariance':  'true',
            'Imu/IgnoreGyroCovariance': 'true',

            # Continuous output ────────────────────────────────────────────
            # Disable the motion-threshold gate so every camera frame produces
            # an /odom message, maintaining the full ~20 Hz output rate.
            'Odom/LinearUpdate':  '0',
            'Odom/AngularUpdate': '0',

            # Odometry strategy ───────────────────────────────────────────
            # 0 = Frame-to-Map (F2M): re-localises each frame against a local
            # map of key-points; more robust than frame-to-frame for slow motion.
            'Odom/Strategy': '0',
        }],
        remappings=[
            # Node's internal name        →  actual ROS topic (driver output)
            ('rgb/image',        '/oak/rgb/image_raw'),
            ('rgb/camera_info',  '/oak/rgb/camera_info'),
            ('depth/image',      '/oak/stereo/image_raw'),
            # depth/camera_info → rgb camera_info: the driver publishes
            # width=0/height=0 on /oak/stereo/camera_info.  The depth image is
            # spatially aligned to the RGB sensor, so RGB intrinsics are valid.
            ('depth/camera_info', '/oak/rgb/camera_info'),
            ('imu',              '/oak/imu/data'),
            # Publish odometry on the canonical /odom topic
            ('odom',             '/odom'),
        ],
    )

    return LaunchDescription([static_tf, rgbd_odom])
