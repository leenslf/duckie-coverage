#!/usr/bin/env python3
"""
RTAB-Map RGB-D Odometry — launch file (ROS 2 Humble).

Supports OAK-D Pro and ZED.  Select with camera_type:=oak (default) or camera_type:=zed.

What this file starts
─────────────────────
  1. static_transform_publisher  base_link → <camera_parent_frame>  (identity)
     Optional (publish_static_tf:=true).  Completes the TF chain so rtabmap
     can look up sensor poses relative to the robot base.

  2. rgbd_odometry (rtabmap_ros)
     Visual-inertial odometry using RGB + aligned depth + IMU.
     Publishes /odom and the TF edge  odom → base_link  at ~20 Hz.

Known OAK driver quirks handled here
─────────────────────────────────────
  • /oak/stereo/camera_info reports width=0/height=0  →  depth/camera_info
    is remapped to /oak/rgb/camera_info, which has valid 640×400 intrinsics
    and matches the aligned depth frame.

  • /oak/imu/data covariance matrices are all-zero  →  set
    Imu/IgnoreAccCovariance and Imu/IgnoreGyroCovariance to "true" so
    rtabmap accepts the data without rejecting it as invalid.

  • /oak/imu/data orientation is a placeholder identity quaternion (not
    estimated)  →  subscribe_imu_orientation=False; treat as raw 6-DOF.

These workarounds are not applied for ZED, which publishes valid covariances
and a real IMU orientation estimate.

Run
───
  ros2 launch vio rtabmap_odom.launch.py                     # OAK
  ros2 launch vio rtabmap_odom.launch.py camera_type:=zed   # ZED
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_camera_parent_frame(camera_type: str, override: str) -> str:
    if override:
        return override
    return 'oak_parent_frame' if camera_type == 'oak' else 'zed_camera_link'


_OAK_REMAPPINGS = [
    ('rgb/image',         '/oak/rgb/image_raw'),
    ('rgb/camera_info',   '/oak/rgb/camera_info'),
    ('depth/image',       '/oak/stereo/image_raw'),
    # /oak/stereo/camera_info is broken (width=0/height=0); depth is
    # spatially aligned to the RGB sensor so RGB intrinsics are valid here.
    ('depth/camera_info', '/oak/rgb/camera_info'),
    ('imu',               '/oak/imu/data'),
    ('odom',              '/odom'),
]

_ZED_REMAPPINGS = [
    ('rgb/image',         '/zed/zed_node/rgb/image_rect_color'),
    ('rgb/camera_info',   '/zed/zed_node/rgb/camera_info'),
    ('depth/image',       '/zed/zed_node/depth/depth_registered'),
    ('depth/camera_info', '/zed/zed_node/depth/camera_info'),
    ('imu',               '/zed/zed_node/imu/data'),
    ('odom',              '/odom'),
]


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    camera_parent_frame = _resolve_camera_parent_frame(
        camera_type,
        LaunchConfiguration('camera_parent_frame').perform(context),
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    publish_static_tf = LaunchConfiguration('publish_static_tf').perform(context).lower() == 'true'

    is_oak = camera_type == 'oak'
    remappings = _OAK_REMAPPINGS if is_oak else _ZED_REMAPPINGS

    params = {
        # Frames ──────────────────────────────────────────────────────
        'frame_id':      'base_link',
        'odom_frame_id': 'odom',
        'publish_tf':    True,

        # Input topology ──────────────────────────────────────────────
        'subscribe_depth':           False,
        'subscribe_imu':             True,
        # OAK does not estimate IMU orientation; ZED does.
        'subscribe_imu_orientation': not is_oak,

        # Synchronisation ─────────────────────────────────────────────
        'approx_sync':              True,
        'approx_sync_max_interval': 0.0,
        'queue_size':               30,

        # Continuous output ────────────────────────────────────────────
        'Odom/LinearUpdate':  '0',
        'Odom/AngularUpdate': '0',

        # Odometry strategy ───────────────────────────────────────────
        'Odom/Strategy': '0',

        'use_sim_time': use_sim_time,
    }

    # OAK publishes all-zero IMU covariance matrices; without these flags
    # rtabmap discards every IMU message as invalid.
    if is_oak:
        params['Imu/IgnoreAccCovariance'] = 'true'
        params['Imu/IgnoreGyroCovariance'] = 'true'

    actions = []

    # ── 1. Static TF: base_link → <camera_parent_frame> ──────────────
    if publish_static_tf:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_parent',
            arguments=[
                '0', '0', '0',       # translation  x y z  (metres)
                '0', '0', '0',       # rotation     yaw pitch roll  (radians)
                'base_link',
                camera_parent_frame,
            ],
            output='screen',
        ))

    # ── 2. RGB-D Odometry node ────────────────────────────────────────
    actions.append(Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rtabmap_odom',
        output='screen',
        parameters=[params],
        remappings=remappings,
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value='oak',
            description='Camera to use: "oak" (depthai_ros_driver_v3) or "zed" (zed_wrapper)',
        ),
        DeclareLaunchArgument(
            'camera_parent_frame',
            default_value='',
            description=(
                'TF child frame for the base_link → camera static transform. '
                'Defaults to "oak_parent_frame" for OAK and "zed_camera_link" for ZED.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='false',
            description='Publish the static base_link → <camera_parent_frame> transform',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time — set true when replaying a bag',
        ),
        OpaqueFunction(function=launch_setup),
    ])
