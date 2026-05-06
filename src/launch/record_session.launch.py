#!/usr/bin/env python3
"""
Record Session — camera sensor stack (ROS 2 Humble).

Supports OAK-D Pro (depthai_ros_driver_v3) and ZED (zed_wrapper).
Select the camera at launch time with camera_type:=oak (default) or camera_type:=zed.

What this file starts
─────────────────────
  1. Camera driver
       oak: depthai_ros_driver_v3  (stereo depth enabled)
       zed: zed_wrapper/zed_camera.launch.py
  2. depth_image_proc/point_cloud_xyz_node  — depth → /oak/points  (OAK only;
     ZED publishes its own point cloud at /zed/zed_node/point_cloud/cloud_registered)
  3. tf2_ros/static_transform_publisher  — base_link → <camera_parent_frame>

Known OAK driver quirks handled here
─────────────────────────────────────
  • /oak/stereo/camera_info reports width=0/height=0 → remap camera_info in
    point_cloud_xyz_node to /oak/rgb/camera_info, which has valid 640×400
    intrinsics and matches the aligned depth frame.

Run
───
  # OAK-D Pro (default)
  ros2 launch src/launch/record_session.launch.py

  # ZED
  ros2 launch src/launch/record_session.launch.py camera_type:=zed

  # ZED with a different model
  ros2 launch src/launch/record_session.launch.py camera_type:=zed zed_camera_model:=zed2i

  # Skip individual nodes
  ros2 launch src/launch/record_session.launch.py launch_driver:=false
  ros2 launch src/launch/record_session.launch.py launch_point_cloud:=false
  ros2 launch src/launch/record_session.launch.py launch_static_tf:=false

Then start the bag recorder in a second terminal:
  ros2 launch src/launch/record_bag.launch.py camera_type:=oak   # or zed
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _resolve_camera_parent_frame(camera_type: str, override: str) -> str:
    if override:
        return override
    return 'oak_parent_frame' if camera_type == 'oak' else 'zed_camera_link'


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    camera_parent_frame = _resolve_camera_parent_frame(
        camera_type,
        LaunchConfiguration('camera_parent_frame').perform(context),
    )
    zed_camera_model = LaunchConfiguration('zed_camera_model').perform(context)
    launch_driver = LaunchConfiguration('launch_driver').perform(context).lower() == 'true'
    launch_point_cloud = LaunchConfiguration('launch_point_cloud').perform(context).lower() == 'true'
    launch_static_tf = LaunchConfiguration('launch_static_tf').perform(context).lower() == 'true'

    actions = [LogInfo(msg=f'[record_session] ── Starting {camera_type.upper()} sensor stack ──')]

    # ── 1. Camera driver ───────────────────────────────────────────────────
    if launch_driver:
        if camera_type == 'oak':
            actions += [
                LogInfo(msg='[record_session] Step 1: launching depthai_ros_driver_v3 …'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('depthai_ros_driver_v3'),
                        '/launch/driver.launch.py',
                    ]),
                    launch_arguments={'stereo.enableDepth': 'true'}.items(),
                ),
            ]
        else:  # zed
            actions += [
                LogInfo(msg=f'[record_session] Step 1: launching zed_wrapper ({zed_camera_model}) …'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('zed_wrapper'),
                        '/launch/zed_camera.launch.py',
                    ]),
                    launch_arguments={'camera_model': zed_camera_model}.items(),
                ),
            ]

    # ── 2. Point-cloud node (OAK only) ─────────────────────────────────────
    # ZED publishes /zed/zed_node/point_cloud/cloud_registered natively.
    # For OAK: /oak/stereo/camera_info is broken (width=0/height=0), so
    # camera_info is remapped to /oak/rgb/camera_info (depth is RGB-aligned).
    if launch_point_cloud and camera_type == 'oak':
        actions += [
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
        ]

    # ── 3. Static TF: base_link → <camera_parent_frame> ───────────────────
    # Identity placeholder — replace with real extrinsics once calibrated.
    if launch_static_tf:
        actions += [
            LogInfo(msg=f'[record_session] Step 3: publishing static TF base_link → {camera_parent_frame} …'),
            Node(
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
            ),
        ]

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
            'zed_camera_model',
            default_value='zed2',
            description='ZED camera model passed to zed_camera.launch.py (e.g. zed2, zed2i, zedx). Ignored for OAK.',
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            description='Launch the camera driver',
        ),
        DeclareLaunchArgument(
            'launch_point_cloud',
            default_value='true',
            description=(
                'Launch depth_image_proc/point_cloud_xyz_node. '
                'OAK only — ZED publishes its own point cloud natively.'
            ),
        ),
        DeclareLaunchArgument(
            'launch_static_tf',
            default_value='true',
            description='Publish static TF base_link → <camera_parent_frame>',
        ),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    print(
        '\n[record_session] This is a ROS 2 launch file — run it with:\n'
        '  ros2 launch src/launch/record_session.launch.py\n'
        '  ros2 launch src/launch/record_session.launch.py camera_type:=zed\n'
    )
