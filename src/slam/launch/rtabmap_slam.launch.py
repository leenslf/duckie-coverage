#!/usr/bin/env python3
"""
RTAB-Map SLAM back-end — launch file (ROS 2 Humble).

Supports OAK-D Pro and ZED.  Select with camera_type:=oak (default) or camera_type:=zed.

What this node does
───────────────────
  • Receives RGB-D frames + pre-computed /odom and builds a persistent 3-D map.
  • Detects loop closures and, when one is found, publishes a corrected
    map → odom TF edge that re-aligns the odometry drift without touching
    the odom → base_link edge owned by rtabmap_odom.
  • Publishes a nav_msgs/OccupancyGrid on /map for Nav2 consumption.

What this node does NOT do
──────────────────────────
  • It does NOT estimate odometry.  Frame-to-frame motion comes from the
    rgbd_odometry node in the 'vio' package (rtabmap_odom.launch.py).
  • It does NOT fuse IMU data.  IMU is consumed exclusively by rtabmap_odom.

Placeholder that must be updated before physical robot deployment
─────────────────────────────────────────────────────────────────
  base_link → <camera_parent_frame> static transform (below) is an identity
  placeholder.  Replace x/y/z and yaw/pitch/roll with the physically
  measured extrinsics once the camera is mounted and calibrated.

Run
───
  ros2 launch slam rtabmap_slam.launch.py                     # OAK
  ros2 launch slam rtabmap_slam.launch.py camera_type:=zed   # ZED
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
    ('odom',              '/odom'),
    ('grid_map',          '/map'),
]

_ZED_REMAPPINGS = [
    ('rgb/image',         '/zed/zed_node/rgb/image_rect_color'),
    ('rgb/camera_info',   '/zed/zed_node/rgb/camera_info'),
    ('depth/image',       '/zed/zed_node/depth/depth_registered'),
    ('depth/camera_info', '/zed/zed_node/depth/camera_info'),
    ('odom',              '/odom'),
    ('grid_map',          '/map'),
]


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    camera_parent_frame = _resolve_camera_parent_frame(
        camera_type,
        LaunchConfiguration('camera_parent_frame').perform(context),
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    publish_static_tf = LaunchConfiguration('publish_static_tf').perform(context).lower() == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context).lower() == 'true'

    is_oak = camera_type == 'oak'
    remappings = _OAK_REMAPPINGS if is_oak else _ZED_REMAPPINGS

    actions = []

    # ── 1. Static TF: base_link → <camera_parent_frame> ───────────────
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

    # ── 2. RTAB-Map SLAM back-end ──────────────────────────────────────
    actions.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            # Frames ──────────────────────────────────────────────────────
            'frame_id':      'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id':  'map',

            # Input topology ──────────────────────────────────────────────
            'subscribe_depth':           True,
            'subscribe_odom':            True,
            # OAK orientation field is a dummy identity quaternion;
            # ZED provides a real orientation estimate.
            'subscribe_imu_orientation': not is_oak,

            # Synchronisation ─────────────────────────────────────────────
            'approx_sync': True,
            'queue_size':  30,

            # SLAM behaviour ──────────────────────────────────────────────
            'Rtabmap/DetectionRate':   '1',
            'Mem/IncrementalMemory':   'true',
            'Mem/InitWMWithAllNodes':  'false',

            # Occupancy grid (Nav2) ───────────────────────────────────────
            'Grid/Sensor': '1',

            'use_sim_time': use_sim_time,
        }],
        remappings=remappings,
    ))

    # ── 3. RViz2 (optional) ────────────────────────────────────────────
    if use_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
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
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualisation',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time — set true when replaying a bag',
        ),
        DeclareLaunchArgument(
            'publish_static_tf',
            default_value='false',
            description='Publish the static base_link → <camera_parent_frame> transform',
        ),
        OpaqueFunction(function=launch_setup),
    ])
