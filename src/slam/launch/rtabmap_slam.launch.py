#!/usr/bin/env python3
"""
RTAB-Map SLAM back-end — OAK-D Pro launch file (ROS 2 Humble).

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
  base_link → oak_parent_frame static transform (below) is an identity
  placeholder.  Replace x/y/z and yaw/pitch/roll with the physically
  measured extrinsics once the camera is mounted and calibrated.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ───────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualisation',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time — set true when replaying a bag',
    )

    publish_static_tf_arg = DeclareLaunchArgument(
        'publish_static_tf',
        default_value='false',
        description='Publish the static base_link → oak_parent_frame transform',
    )

    # ── 1. Static TF: base_link → oak_parent_frame ─────────────────────────
    # Identity transform (x y z yaw pitch roll).
    # PLACEHOLDER — replace with real extrinsics before deploying on robot.
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
        condition=IfCondition(LaunchConfiguration('publish_static_tf')),
    )

    # ── 2. RTAB-Map SLAM back-end ──────────────────────────────────────────
    rtabmap_slam = Node(
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
            # OAK-D Pro orientation field is a dummy identity quaternion —
            # do not feed it to the SLAM back-end.
            'subscribe_imu_orientation': False,

            # Synchronisation ─────────────────────────────────────────────
            # /odom (rtabmap_odom) and camera topics share the same ~20 Hz
            # source but arrive on separate ROS topics with small clock skew.
            'approx_sync': True,
            'queue_size':  30,

            # SLAM behaviour ──────────────────────────────────────────────
            # Process one keyframe per second — sufficient for a slow robot
            # and reduces CPU load significantly.
            'Rtabmap/DetectionRate':   '1',
            'Mem/IncrementalMemory':   'true',
            'Mem/InitWMWithAllNodes':  'false',

            # Occupancy grid (Nav2) ───────────────────────────────────────
            # Build a 2-D occupancy grid from depth data (sensor type 1).
            'Grid/Sensor': '1',

            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            # Node's internal name         → actual ROS topic
            ('rgb/image',         '/oak/rgb/image_raw'),
            ('rgb/camera_info',   '/oak/rgb/camera_info'),
            ('depth/image',       '/oak/stereo/image_raw'),
            # depth/camera_info → rgb camera_info: stereo camera_info
            # reports width=0/height=0; depth is spatially aligned to RGB.
            ('depth/camera_info', '/oak/rgb/camera_info'),
            ('odom',              '/odom'),
            # Publish the occupancy grid on the canonical /map topic.
            ('grid_map',          '/map'),
        ],
    )

    # ── 3. RViz2 (optional) ────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        publish_static_tf_arg,
        static_tf,
        rtabmap_slam,
        rviz,
    ])
