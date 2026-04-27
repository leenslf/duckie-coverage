# Launch file for Nav2 waypoint navigation on the Duckietown DM21.
#
# What this file starts:
#   1. static_transform_publisher  — base_link → oak_parent_frame (identity placeholder)
#   2. depthimage_to_laserscan     — OAK-D depth image → /scan for Nav2 obstacle layers
#   3. Nav2 stack (controller, planner, behavior, bt_navigator, waypoint_follower,
#                  smoother, velocity_smoother) + lifecycle_manager
#
# What this file does NOT start (already running upstream):
#   - rtabmap_odom  (publishes /odom, odom→base_link TF)
#   - RTAB-Map SLAM (publishes /map, map→odom TF)
#   - depthai_ros_driver (publishes /oak/* topics and OAK-D TF subtree)

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch argument: params_file ─────────────────────────────────────────
    # Defaults to the nav2_params.yaml inside this package's share directory.
    # Override at runtime:  ros2 launch waypoint_nav waypoint_nav.launch.py \
    #                           params_file:=/path/to/custom_params.yaml
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('waypoint_nav'), 'config', 'nav2_params.yaml']
        ),
        description='Full path to the Nav2 parameter YAML file',
    )

    params_file = LaunchConfiguration('params_file')

    # ── Environment: line-buffered logging for clean terminal output ──────────
    stdout_linebuf = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    # ── Static TF: base_link → oak_parent_frame ──────────────────────────────
    # PLACEHOLDER (identity transform): update --x/--y/--z/--yaw/--pitch/--roll
    # once the OAK-D camera mounting position is physically measured on the robot.
    static_tf_base_to_oak = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_oak_parent_frame',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'oak_parent_frame',
        ],
        output='screen',
    )

    # ── depthimage_to_laserscan ───────────────────────────────────────────────
    # Converts /oak/stereo/image_raw (16UC1 depth, mm) to a LaserScan on /scan.
    # /oak/rgb/camera_info is used instead of /oak/stereo/camera_info because
    # the stereo info topic reports width=0/height=0 and cannot be used.
    depthimage_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_height': 1,               # single center row — reduces depth noise
            'output_frame': 'oak_rgb_camera_frame',
            'range_min': 0.3,               # OAK-D minimum reliable depth in meters
            'range_max': 5.0,
        }],
        remappings=[
            ('image', '/oak/stereo/image_raw'),
            ('camera_info', '/oak/rgb/camera_info'),
            ('scan', '/scan'),
        ],
        output='screen',
    )

    # ── TF remappings shared by all Nav2 lifecycle nodes ─────────────────────
    # Required so that nodes running without a namespace still resolve /tf and
    # /tf_static correctly in case a namespace is added in the future.
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ── Nav2 lifecycle nodes ──────────────────────────────────────────────────
    # The lifecycle_manager activates these nodes in order after launch.
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=tf_remappings,
    )

    # Lifecycle manager: brings Nav2 nodes through configure → activate at startup.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes,
        }],
    )

    # ── Assemble launch description ───────────────────────────────────────────
    ld = LaunchDescription()

    ld.add_action(stdout_linebuf)
    ld.add_action(declare_params_file)

    # Non-lifecycle infrastructure
    ld.add_action(static_tf_base_to_oak)
    ld.add_action(depthimage_to_laserscan)

    # Nav2 lifecycle nodes
    ld.add_action(controller_server)
    ld.add_action(smoother_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(velocity_smoother)

    # Lifecycle manager last — activates nodes after they are all registered
    ld.add_action(lifecycle_manager)

    return ld
