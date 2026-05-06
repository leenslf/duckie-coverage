#!/usr/bin/env python3
"""
Record Bag — ros2 bag recorder for the camera session (ROS 2 Humble).

Supports OAK-D Pro and ZED.  Topics are selected automatically based on camera_type.

What this file starts
─────────────────────
  ros2 bag record — records all key camera topics to a bag file.

Prerequisites
─────────────
  The sensor stack must already be running:
    ros2 launch src/launch/record_session.launch.py [camera_type:=oak|zed]

Run
───
  # OAK-D Pro (default)
  ros2 launch src/launch/record_bag.launch.py
  ros2 launch src/launch/record_bag.launch.py bag_prefix:=my_run_01

  # ZED
  ros2 launch src/launch/record_bag.launch.py camera_type:=zed
  ros2 launch src/launch/record_bag.launch.py camera_type:=zed bag_prefix:=zed_run_01

Topics recorded — OAK
──────────────────────
  /oak/rgb/image_raw
  /oak/rgb/camera_info
  /oak/stereo/image_raw
  /oak/imu/data
  /tf
  /tf_static
  /oak/points

Topics recorded — ZED
──────────────────────
  /zed/zed_node/rgb/image_rect_color
  /zed/zed_node/rgb/camera_info
  /zed/zed_node/depth/depth_registered
  /zed/zed_node/imu/data
  /tf
  /tf_static
  /zed/zed_node/point_cloud/cloud_registered
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


_OAK_TOPICS = [
    '/oak/rgb/image_raw',
    '/oak/rgb/camera_info',
    '/oak/stereo/image_raw',
    '/oak/imu/data',
    '/tf',
    '/tf_static',
    '/oak/points',
]

_ZED_TOPICS = [
    '/zed/zed_node/rgb/image_rect_color',
    '/zed/zed_node/rgb/camera_info',
    '/zed/zed_node/depth/depth_registered',
    '/zed/zed_node/imu/data',
    '/tf',
    '/tf_static',
    '/zed/zed_node/point_cloud/cloud_registered',
]


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    bag_prefix = LaunchConfiguration('bag_prefix').perform(context)

    topics = _OAK_TOPICS if camera_type == 'oak' else _ZED_TOPICS

    bag_recorder_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', PathJoinSubstitution(['bags', bag_prefix]),
            *topics,
        ],
        output='screen',
    )

    bag_record = TimerAction(
        period=3.0,
        actions=[bag_recorder_process],
    )

    bag_started_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=bag_recorder_process,
            on_start=[LogInfo(msg='[record_bag] Bag recorder started — writing to bags/<bag_prefix>.')],
        )
    )
    bag_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_recorder_process,
            on_exit=[LogInfo(msg='[record_bag] Bag recorder exited.')],
        )
    )

    return [
        LogInfo(msg=f'[record_bag] ── Starting bag recorder ({camera_type.upper()}) ──'),
        LogInfo(msg='[record_bag] Waiting 3 s for topics to become active …'),
        bag_record,
        bag_started_handler,
        bag_exit_handler,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value='oak',
            description='Camera in use: "oak" or "zed". Selects which topics to record.',
        ),
        DeclareLaunchArgument(
            'bag_prefix',
            default_value='camera_session',
            description='Output bag name (written to the bags/ directory)',
        ),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    print(
        '\n[record_bag] This is a ROS 2 launch file — run it with:\n'
        '  ros2 launch src/launch/record_bag.launch.py\n'
        '  ros2 launch src/launch/record_bag.launch.py camera_type:=zed\n'
        '  ros2 launch src/launch/record_bag.launch.py bag_prefix:=my_run_01\n'
    )
