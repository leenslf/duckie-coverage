#!/usr/bin/env python3
"""
Record Bag — ros2 bag recorder for the OAK-D Pro session (ROS 2 Humble).

What this file starts
─────────────────────
  ros2 bag record — records all key OAK-D topics to a bag file.

Prerequisites
─────────────
  The sensor stack must already be running:
    ros2 launch src/launch/record_session.launch.py

Run
───
  ros2 launch src/launch/record_bag.launch.py
  ros2 launch src/launch/record_bag.launch.py bag_prefix:=my_run_01

Topics recorded
───────────────
  /oak/rgb/image_raw
  /oak/rgb/camera_info
  /oak/stereo/image_raw
  /oak/imu/data
  /tf
  /tf_static
  /oak/points
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    bag_prefix_arg = DeclareLaunchArgument(
        'bag_prefix',
        default_value='oak_session',
        description='Output bag name (written to the bags/ directory)',
    )

    log_start = LogInfo(msg='[record_bag] ── Starting bag recorder ──')
    log_pending = LogInfo(msg='[record_bag] Waiting 3 s for topics to become active …')

    bag_recorder_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', PathJoinSubstitution(['bags', LaunchConfiguration('bag_prefix')]),
            '/oak/rgb/image_raw',
            '/oak/rgb/camera_info',
            '/oak/stereo/image_raw',
            '/oak/imu/data',
            '/tf',
            '/tf_static',
            '/oak/points',
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

    return LaunchDescription([
        bag_prefix_arg,
        log_start,
        log_pending,
        bag_record,
        bag_started_handler,
        bag_exit_handler,
    ])


if __name__ == '__main__':
    print(
        '\n[record_bag] This is a ROS 2 launch file — run it with:\n'
        '  ros2 launch src/launch/record_bag.launch.py\n'
        '  ros2 launch src/launch/record_bag.launch.py bag_prefix:=my_run_01\n'
    )
