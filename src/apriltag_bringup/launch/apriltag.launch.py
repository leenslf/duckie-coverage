#!/usr/bin/env python3
"""
AprilTag detection — launch file (ROS 2 Humble).

Supports OAK-D Pro and ZED.  Select with camera_type:=oak (default) or camera_type:=zed.

Run
───
  ros2 launch apriltag_bringup apriltag.launch.py                     # OAK
  ros2 launch apriltag_bringup apriltag.launch.py camera_type:=zed   # ZED
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_OAK_REMAPPINGS = [
    ('image_rect', '/oak/rgb/image_raw'),
    ('camera_info', '/oak/rgb/camera_info'),
]

_ZED_REMAPPINGS = [
    ('image_rect', '/zed/zed_node/rgb/image_rect_color'),
    ('camera_info', '/zed/zed_node/rgb/camera_info'),
]


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    remappings = _OAK_REMAPPINGS if camera_type == 'oak' else _ZED_REMAPPINGS

    return [
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=remappings,
            parameters=[{
                'family': '36h11',
                'size': 0.065,
                'max_hamming': 0,
                'publish_tf': True,
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value='oak',
            description='Camera to use: "oak" or "zed". Selects image and camera_info topics.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
