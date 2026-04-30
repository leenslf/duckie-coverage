from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/oak/rgb/image_raw'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.065,
                'max_hamming': 0,
                'publish_tf': True,
            }],
        ),
    ])
