from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ground_truth_publisher',
            executable='ground_truth_publisher_node',
            name='ground_truth_publisher',
            output='screen',
        ),
    ])
