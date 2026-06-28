from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.3',
        description='Robot wheel separation in meters',
    )

    cmd_vel_to_wheels_node = Node(
        package='nav_launch',
        executable='cmd_vel_to_wheels',
        parameters=[{'wheel_base': LaunchConfiguration('wheel_base')}],
    )

    return LaunchDescription([wheel_base_arg, cmd_vel_to_wheels_node])
