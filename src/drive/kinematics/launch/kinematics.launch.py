from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_width     = LaunchConfiguration('base_width')
    wheel_radius   = LaunchConfiguration('wheel_radius')
    reduction_rate = LaunchConfiguration('reduction_rate')

    return LaunchDescription([
        DeclareLaunchArgument('base_width',     default_value='0.355'),
        DeclareLaunchArgument('wheel_radius',   default_value='0.1'),
        DeclareLaunchArgument('reduction_rate', default_value='25.0'),

        Node(
            package='kinematics',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[{
                'base_width':     base_width,
                'wheel_radius':   wheel_radius,
                'reduction_rate': reduction_rate,
            }],
        ),

        Node(
            package='kinematics',
            executable='inverse_kinematics_node',
            name='inverse_kinematics_node',
            output='screen',
            parameters=[{
                'base_width':     base_width,
                'wheel_radius':   wheel_radius,
                'reduction_rate': reduction_rate,
            }],
        ),
    ])
