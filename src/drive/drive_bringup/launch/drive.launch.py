from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_width     = LaunchConfiguration('base_width')
    wheel_radius   = LaunchConfiguration('wheel_radius')
    reduction_rate = LaunchConfiguration('reduction_rate')

    robot_params = {
        'base_width':     base_width,
        'wheel_radius':   wheel_radius,
        'reduction_rate': reduction_rate,
    }

    return LaunchDescription([
        # Robot physical parameters — tune these for your hardware
        DeclareLaunchArgument('base_width',     default_value='0.355',
                              description='Distance between left and right wheels (m)'),
        DeclareLaunchArgument('wheel_radius',   default_value='0.1',
                              description='Wheel radius (m)'),
        DeclareLaunchArgument('reduction_rate', default_value='25.0',
                              description='Gear reduction ratio'),
        DeclareLaunchArgument('launch_inverse_kinematics', default_value='false',
                              description='Whether to launch inverse_kinematics_node (odom + TF)'),

        # cmd_vel → motor RPM commands
        Node(
            package='kinematics',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
            parameters=[robot_params],
        ),

        # motor RPMs → odometry + TF
        Node(
            package='kinematics',
            executable='inverse_kinematics_node',
            name='inverse_kinematics_node',
            output='screen',
            parameters=[robot_params],
            condition=IfCondition(LaunchConfiguration('launch_inverse_kinematics')),
        ),

        # Motor controller hardware interface
        Node(
            package='roboteq_driver',
            executable='roboteq_driver',
            name='roboteq_driver',
            output='screen',
        ),
    ])
