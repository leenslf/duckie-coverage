from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    params_path = os.path.join(get_package_share_directory('nav_launch'), 'config', 'nav_params_sim.yaml')

    launch_nav2_declaration = DeclareLaunchArgument(
        "launch_nav2",
        default_value="false",
        description='whether to start nav2 or not'
    )
    launch_nav2 = LaunchConfiguration("launch_nav2")

    static_map_odom_declaration = DeclareLaunchArgument(
        "static_map_odom",
        default_value="false",
        description='publish a static identity transform from map to odom'
    )
    static_map_odom = LaunchConfiguration("static_map_odom")

    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(static_map_odom)
    )

 
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav_launch'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'autostart': 'true',
                'use_sim_time': 'true', 
                'params_file': params_path,
            }.items(),
            condition=IfCondition(launch_nav2)
        )

    nav2 = TimerAction(
        period=3.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        launch_nav2_declaration,
        static_map_odom_declaration,
        map_odom_tf,
        nav2
    ])
