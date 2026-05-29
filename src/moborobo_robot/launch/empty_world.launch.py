# moborobo_robot/launch/empty_world.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('moborobo_robot')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': '',          # empty world
            'gui': 'true',
            'server': 'true',
            'verbose': 'true'
        }.items()
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg, 'launch', 'robot_state_publisher.launch.py')
        ])
    )

    spawn = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-topic', '/robot_description',
                    '-entity', 'moborobo_robot',
                    '-x', '0.0', '-y', '0.0', '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([gazebo, rsp, spawn])