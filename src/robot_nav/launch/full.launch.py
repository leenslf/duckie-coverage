from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

# ros2 launch robot_nav nav2_sim.launch.py launch_nav2:=true launch_rviz:=true launch_coverage:=true launch__state:=true

def generate_launch_description():
    
    params_path = os.path.join(get_package_share_directory('robot_nav'), 'config', 'nav2_params.yaml')
    
    launch_nav2_declaration = DeclareLaunchArgument(
        "launch_nav2",
        default_value="false",
        description='whether to start nav2 or not'
    )
    launch_nav2 = LaunchConfiguration("launch_nav2")
    
    # add this
    launch_coverage_test_declaration = DeclareLaunchArgument( 
        "launch_coverage",
        default_value="false", 
        description='whether to start coverage test or not'
    )
    launch_coverage = LaunchConfiguration("launch_coverage")
    
    # New declaration argument for  state publisher (add this)
    launch__state_declaration = DeclareLaunchArgument(
        "launch__state",
        default_value="true", 
        description='whether to start  state publisher or not'
    )
    launch__state = LaunchConfiguration("launch__state")
    
        
    create_tf_tree = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('robot_nav'), 'launch', 'tf_tree.launch.launch.py')
        ])
    )
        
    nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_nav'), 'launch', 'nav2.launch.py')),
            launch_arguments={
                'namespace': '',
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': params_path,
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': 'info'
            }.items(),
            condition=IfCondition(launch_nav2)
        )
    
    nav2 = TimerAction(
        period=3.0,
        actions=[nav2_launch]
    )
    
    # Coverage test node (add this, this basically has the input of area we want to plan)
    coverage_test_node = Node(
        package='robot_nav',
        executable='tester.py',
        name='coverage_tester',
        output='screen',
        condition=IfCondition(launch_coverage)
    )
    
    coverage_test = TimerAction(
        period=8.0,  # Start coverage test after navigation stack is fully ready
        actions=[coverage_test_node]
    )
        
    #  state publisher node with conditional launch
    _state_publisher = Node(
        package='_state',
        executable='_state_publisher',  
        name='_state_publisher',
        output='screen',
        condition=IfCondition(launch__state)  # Add condition to control when it launches
    )
    
    return LaunchDescription([
        # Declaration arguments
        launch_nav2_declaration,
        launch_coverage_test_declaration,
        launch__state_declaration,  # Add the new declaration
        
        # Nodes and actions
        create_tf_tree,
        _state_publisher, 
        nav2,
        coverage_test,
    ])