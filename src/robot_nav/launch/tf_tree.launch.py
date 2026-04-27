from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # tune this 
    
    # IMPORTANT: Currently the map frame and the odom frame are the same, but they differ in principle according to REP 105.
    # map -> base_link represents non-continuous pose changes of the robot w.r.t. localization algorithms.
    # odom -> base_link represents continuous pose changes of the robot w.r.t. integration of odometry data.
   
    map2odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '8.17', '--y', '15.64', '--z', '3.7', \
                         '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', \
                         '--frame-id', 'map', '--child-frame-id', 'odom']
       )
    

    
    auto_platform_transforms = [map2odom]
    return LaunchDescription(auto_platform_transforms)