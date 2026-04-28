# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from datetime import datetime
# import os


# def generate_launch_description():
#     apriltag_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('apriltag_bringup'),
#                 'launch', 'apriltag.launch.py'
#             )
#         )
#     )

#     ground_truth_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(
#                 get_package_share_directory('ground_truth_publisher'),
#                 'launch', 'ground_truth_publisher.launch.py'
#             )
#         )
#     )

#     bags_dir = os.path.join(os.path.dirname(__file__), '..', 'bags')
#     os.makedirs(bags_dir, exist_ok=True)
#     bag_output = os.path.join(bags_dir, 'vio_session_' + datetime.now().strftime('%Y%m%d_%H%M%S'))

#     bag_record = ExecuteProcess(
#         cmd=[
#             'ros2', 'bag', 'record',
#             '--output', bag_output,
#             '/oak/rgb/image_raw',
#             '/oak/rgb/camera_info',
#             '/oak/stereo/image_raw',
#             '/oak/imu/data',
#             '/odom',
#             '/ground_truth/odometry',
#             '/detections',
#             '/tf',
#             '/tf_static',
#         ],
#         output='screen',
#     )

#     return LaunchDescription([
#         apriltag_launch,
#         ground_truth_launch,
#         bag_record,
#     ])
