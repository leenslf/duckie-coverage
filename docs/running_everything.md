0. launch the driver 
```bash 
ros2 launch drive_bringup drive.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
1. launch the camera 
``` bash 
ros2 launch depthai_ros_driver_v3 driver.launch.py stereo.enableDepth:=true
```
1.5. pointcloud
```bash
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/oak/stereo/image_raw \
  -r camera_info:=/oak/stereo/camera_info \
  -r points:=/oak/points
```

2. launch static tf between baselink and camera (match that from camera.xacro)
``` bash
ros2 run tf2_ros static_transform_publisher 0.065 0 0.51 -0.5 0.5 -0.5 0.5 base_link oak_parent_frame
```
3. launch odom node
``` bash
ros2 launch vio rtabmap_odom.launch.py publish_static_tf:=false
```
4. launch slam node
``` bash
rm ~/.ros/rtabmap.db
ros2 launch slam rtabmap_slam.launch.py publish_static_tf:=false
```
5. launch nav2 node
``` bash
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=false use_sim_time:=false
```
6. on a pc with ros2 with similar ros domain id (aka 50) start rviz
``` bash
rviz2
```
7. launch the waypoint client
``` bash
ros2 run bot_nav_client navigate_to_pose_client 0.5 0.0 0.0
# or if you have a preset point in config/waypoints.yaml you can run
ros2 run bot_nav_client named_waypoint_client point1
```