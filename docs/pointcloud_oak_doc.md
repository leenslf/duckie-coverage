We will work in multiple terminals to see the point cloud from the camera in rviz2:

Terminal 1: Run gazebo
```
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch moborobo_robot minimal_gazebo.launch.py
```

Terminal 2: Oak driver
```
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

# Starts OAK-D Pro driver and publishes RGB/stereo/IMU topics.
# stereo.enableDepth:=true asks the driver to run depth/RGBD pipeline.
ros2 launch depthai_ros_driver_v3 driver.launch.py stereo.enableDepth:=true
```

Terminal 3: Convert OAK depth image to point cloud
```
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

# Converts OAK stereo depth image + camera_info into PointCloud2.
# Output topic used by RViz/Nav2: /oak/points
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/oak/stereo/image_raw \
  -r camera_info:=/oak/stereo/camera_info \
  -r points:=/oak/points
```

TERMINAL 4: TF (camera → robot)
```
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tf2_ros static_transform_publisher \
  0.10 0.00 0.50 0 0 0 \
  base_link oak_rgb_camera_optical_frame
```

Before running this termnial, to make sure thispointcloud is fed into nav2, make sure the `nav_params_sim.yaml` file is updated as follows:
```
observation_sources: oak_points
oak_points:
  topic: /oak/points
  data_type: PointCloud2
```

Terminal 5: Start Nav2
```
cd /home/mnt
source /opt/ros/humble/setup.bash

colcon build --packages-select nav_launch
source install/setup.bash

# Starts Nav2.
# launch_nav2:=true actually starts Nav2 nodes.
# static_map_odom:=true publishes fake map -> odom for this sim test.
ros2 launch nav_launch nav2_sim.launch.py \
  launch_nav2:=true \
  static_map_odom:=true \
```

Terminal 6: Start rviz2:
```
rviz2
```
