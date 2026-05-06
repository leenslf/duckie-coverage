We will work in multiple terminals to see the point cloud from the camera in rviz2:

Terminal 1: Run gazebo
```
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

# Starts Gazebo robot, /odom, /tf, /robot_description, simulated robot motion.
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

# Temporary fake camera mount transform.
# Says the OAK parent frame is 10 cm forward and 50 cm above base_link.
# Real robot needs measured/calibrated values here.
ros2 run tf2_ros static_transform_publisher \
  0.10 0.00 0.50 0 0 0 \
  base_link oak_parent_frame
```

Before running this termnial, to make sure thispointcloud is fed into nav2, make sure the `nav_params_sim.yaml` file is updated as follows:
```
observation_sources: oak_points
oak_points:
  topic: /oak/points
  data_type: PointCloud2
```

Also, change all `use_sim_time` in the file to false.



Terminal 5: Start Nav2
```
cd /home/mnt
source /opt/ros/humble/setup.bash

# Rebuild nav_launch after YAML/config edits.
colcon build --packages-select nav_launch
source install/setup.bash

# Starts Nav2.
# launch_nav2:=true actually starts Nav2 nodes.
# static_map_odom:=true publishes fake map -> odom for this sim test.
# use_sim_time:=false avoids timestamp conflict with real OAK-D camera.
ros2 launch nav_launch nav2_sim.launch.py \
  launch_nav2:=true \
  static_map_odom:=true \
  use_sim_time:=false
```

Terminal 6: Start rviz2:
```
rviz2
```

------------------

View only the OAK point cloud in RViz:

Use this when you only want to verify the camera cloud, not map integration.

In Rviz make the Fixed Frame: `oak_rgb_camera_frame`.
Then add `Add → By display type → PointCloud2`,


Configure PointCloud2:
```
Topic: /oak/points
Reliability Policy: Best Effort
Size (m): 0.03
```

-------------------

Some tests:

To verify TF:
```
ros2 run tf2_ros tf2_echo map oak_rgb_camera_optical_frame
```
Verify Nav2 is actually consuming OAK:
```
ros2 topic info /oak/points --verbose
```
You want to see subscribers:
```
/global_costmap
/local_costmap
```
==============================

Notes:
- The use-sim-time is set to false in the nav2 yaml file because we mixed simulation time (Gazebo) with a real sensor (OAK-D), and those clocks don’t match.

---------------------------

Some terminal outputs:

```
Duckie@strix:/home/mnt$ ros2 topic info /oak/points --verbose
Type: sensor_msgs/msg/PointCloud2

Publisher count: 1

Node name: PointCloudXyzNode
Node namespace: /
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: PUBLISHER
GID: 01.0f.d3.bb.ed.3e.d6.f7.00.00.00.00.00.00.14.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 2

Node name: local_costmap
Node namespace: /local_costmap
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.d3.bb.78.41.8c.f3.00.00.00.00.00.00.5c.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Node name: global_costmap
Node namespace: /global_costmap
Topic type: sensor_msgs/msg/PointCloud2
Endpoint type: SUBSCRIPTION
GID: 01.0f.d3.bb.7c.41.99.38.00.00.00.00.00.00.5b.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

```
Duckie@strix:/home/mnt$ ros2 run tf2_ros tf2_echo map oak_rgb_camera_optical_frame
[INFO] [1777839658.874923034] [tf2_echo]: Waiting for transform map ->  oak_rgb_camera_optical_frame: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[INFO] [1777839660.836458457] [tf2_echo]: Waiting for transform map ->  oak_rgb_camera_optical_frame: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
At time 177.541000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, -0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, -0.000, -89.456]
- Matrix:
  0.009 -0.000  1.000  0.120
 -1.000 -0.000  0.009  0.015
  0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
At time 177.841000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, 0.000, -89.456]
- Matrix:
  0.009 -0.000  1.000  0.120
 -1.000  0.000  0.009  0.015
 -0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
At time 178.141000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, 0.001, -89.456]
- Matrix:
  0.009 -0.000  1.000  0.120
 -1.000  0.000  0.009  0.015
 -0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
At time 178.441000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, 0.000, -89.455]
- Matrix:
  0.010 -0.000  1.000  0.120
 -1.000  0.000  0.010  0.015
 -0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
At time 178.741000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, 0.000, -89.455]
- Matrix:
  0.010 -0.000  1.000  0.120
 -1.000 -0.000  0.010  0.015
 -0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
At time 179.41000000
- Translation: [0.120, 0.015, 0.720]
- Rotation: in Quaternion (xyzw) [-0.502, 0.498, -0.498, 0.502]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.561]
- Rotation: in RPY (degree) [-90.000, 0.000, -89.454]
- Matrix:
  0.010 -0.000  1.000  0.120
 -1.000  0.000  0.010  0.015
 -0.000 -1.000 -0.000  0.720
  0.000  0.000  0.000  1.000
^C[INFO] [1777839667.466045201] [rclcpp]: signal_handler(SIGINT/SIGTERM)
```

