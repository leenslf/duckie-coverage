Updated process for simulated ZED camera → Nav2:

Terminal 1: Start Gazebo

```bash
cd /home/mnt
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch moborobo_robot minimal_gazebo.launch.py
```

This now publishes the simulated ZED depth point cloud:

```bash
/zed2/depth/points
```


Verify ZED point cloud

```bash
ros2 topic info /zed2/depth/points
ros2 topic hz /zed2/depth/points
```

Expected type:

```bash
sensor_msgs/msg/PointCloud2
```

Updated Nav2 params

In `nav_params_sim.yaml`,


```yaml
observation_sources: zed_points
zed_points:
  topic: /zed2/depth/points
  data_type: PointCloud2
```

Terminal 2: Start Nav2

```bash
cd /home/mnt
source /opt/ros/humble/setup.bash

colcon build --packages-select nav_launch
source install/setup.bash

ros2 launch nav_launch nav2_sim.launch.py \
  launch_nav2:=true \
  static_map_odom:=true \
  use_sim_time:=true
```

Terminal 3: RViz

```bash
rviz2
```

For checking only the ZED cloud:

Fixed Frame:

```bash
camera_link_optical
```

Add `PointCloud2`:

```bash
Topic: /zed2/depth/points
Reliability Policy: Best Effort
Size: 0.03
```

For checking Nav2 integration:

Fixed Frame:

```bash
map
```
===============

Then check whether costmaps receive the ZED cloud:

```bash
ros2 topic info /zed2/depth/points --verbose
```

You want subscribers like:

```bash
/local_costmap
/global_costmap
```

TF test:

```bash
ros2 run tf2_ros tf2_echo map camera_link_optical
```

