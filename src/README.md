# Duckiebot DM21 — ROS 2 Packages

This directory contains the ROS 2 packages for the Duckiebot DM21 with an OAK-D Pro camera.
The perception pipeline is split into two stages that must be launched in order.

---

## Pipeline overview

```
OAK-D Pro driver
  │
  ├─ /oak/rgb/image_raw          ─┐
  ├─ /oak/rgb/camera_info         │
  ├─ /oak/stereo/image_raw        ├──► [ vio ] rgbd_odometry
  └─ /oak/imu/data               ─┘         │
                                             │ /odom  +  TF: odom → base_link
                                             ▼
                                   [ slam ] rtabmap
                                             │
                                             ├─ /map  (nav_msgs/OccupancyGrid)
                                             └─ TF: map → odom  (loop-closure correction)
```

---

## Package: `vio` — visual-inertial odometry front-end

**Launch**

```bash
ros2 launch vio rtabmap_odom.launch.py publish_static_tf:=true
```

**What it does**

Runs `rtabmap_ros/rgbd_odometry` to estimate frame-to-frame motion from RGB-D
images and raw IMU data.  Publishes continuous odometry at ~20 Hz.

**Inputs**

| Topic | Type | Source |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | depthai driver |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | depthai driver |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` | depthai driver (depth, mm) |
| `/oak/imu/data` | `sensor_msgs/Imu` | depthai driver |

**Outputs**

| Topic / TF edge | Type | Rate |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ~20 Hz |
| TF `odom → base_link` | — | ~20 Hz |

**Known quirks handled**

- `/oak/stereo/camera_info` reports `width=0, height=0` — remapped to `/oak/rgb/camera_info`.
- IMU covariance matrices are all-zero — `Imu/IgnoreAccCovariance` and `Imu/IgnoreGyroCovariance` set to `true`.
- IMU orientation field is a dummy quaternion — `subscribe_imu_orientation: false`.

---

## Package: `slam` — SLAM back-end

**Launch**

```bash
# vio must already be running before starting slam
ros2 launch slam rtabmap_slam.launch.py

# optionally open RViz
ros2 launch slam rtabmap_slam.launch.py use_rviz:=true
```

**What it does**

Runs `rtabmap_slam/rtabmap` to build a persistent 3-D map, detect loop closures,
and publish a corrected `map → odom` TF edge that compensates for odometry drift.
Also publishes a 2-D occupancy grid for Nav2.

**Inputs**

| Topic | Type | Source |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/Image` | depthai driver |
| `/oak/rgb/camera_info` | `sensor_msgs/CameraInfo` | depthai driver |
| `/oak/stereo/image_raw` | `sensor_msgs/Image` | depthai driver (depth, mm) |
| `/odom` | `nav_msgs/Odometry` | `vio` package |

**Outputs**

| Topic / TF edge | Type | Notes |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | consumed by Nav2 |
| TF `map → odom` | — | published on loop closure |

**What it does NOT do**

- Does not estimate odometry — that is `vio`'s job.
- Does not consume IMU data — IMU is fused in the odometry front-end only.

---

## How they relate

`vio` and `slam` share the same RGB-D camera topics but have distinct roles:

- **`vio`** owns the `odom → base_link` edge and runs at full camera rate (~20 Hz).
  It gives the robot a fast, continuous pose estimate within the odometry frame.

- **`slam`** owns the `map → odom` edge and runs at 1 keyframe/sec.  When it
  detects a loop closure it shifts the entire odometry frame relative to the map,
  correcting accumulated drift without disrupting `vio`'s output.

Nav2 sees the full chain: `map → odom → base_link`.

---

## Placeholder before physical deployment

The `base_link → oak_parent_frame` static transform is an identity placeholder
published by both launch files.  Replace the `x y z yaw pitch roll` arguments
with the physically measured camera extrinsics before deploying on the robot.


## Simulation and Nav2

``` bash 
  ros2 launch moborobo_robot minimal_gazebo.launch.py
  rviz2
  ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true
```
This was tested and it works for simple waypoint navigation. You should be able to set goals in rviz2 and see the robot navigate to them. 
