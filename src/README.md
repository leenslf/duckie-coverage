# Duckiebot DM21 — ROS 2 Packages

This directory contains the ROS 2 packages for the robot.
The perception pipeline supports **OAK-D Pro** and **ZED** cameras and is split into
two stages that must be launched in order.

---

## Camera support

All launch files accept a `camera_type` argument:

| Value | Driver | Default |
|---|---|---|
| `oak` | `depthai_ros_driver_v3` | yes |
| `zed` | `zed_wrapper` | — |

Pass `camera_type:=zed` to any launch command to use the ZED instead of the OAK.

---

## Pipeline overview

**OAK-D Pro**

```
depthai_ros_driver_v3
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

**ZED**

```
zed_wrapper
  │
  ├─ /zed/zed_node/rgb/image_rect_color   ─┐
  ├─ /zed/zed_node/rgb/camera_info         │
  ├─ /zed/zed_node/depth/depth_registered  ├──► [ vio ] rgbd_odometry
  └─ /zed/zed_node/imu/data              ─┘         │
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
# OAK-D Pro (default)
ros2 launch vio rtabmap_odom.launch.py publish_static_tf:=false

# ZED
ros2 launch vio rtabmap_odom.launch.py camera_type:=zed publish_static_tf:=true
```

**What it does**

Runs `rtabmap_ros/rgbd_odometry` to estimate frame-to-frame motion from RGB-D
images and raw IMU data.  Publishes continuous odometry at ~20 Hz.

**Inputs**

| Topic | Type | OAK source | ZED source |
|---|---|---|---|
| RGB image | `sensor_msgs/Image` | `/oak/rgb/image_raw` | `/zed/zed_node/rgb/image_rect_color` |
| RGB camera info | `sensor_msgs/CameraInfo` | `/oak/rgb/camera_info` | `/zed/zed_node/rgb/camera_info` |
| Depth image | `sensor_msgs/Image` | `/oak/stereo/image_raw` | `/zed/zed_node/depth/depth_registered` |
| Depth camera info | `sensor_msgs/CameraInfo` | `/oak/rgb/camera_info` ¹ | `/zed/zed_node/depth/camera_info` |
| IMU | `sensor_msgs/Imu` | `/oak/imu/data` | `/zed/zed_node/imu/data` |

¹ OAK quirk: `/oak/stereo/camera_info` reports `width=0, height=0` — remapped to RGB camera info instead.

**Outputs**

| Topic / TF edge | Type | Rate |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | ~20 Hz |
| TF `odom → base_link` | — | ~20 Hz |

**OAK-specific quirks handled**

These parameters are applied automatically when `camera_type:=oak` and are not set for ZED:

- `/oak/stereo/camera_info` reports `width=0, height=0` — `depth/camera_info` remapped to `/oak/rgb/camera_info`.
- IMU covariance matrices are all-zero — `Imu/IgnoreAccCovariance` and `Imu/IgnoreGyroCovariance` set to `true`.
- IMU orientation field is a dummy quaternion — `subscribe_imu_orientation: false`.

---

## Package: `slam` — SLAM back-end

**Launch**

```bash
# vio must already be running before starting slam

# OAK-D Pro (default)
ros2 launch slam rtabmap_slam.launch.py publish_static_tf:=false

# ZED
ros2 launch slam rtabmap_slam.launch.py camera_type:=zed

# Optionally open RViz
ros2 launch slam rtabmap_slam.launch.py use_rviz:=true
```

**What it does**

Runs `rtabmap_slam/rtabmap` to build a persistent 3-D map, detect loop closures,
and publish a corrected `map → odom` TF edge that compensates for odometry drift.
Also publishes a 2-D occupancy grid for Nav2.

**Inputs**

| Topic | Type | OAK source | ZED source |
|---|---|---|---|
| RGB image | `sensor_msgs/Image` | `/oak/rgb/image_raw` | `/zed/zed_node/rgb/image_rect_color` |
| RGB camera info | `sensor_msgs/CameraInfo` | `/oak/rgb/camera_info` | `/zed/zed_node/rgb/camera_info` |
| Depth image | `sensor_msgs/Image` | `/oak/stereo/image_raw` | `/zed/zed_node/depth/depth_registered` |
| Depth camera info | `sensor_msgs/CameraInfo` | `/oak/rgb/camera_info` ¹ | `/zed/zed_node/depth/camera_info` |
| Odometry | `nav_msgs/Odometry` | `/odom` | `/odom` |

¹ OAK quirk: same remapping as in `vio`.

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

Both launch files publish a static `base_link → <camera_parent_frame>` identity
transform as a placeholder.  The frame name defaults to `oak_parent_frame` for OAK
and `zed_camera_link` for ZED.  Replace the `x y z yaw pitch roll` arguments with
the physically measured camera extrinsics before deploying on the robot.

The frame can be overridden explicitly:

```bash
ros2 launch vio rtabmap_odom.launch.py \
  camera_type:=zed \
  camera_parent_frame:=zed_camera_link \
  publish_static_tf:=true
```

---

## Simulation and Nav2

```bash
ros2 launch moborobo_robot minimal_gazebo.launch.py
rviz2
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true
```

This was tested and works for simple waypoint navigation.
Set goals in RViz2 and the robot will navigate to them.
