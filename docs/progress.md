# Progress

## Follow this format

[Day - Date] [Person's name]
- Brief descriptions of something you added
- Brief descriptions of something you added

## Log
### Monday 27-04-2026 | Leen 
- Started the workspace and documentation
- Added `vio` package to publish odom from rgb image, depth image, rgb camera_info (used for both), imu.

### Tuesday 28-04-2026 | Leen 
- Added `slam` package to perform loop closure using RTAB-Map SLAM
- Added `apriltag_bringup` package for OAK-D Pro AprilTag detection
- Added `ground_truth_publisher` package to republish pose from `apriltag_bringup` as `/ground_truth/odometry`'. Looks like it's not getting published yet.
- Added `src/launch/record_session.launch.py` to bring up AprilTag + ground truth nodes and record a ROS 2 bag. Doesn't launch anything correctly yet, keeping it for the spirit.
- Added `scripts/evaluate.py` to evaluate individual experiments discussed in docs/trajectories.md

### Wednesday 29-04-2026 | Leen 
- removed `ground_truth_publisher` package and `scripts/evaluate.py`. Simpler approach might be used to benchmark. `src/launch/record_session.launch.py` only starts bag recording. 

### Thursday 30-04-2026 | Leen 
- Copied `moborobo_robot` package from ig-lio codebase. We can use odom produced by `libgazebo_ros_diff_drive.so` plugin in the simulation for now. We can use existing lidar to mock depth sensor. In reality odom will be obtained from OAK's VIO and depth will obtained from stero matching stream. Camera should be added to the TF tree. Check [to-do](docs/TODO.md).
- Copied working `nav_launch` packge from ig-lio codebase. This package was tested in simulation before. Some changes need to be made to make it work immediately in the sim, like changing `odom_topic` etc.
- Added `roboteq_driver` package, which is a ROS2 migrated package from previous group's work. Not tested yet.

### Sunday 03-05-2026 | Hamza
- Can now view the point cloud from the oak camera check the [documentation](pointcloud_oak_doc.md) to see how to run it and look at some terminal outputs.

### Wednesday 06-05-2026 | Leen
- Recorded some bag data. Added some tools under `src/launch` to easily help record and analyze this data. However the experiments I recorded weren't exactly controlled. 
- Added `/map` to `global_costmap` in nav2 params.