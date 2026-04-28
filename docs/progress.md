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

