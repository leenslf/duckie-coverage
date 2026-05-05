## Finished tasks [] → [x]
[] 1. mount the camera on the robot

[] 2. if moborobot is to be used, the tf tree should be edited to include the camera. Check the current [tf tree](src/moborobo_robot/data/frames_2026-04-30_09.27.53.pdf). Currently RTAB-Map Odom node publishes a static tf base_link → oak_parent_frame. 
The correct order should be:
     
    - map → odom (publish by RTAB-Map SLAM node)
    - odom → basefootprint or baselink (publish by RTAB-Map Odom node)
    - There needs to be a tf from baselink to OAK parent frame, this depends on task 1, with that being done the tf tree's camera link should refer to OAK-D sensor. LIDAR link should be dropped because we don't use lidar for this project. 
    

[x] 3. obtain pointcloud from OAK-D's stereo matching stream

[x] 4. Compile moborobot's driver manager library and test it. 

[x] 5. `coverage_server` should be added to `src/nav_launch` and tuned. Take inspiration from messy `src/robot_nav`. Follow [instructions here](docs/coverage.md) to start with setting up coverage. 

[] 6. tune coverage parameters. currently the set up works but not well. 
