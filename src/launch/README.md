To record:
```bash 
  ros2 launch src/launch/record_session.launch.py
  # once the first one is working start this
  ros2 launch src/launch/record_bag.launch.py
```
To test recordings:
``` bash 
# in separate terminals
ros2 bag play oak_session/
ros2 launch slam rtabmap_slam.launch.py publish_static_tf:=true 
ros2 launch vio rtabmap_odom.launch.py  publish_static_tf:=true 
```

To remove persistent map:
``` bash 
rm ~/.ros/rtabmap.db
```

view `/map ``/mapPath` on rviz. 

To analyze:

``` bash 
python3 src/launch/dump_topic.py  

python3 src/launch/plot_trajectory.py analysis/{name}.txt {--no-altitude}
```