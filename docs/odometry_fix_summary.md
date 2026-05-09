# Odometry Fix Summary
## moborobo_robot — Gazebo Ground Truth Odometry

---

## The Problem

During coverage navigation, the robot was driving diagonally on northward passes instead of straight. Every headland turn introduced a heading error that compounded over the run.

**Root cause: `odometry_source: world` is silently broken** in this Gazebo Classic + ROS2 Humble combination (`gazebo_ros_pkgs 3.9.0`). Despite the parameter being correctly set in the URDF, `libgazebo_ros_diff_drive.so` silently falls back to **encoder-based odometry**.

Evidence:
- Circle test (radius 0.8m, one full loop): robot physically returned to start in Gazebo, but `/odom` reported **0.58m drift**
- Odometry covariance values (`1.0e-05`) look like ground truth but are hardcoded defaults — they don't confirm which source is active
- `Param.cc:449` error in Gazebo server log: `Unable to set value [world ] for key[odometry_source]` — parameter was being rejected internally

With encoder odometry, wheel slip from the zero-friction caster wheels (`mu1: 0.0`, `mu2: 0.0`) accumulated heading error during turns. Nav2 planned paths correctly in the `odom` frame, but the `odom` frame itself was drifting relative to the real Gazebo world — causing the robot to exit turns at the wrong angle.

---

## The Fix: `ground_truth_odom` Node

Rather than rely on the broken diff drive parameter, we bypass it entirely. The node reads Gazebo's physics engine directly and republishes it as standard ROS odometry.

```
/gazebo/model_states  →  ground_truth_odom node  →  /odom topic
                                                  →  TF: odom → base_footprint
```

### How it works

`libgazebo_ros_state.so` (plugin in `gravel.world`) publishes the true Gazebo physics pose for every model at 50Hz on `/gazebo/model_states`. The node looks up `moborobo_robot` in that list and republishes its pose and twist as standard ROS odometry — both the `/odom` topic and the `odom → base_footprint` TF transform.

### Critical guard: sim_time race condition

```python
now = self.get_clock().now().to_msg()
if now.sec == 0:
    return  # clock not ready yet
```

Without this guard, the node publishes TF with timestamp `0` before `/clock` arrives from Gazebo. `tf2` silently discards transforms with timestamp `0`, causing nav2 to fail at startup with:
```
Could not find a connection between 'odom' and 'base_footprint' because they are not part of the same tree
```

### Spawn-relative coordinates

The node records the robot's first reported Gazebo position as `spawn_pose` and subtracts it from all subsequent positions. This means:
- Robot at spawn → odom `(0, 0, 0)` always, regardless of where it spawns in the Gazebo world
- Consistent with how real robot odometry works (starts at zero, accumulates from there)

---

## File Changes

### `robot_core.xacro`
Disabled the diff drive plugin's broken odometry output:
```xml
<publish_odom>false</publish_odom>
<publish_odom_tf>false</publish_odom_tf>
```
The diff drive plugin still handles velocity commands (`/cmd_vel`) and drives the wheels — only its position reporting is disabled.

### `nav2_sim.launch.py`
- Removed the `odom_base_tf` static transform publisher (was providing a frozen fake pose)
- Changed `map→odom` TF to identity `(0, 0, 0)` — all coordinate frames now coincide with Gazebo world frame, eliminating the confusing offset
- Added `ground_truth_odom` node to the launch description (starts immediately, before the 3s nav2 delay)

### `nav_launch/nav_launch/ground_truth_odom.py`
New node — see full source below.

### `nav_launch/setup.py`
Added entry point:
```python
entry_points={
    'console_scripts': [
        'ground_truth_odom = nav_launch.ground_truth_odom:main',
    ],
},
```

---

## Resulting TF Tree

```
map ──(static, identity, 10kHz)──► odom ──(ground_truth_odom, 50Hz)──► base_footprint
                                                                              │
                                                          (robot_state_publisher, 30Hz)
                                                               ├── base_link
                                                               ├── left_wheel
                                                               ├── right_wheel
                                                               ├── lidar
                                                               ├── camera_link
                                                               ├── imu_link
                                                               └── caster wheels (×4)
```

All transforms verified with `ros2 run tf2_tools view_frames`:

| Transform | Rate | Source |
|---|---|---|
| `map → odom` | 10000 Hz (static) | `static_transform_publisher` |
| `odom → base_footprint` | 50.8 Hz | `ground_truth_odom` node |
| `base_footprint → base_link → *` | 30.2 Hz | `robot_state_publisher` |

---

## Ground Truth Node — Full Source

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from gazebo_msgs.msg import ModelStates

class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.spawn_pose = None
        self.create_subscription(
            ModelStates, '/gazebo/model_states',
            self.model_states_cb, 10)
        self.get_logger().info('Ground truth odometry node started')

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index('moborobo_robot')
        except ValueError:
            return

        # Wait for sim clock to be ready
        now = self.get_clock().now().to_msg()
        if now.sec == 0:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # Record spawn position on first valid message
        if self.spawn_pose is None:
            self.spawn_pose = pose
            self.get_logger().info(
                f'Spawn position recorded: '
                f'x={pose.position.x:.3f}, y={pose.position.y:.3f}')

        # Publish TF: odom → base_footprint (spawn-relative)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = pose.position.x - self.spawn_pose.position.x
        t.transform.translation.y = pose.position.y - self.spawn_pose.position.y
        t.transform.translation.z = pose.position.z - self.spawn_pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Publish /odom topic
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = pose.position.x - self.spawn_pose.position.x
        odom.pose.pose.position.y = pose.position.y - self.spawn_pose.position.y
        odom.pose.pose.position.z = pose.position.z - self.spawn_pose.position.z
        odom.pose.pose.orientation = pose.orientation
        odom.twist.twist = twist
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = GroundTruthOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## Validation

After the fix, rerun the circle test:
```bash
# Note starting position
ros2 topic echo /odom --once | grep -A4 "position:"

# Drive one full circle (radius 0.8m, ~17 seconds)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.375}}" -r 10
# Ctrl+C after ~17 seconds, then stop robot:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once

# Check final position — should be near (0,0,0)
ros2 topic echo /odom --once | grep -A4 "position:"
```

**Expected result with ground truth:** return position within `~0.02m` of start
**Previous result with encoder odometry:** `0.58m` drift after one circle

---

## Why This Matters for Coverage Navigation

Every headland turn in the boustrophedon pattern requires the robot to arc ~180° and exit on a precise heading. With 0.58m/circle drift, each turn introduced a heading error of several degrees. Over a full coverage run (10+ turns), this compounded into the robot driving diagonally — exiting turns at the wrong angle, drifting off rows, and eventually bumping into walls.

With ground truth odometry, the robot's reported heading after every turn exactly matches its real Gazebo heading. Nav2's path follower (RegulatedPurePursuit) then commands the correct steering to follow the next row straight — eliminating diagonal drift entirely.
