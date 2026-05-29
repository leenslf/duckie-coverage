# CLAUDE.md — moborobo_robot Coverage Navigation

## What This Project Is

ROS 2 (Humble) differential-drive robot (`moborobo_robot`) running boustrophedon
coverage navigation in Gazebo simulation using Nav2 + opennav_coverage (Fields2Cover).

## How to Launch

```bash
# Terminal 1 — Gazebo + robot
ros2 launch moborobo_robot minimal_gazebo.launch.py

# Terminal 2 — Nav2 + ground truth odom + map→odom TF
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true

# Terminal 3 — Run coverage
ros2 run opennav_coverage_demo demo_coverage
```

Build after source changes to moborobo_robot (launch files are NOT symlinked):
```bash
colcon build --packages-select moborobo_robot --symlink-install
# Then manually copy any launch/world files to install/ or just rebuild
cp src/moborobo_robot/launch/minimal_gazebo.launch.py \
   install/moborobo_robot/share/moborobo_robot/launch/
cp src/moborobo_robot/world/gravel_featureless.world \
   install/moborobo_robot/share/moborobo_robot/world/
```

nav_launch yaml configs ARE symlinked — edit source, restart nav2 only (no rebuild).

---

## The Odometry Problem and Fix

### Problem
`odometry_source: world` in `libgazebo_ros_diff_drive.so` is **silently broken**
in this Gazebo/ROS2 combination. It falls back to encoder odometry, which drifts
~0.58m per circle due to caster wheel slip. This causes Nav2 to navigate in a
drifting odom frame → robot drives diagonally during coverage.

### Fix: ground_truth_odom node
`src/moborobo_robot/moborobo_robot/ground_truth_odom.py`

Reads the robot's true physics pose from `/gazebo/model_states` (published by
`libgazebo_ros_state.so` at 50 Hz in the world file) and republishes it as the
`odom → base_footprint` TF and `/odom` topic.

**Critical guard — sim_time race condition:**
```python
now = self.get_clock().now().to_msg()
if now.sec == 0:
    return  # clock not ready; TF with timestamp 0 is silently dropped by tf2
```
Without this, the node spams TF with timestamp 0 before `/clock` arrives, causing
"cannot find transform odom→base_footprint" and nav2 startup failures.

**Supporting changes that go with this fix:**
- `robot_core.xacro`: `publish_odom: false`, `publish_odom_tf: false` — disables
  the broken diff-drive odom output to prevent conflicts
- `nav2_sim.launch.py`: launches `ground_truth_odom` node, map→odom TF = identity
- `gravel_featureless.world`: has `libgazebo_ros_state.so` plugin (required)

### TF Tree
```
map ──(static, identity)──► odom ──(ground_truth_odom, 50Hz)──► base_footprint
                                                                       │
                                                       (robot_state_publisher)
                                                            ├── base_link
                                                            ├── left_wheel / right_wheel
                                                            └── caster wheels
```

---

## Coordinate System

**map = odom = Gazebo world frame** (map→odom TF is identity, zero offset).
Any coordinate in `demo_coverage.py`, `minimal_gazebo.launch.py`, or wall poses
is in the same frame — no conversion needed.

```
        +y (North)
         ↑
         │   All field coordinates are negative y
─────────┼──────────► +x (East)
         │   because the field sits south of the Gazebo origin
```

### Reading Wall Poses
`<pose>Cx Cy 1.0 0 0 0</pose>` + `<size>Sx Sy 2</size>`:
- Wall occupies x ∈ [Cx−Sx/2, Cx+Sx/2], y ∈ [Cy−Sy/2, Cy+Sy/2]
- Inner face (toward field): N wall → y = Cy − 0.1; S wall → y = Cy + 0.1;
  E wall → x = Cx − 0.1; W wall → x = Cx + 0.1

### Clearance Rule
`obstacle_max_range = 1.5m`. Wall inner face must be ≥ 1.5m from the field
boundary **at the sides where Dubins turns happen**.

Current setup uses E-W rows (turns at east/west headlands):
- East/west walls need ≥ 1.5m clearance from field east/west boundaries
- North/south walls need ≥ 1.5m clearance from the **northernmost/southernmost
  row center** (not the field boundary), because turns don't happen there

### Current World Geometry (Setup A — baseline)
```
Walls:   west inner x=−0.1  east inner x=9.0  north inner y=−3.3  south inner y=−8.8
Field:   x ∈ [1.45, 7.45],  y ∈ [−8.8, −3.3]
Spawn:   (2.3, −3.5, 0.2)  yaw=−1.5708 (facing south)
```

Headland = 0.85m → swath x∈[2.3,6.6] y∈[−7.95,−4.15] = 8 E-W rows @ 0.47m
Row centres (N→S): −4.4, −4.87, −5.34, −5.81, −6.28, −6.75, −7.22, −7.69

See src/docs/coverage_setups.md for Setup B (OEZ) and Setup C (dynamic obstacle) configurations.

### Recipe for a New World
1. Place walls, note each center pose (Cx, Cy) and thickness (0.2m)
2. Inner face = Cx ± 0.1 or Cy ± 0.1
3. Subtract 1.5m from inner faces at turn sides → field boundary
4. Write field polygon from those boundaries in `demo_coverage.py`
5. Spawn robot at `(field_west + headland, first_row_y)` to land at coverage start
   where `first_row_y = field_north − headland − operation_width/2`

---

## Key Parameters (`nav_params_sim.yaml`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `operation_width` | 0.47m | row spacing = robot body width |
| `default_headland_width` | 0.85m | must be ≥ min_turning_radius |
| `min_turning_radius` | 0.8m | larger = gentler Dubins arc |
| `default_swath_angle` | 1.5708 (π/2) | produces E-W rows in this setup |
| `default_path_type` | REEDS_SHEPP | Allows reversing; tighter headland turns than Dubins |
| `obstacle_max_range` | 1.5m | reduced from 3m to avoid wall detection during turns |

`origin_x`/`origin_y` in local costmap: **irrelevant** — `rolling_window: true`
means the costmap always re-centers on the robot; these initial values are overridden.

---

## Key Files

| File | Purpose |
|------|---------|
| `moborobo_robot/moborobo_robot/ground_truth_odom.py` | True odom from Gazebo physics |
| `moborobo_robot/description/robot_core.xacro` | Robot URDF + Gazebo diff-drive plugin |
| `moborobo_robot/world/gravel_featureless.world` | Gazebo world with walls + ros_state plugin |
| `moborobo_robot/launch/minimal_gazebo.launch.py` | Launches Gazebo + spawns robot |
| `nav_launch/launch/nav2_sim.launch.py` | Launches Nav2 + ground_truth_odom + TF |
| `nav_launch/config/nav_params_sim.yaml` | All Nav2 + coverage parameters |
| `opennav_coverage/opennav_coverage_demo/opennav_coverage_demo/demo_coverage.py` | Coverage field polygon + action client |
