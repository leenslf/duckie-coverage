# Coverage Navigation Setups

Three configurations are documented here:
- **Setup A** — single 8-row field, no obstacle (baseline)
- **Setup B** — OEZ experiment: 3-row north + obstacle + 3-row south
- **Setup C** — Dynamic Obstacle Response: Setup A + moving obstacle node

Both setups A and B share the same launch procedure and Nav2 parameters.
Setup C runs on top of Setup A with one extra terminal.

---

## Launch Procedure (both setups)

```bash
# Terminal 1 — Gazebo + robot
ros2 launch moborobo_robot minimal_gazebo.launch.py

# Terminal 2 — Nav2 stack
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true

# Terminal 3 — run coverage
ros2 run opennav_coverage_demo demo_coverage
```

Robot spawns at **(2.3, −3.5, 0.2) yaw = −1.5708 rad** (facing south) in both setups.

After editing `demo_coverage.py`, rebuild before running:
```bash
colcon build --packages-select opennav_coverage_demo
```

The world file (`gravel_featureless.world`) is symlinked in install — changes take effect on next Gazebo launch without rebuilding.

---

## Shared Nav2 Parameters (`nav_params_sim.yaml`)

These are unchanged between setups.

| Parameter | Value | Notes |
|-----------|-------|-------|
| `operation_width` | 0.47 m | row spacing = robot body width |
| `default_headland_width` | 0.85 m | must be ≥ min_turning_radius |
| `min_turning_radius` | 0.8 m | Reeds-Shepp arc radius |
| `default_swath_angle` | 1.5708 (π/2) | produces E-W rows |
| `default_path_type` | REEDS_SHEPP | allows reversing at headlands |
| `obstacle_max_range` | 1.5 m | local costmap LiDAR marking range |
| `xy_goal_tolerance` | 0.10 m | tight enough to prevent early success |
| `yaw_goal_tolerance` | 0.5 rad | |

---

## Coordinate System

`map = odom = Gazebo world frame` (map→odom TF is identity).

```
        +y (North)
         ↑
─────────┼──────────► +x (East)
         │   field sits at negative y (south of origin)
```

Wall geometry: `<pose>Cx Cy 1.0</pose>` + `<size>Sx Sy 2</size>`
- East wall inner face = Cx − Sx/2
- West wall inner face = Cx + Sx/2
- North wall inner face = Cy − Sy/2
- South wall inner face = Cy + Sy/2

---

## Setup A — Baseline (single field, 8 rows)

### World geometry

```
Walls (inner faces):
  North  y = −3.3    South  y = −8.8
  East   x =  9.0    West   x = −0.1

Field:   x ∈ [1.45, 7.45],  y ∈ [−8.8, −3.3]
Spawn:   (2.3, −3.5, 0.2)  yaw = −1.5708

Swath area (after 0.85 m headland each side):
  x ∈ [2.3, 6.6],  y ∈ [−7.95, −4.15]  →  3.8 m tall  →  8 rows

Row centres (N → S):  −4.4, −4.87, −5.34, −5.81, −6.28, −6.75, −7.22, −7.69
```

### `gravel_featureless.world` wall models

```xml
<!-- North wall: inner face y = −3.3 -->
<model name="wall_north">
  <static>true</static>
  <pose>4.45 -3.2 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>9.5 0.2 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>9.5 0.2 2</size></box></geometry></visual>
  </link>
</model>

<!-- South wall: inner face y = −8.8 -->
<model name="wall_south">
  <static>true</static>
  <pose>4.45 -8.9 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>9.5 0.2 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>9.5 0.2 2</size></box></geometry></visual>
  </link>
</model>

<!-- East wall: inner face x = 9.0  (1.55 m east of field_east 7.45) -->
<model name="wall_east">
  <static>true</static>
  <pose>9.1 -6.05 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>0.2 5.9 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>0.2 5.9 2</size></box></geometry></visual>
  </link>
</model>

<!-- West wall: inner face x = −0.1  (1.55 m west of field_west 1.45) -->
<model name="wall_west">
  <static>true</static>
  <pose>-0.2 -6.05 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>0.2 5.9 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>0.2 5.9 2</size></box></geometry></visual>
  </link>
</model>
```

No obstacle model is present.

Camera (GUI):
```xml
<pose frame=''>4.45 -17.0 14.0 0.0 0.8 1.57</pose>
```

### `demo_coverage.py` `main()` body

```python
field = [[1.45, -8.8], [1.45, -3.3], [7.45, -3.3], [7.45, -8.8], [1.45, -8.8]]
navigator.navigateCoverage(field)

i = 0
while not navigator.isTaskComplete():
    i += 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
        print('ETA: ' + '{0:.0f}'.format(
              Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
              + ' seconds.')
    time.sleep(1)

result = navigator.getResult()
if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
elif result == TaskResult.FAILED:
    print('Goal failed!')
else:
    print('Goal has an invalid return status!')
```

---

## Setup B — OEZ Experiment (current)

### Concept

The field is split into two zones that bracket a physical obstacle. The coverage planner
generates independent paths for each zone; the robot navigates around the obstacle between
zones using Nav2's global costmap (obstacle is 1.2 m tall → detectable by LiDAR).

The `opennav_coverage` inner-polygon void API (`polygons[1]` as a hole) was found to be
unreliable — the headland generator strips inner rings before passing the field to the
swath generator. Two sequential calls is the reliable approach.

### World geometry

```
Walls (inner faces):
  North  y = −3.3     South  y = −11.3
  East   x =  9.0     West   x = −0.1

field_north:  x ∈ [1.45, 7.45],  y ∈ [−6.64, −3.3]
  height 3.34 m → swath y ∈ [−5.79, −4.15] = 1.64 m → 3 rows
  row centres: −4.40, −4.87, −5.34

Obstacle (OEZ):
  Gazebo pose:  (4.45, −7.4, 0.6)     ← centre (x, y, z=half-height)
  Gazebo size:  (3.0,  1.2, 1.2)      ← (E-W length, N-S depth, height)
  Footprint:    x ∈ [2.95, 5.95],  y ∈ [−8.0, −6.8]
  Both fields' boundaries clear the obstacle with ≥ 0.1 m margin.
  1.2 m height → LiDAR detects it → NavfnPlanner routes around it.

field_south:  x ∈ [1.45, 7.45],  y ∈ [−11.3, −8.1]
  height 3.2 m → swath y ∈ [−10.45, −8.95] = 1.5 m → 3 rows
  row centres: −9.19, −9.66, −10.13

Spawn:  (2.3, −3.5, 0.2)  yaw = −1.5708  (unchanged)

Gap zone (not covered by design):  y ∈ [−8.1, −6.64]
  Contains obstacle y ∈ [−8.0, −6.8] — fully enclosed ✓
  Flanking corridors (robot transit paths):
    West:  x ∈ [1.45, 2.95]  — robot routes west of obstacle between fields
    East:  x ∈ [5.95, 7.45]  — robot routes east of obstacle between fields
```

### Clearance verification

| Boundary | Field edge | Obstacle face | Margin |
|----------|-----------|---------------|--------|
| field_north south | −6.64 | obstacle north −6.8 | 0.16 m |
| field_south north | −8.1 | obstacle south −8.0 | 0.10 m |
| East wall inner | 9.0 | field_east 7.45 | 1.55 m (> obstacle_max_range 1.5 m ✓) |
| West wall inner | −0.1 | field_west 1.45 | 1.55 m ✓ |

### `gravel_featureless.world` wall and obstacle models

```xml
<!-- North wall: inner face y = −3.3  (unchanged from Setup A) -->
<model name="wall_north">
  <static>true</static>
  <pose>4.45 -3.2 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>9.5 0.2 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>9.5 0.2 2</size></box></geometry></visual>
  </link>
</model>

<!-- South wall: inner face y = −11.3 -->
<model name="wall_south">
  <static>true</static>
  <pose>4.45 -11.4 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>9.5 0.2 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>9.5 0.2 2</size></box></geometry></visual>
  </link>
</model>

<!-- East wall: inner face x = 9.0 -->
<model name="wall_east">
  <static>true</static>
  <pose>9.1 -7.3 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>0.2 8.4 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>0.2 8.4 2</size></box></geometry></visual>
  </link>
</model>

<!-- West wall: inner face x = −0.1 -->
<model name="wall_west">
  <static>true</static>
  <pose>-0.2 -7.3 1.0 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>0.2 8.4 2</size></box></geometry></collision>
    <visual    name="visual">  <geometry><box><size>0.2 8.4 2</size></box></geometry></visual>
  </link>
</model>

<!-- OEZ obstacle: brown box in gap zone, clear of both coverage fields -->
<model name="oez_obstacle">
  <static>true</static>
  <pose>4.45 -7.4 0.6 0 0 0</pose>
  <link name="link">
    <collision name="collision"><geometry><box><size>3.0 1.2 1.2</size></box></geometry></collision>
    <visual name="visual">
      <geometry><box><size>3.0 1.2 1.2</size></box></geometry>
      <material>
        <ambient>0.6 0.3 0.1 1</ambient>
        <diffuse>0.6 0.3 0.1 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

Camera (GUI):
```xml
<pose frame=''>4.45 -22.0 20.0 0.0 0.8 1.57</pose>
```

### `demo_coverage.py` `main()` body

```python
field_north = [[1.45, -6.64], [1.45, -3.3], [7.45, -3.3], [7.45, -6.64], [1.45, -6.64]]
field_south = [[1.45, -11.3], [1.45, -8.1], [7.45, -8.1], [7.45, -11.3], [1.45, -11.3]]

result = runCoverage(navigator, field_north, 'North zone (rows 1-3)')
if result != TaskResult.SUCCEEDED:
    rclpy.shutdown()
    return

result = runCoverage(navigator, field_south, 'South zone (rows 6-8)')
rclpy.shutdown()
```

The `runCoverage` helper (defined above `main`) calls `navigator.navigateCoverage(field)`,
then spins until completion, printing ETA feedback every 5 seconds.

### Observed behaviours (expected)

| Behaviour | Explanation |
|-----------|-------------|
| Robot enters south field mid-row | `CoverageNavigator` attaches to the nearest waypoint on the generated path, not waypoint 0. Avoids a long backtrack. |
| Flanking columns not covered | Those columns are inside the gap zone (OEZ), not part of either field — intentional. |
| Robot routes east or west of obstacle during transit | NavfnPlanner uses global costmap; 1.2 m obstacle is LiDAR-visible and inflated, so the planner finds the clear corridor. |

---

## Setup C — Dynamic Obstacle Response

### Concept

Setup A (8-row field) runs normally. A separate ROS 2 node spawns a green box and sweeps it
north-south across the field at a configurable speed, crossing each coverage row in sequence.
The robot detects the obstacle via LiDAR, the local costmap marks it, and Nav2's recovery
behaviors (backup → spin → retry) keep the robot from being permanently blocked.

No changes to the world file or `demo_coverage.py` are needed — this is a run-time addition.

### How it works

```
Coverage path (E-W rows, pre-planned)
         ←────────── row 1 ──────────→
         ←────────── row 2 ──────────→
              ↕  green box sweeps N-S at 0.2 m/s
         ←────────── row 3 ──────────→
              ...
```

1. LiDAR detects the box → `/points_raw` PointCloud2 → local costmap marks it lethal
2. Controller (`use_collision_detection: true`) sees the carrot point is in a lethal cell
3. Controller reports failure → recovery tree: **backup → spin → resume**
4. Box moves on → raytrace clears its old position from costmap → robot continues the row
5. Cycle repeats as box crosses each subsequent row

### Launch procedure

Use **Setup A** (world file and `demo_coverage.py` in single-field configuration, see above).

```bash
# Terminal 1 — Gazebo + robot (Setup A world file — no pre-placed obstacles)
ros2 launch moborobo_robot minimal_gazebo.launch.py

# Terminal 2 — Nav2
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true

# Terminal 3 — coverage
ros2 run opennav_coverage_demo demo_coverage

# Terminal 4 — moving obstacle (start any time after Gazebo is up)
ros2 run moborobo_robot dynamic_obstacle_mover
```

Ctrl-C in Terminal 4 cleanly deletes the model from Gazebo.

### Obstacle specification

```
Model name:  moving_obstacle
Size:        1.5 m (E-W) × 0.5 m (N-S) × 1.2 m tall
Colour:      green  (ambient/diffuse 0.1 0.6 0.1)
Static:      true (teleported each tick; no physics reaction with robot)
Update rate: 10 Hz

Default sweep:
  x (fixed):  4.45  (field centre, E-W)
  y_north:   −4.0   (just inside row 1)
  y_south:   −8.5   (just inside row 8)
  speed:      0.2 m/s
  Period:     ~45 s per full N→S→N sweep
```

### Parameter overrides

```bash
# Slower sweep, offset to the west side of the field
ros2 run moborobo_robot dynamic_obstacle_mover \
  --ros-args -p x:=3.0 -p speed:=0.1 -p y_north:=-4.5 -p y_south:=-7.5
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `x` | 4.45 | Fixed E-W position (m) |
| `y_north` | −4.0 | Northern reversal point (m) |
| `y_south` | −8.5 | Southern reversal point (m) |
| `speed` | 0.2 | Sweep speed (m/s) |
| `model_name` | `moving_obstacle` | Gazebo entity name |

### Relevant Nav2 settings (already configured)

| Parameter | Value | Role in this experiment |
|-----------|-------|------------------------|
| `use_collision_detection` | true | controller detects imminent costmap collision |
| `max_allowed_time_to_collision_up_to_carrot` | 1.0 s | how far ahead collision is checked |
| `obstacle_max_range` | 1.5 m | local costmap marks obstacle within this range |
| `raytrace_max_range` | 2.0 m | clears old obstacle cells as robot moves |
| `failure_tolerance` | −1.0 | controller never permanently aborts; always retries |
| Recovery behaviors | backup, spin, wait | trigger when controller fails |

### Observed behaviours (expected)

| Behaviour | Explanation |
|-----------|-------------|
| Robot stops and backs up | Controller collision detection fires; backup recovery runs |
| Robot spins, then resumes | Spin recovery re-orients; obstacle has moved on → path is clear |
| Slight row undershoot/overshoot | Recovery behaviors shift the robot; RPPC re-locks onto path on next segment |
| Robot gets temporarily stuck at fast speeds | At speeds > 0.3 m/s the obstacle may re-enter the costmap before recovery finishes; reduce `speed` |

### Troubleshooting

**`/set_entity_state` service not found**
The service is provided by `libgazebo_ros_state.so`. Verify it is loaded:
```bash
ros2 service list | grep set_entity
```
If missing, the world file plugin may not have started yet — wait a few seconds after Gazebo launches.

**Obstacle not visible in RViz costmap**
The box is 1.2 m tall. If the LiDAR is mounted lower than expected it may clear the top of the box. Increase obstacle height in `_SDF` inside `dynamic_obstacle_mover.py`.

**Robot never recovers**
Check `failure_tolerance` in `nav_params_sim.yaml`. If set to a positive value the controller
will abort after that many seconds of failure rather than retrying indefinitely.

---

## Reverting A ↔ B

### A → B (add OEZ)

1. In `gravel_featureless.world`:
   - Change south wall pose y: `-8.9` → `-11.4`
   - Change E/W wall centre_y: `-6.05` → `-7.3`, size_y: `5.9` → `8.4`
   - Add `oez_obstacle` model (see above)
   - Update camera pose y/z
2. In `demo_coverage.py`: replace single-field `main()` body with two-call version
3. Rebuild: `colcon build --packages-select opennav_coverage_demo`

### B → A (remove OEZ)

1. In `gravel_featureless.world`:
   - Change south wall pose y: `-11.4` → `-8.9`
   - Change E/W wall centre_y: `-7.3` → `-6.05`, size_y: `8.4` → `5.9`
   - Remove `oez_obstacle` model block entirely
   - Restore camera pose to `4.45 -17.0 14.0 0.0 0.8 1.57`
2. In `demo_coverage.py`: replace two-call `main()` body with single-field version (see Setup A)
3. Rebuild: `colcon build --packages-select opennav_coverage_demo`

### A/B → C (add dynamic obstacle)

No world or demo script changes needed. Just build and run the mover node:
```bash
colcon build --packages-select moborobo_robot
ros2 run moborobo_robot dynamic_obstacle_mover
```

No changes to `nav_params_sim.yaml`, `minimal_gazebo.launch.py`, or `nav2_sim.launch.py`
are needed for any direction.
