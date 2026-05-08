# Coverage Navigation Experiments & Demo Plan

## Setup Reference
- Robot: moborobo_robot (differential drive, 0.70×0.47m body, 16-beam 3D lidar)
- World: gravel.world (12×16m room with mixed obstacles)
- Stack: Nav2 + opennav_coverage + RegulatedPurePursuit
- Launch: `ros2 launch moborobo_robot minimal_gazebo.launch.py` + `ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true`

---

## Experiment 1 — Baseline Full Room Coverage

**Goal:** Demonstrate the robot covering the maximum navigable area of the room in one run.

**What to show:** The robot systematically sweeping the entire obstacle-free floor area, demonstrating that the coverage path generator correctly tiles the space with no large gaps.

### Setup
```python
# demo_coverage.py
field = [[-5.0, -7.0], [-5.0, 7.0], [5.0, 7.0], [5.0, -7.0], [-5.0, -7.0]]
```
```python
# minimal_gazebo.launch.py spawn
'-x', '0.0', '-y', '0.0', '-z', '0.2'
```
```yaml
# coverage_server params
default_swath_angle_type: "DIRECTION"
default_swath_angle: 0.0       # east-west rows
robot_width: 0.47
operation_width: 0.47
```

### Steps
- [ ] Launch Gazebo + Nav2
- [ ] Record RViz screen showing `/received_global_plan` before demo starts
- [ ] Run demo: `ros2 run opennav_coverage_demo demo_coverage`
- [ ] Record Gazebo + RViz side by side for full run
- [ ] Note: `distance_remaining` in terminal output, total time, final result

### Success criteria
- Robot covers all rows without getting permanently stuck
- Demo prints `Goal succeeded!`
- RViz path shows all rows completed (path disappears as waypoints are visited)

### Metrics to capture
```bash
# In separate terminal during run:
ros2 topic echo /navigate_complete_coverage/_action/feedback | grep -E "distance|navigation_time"
```

---

## Experiment 2 — Obstacle Exclusion Zones

**Goal:** Demonstrate the inner polygon feature — the robot skips areas around known obstacles rather than trying to navigate through them.

**What to show:** Side-by-side comparison of coverage path with and without inner exclusion polygons. Shows intelligent field decomposition.

### Setup — Run A (no exclusions, baseline)
```python
field = [[-3.5, -5.5], [-3.5, 5.5], [3.5, 5.5], [3.5, -5.5], [-3.5, -5.5]]
navigator.navigateCoverage(field)
```

### Setup — Run B (with exclusions around main obstacles)
```python
field = [[-3.5, -5.5], [-3.5, 5.5], [3.5, 5.5], [3.5, -5.5], [-3.5, -5.5]]

# Exclude L-obstacle area
inner1 = [[-3.8, 2.6], [-3.8, 4.8], [-1.0, 4.8], [-1.0, 2.6], [-3.8, 2.6]]

# Exclude medium pillar
inner2 = [[0.3, 1.3], [0.3, 3.7], [2.7, 3.7], [2.7, 1.3], [0.3, 1.3]]

goal_msg.polygons.append(navigator.toPolygon(field))
goal_msg.polygons.append(navigator.toPolygon(inner1))
goal_msg.polygons.append(navigator.toPolygon(inner2))
```

### Steps
- [ ] Run A first — record the robot attempting to navigate through obstacle areas
- [ ] Reset simulation
- [ ] Run B — record the robot cleanly skipping excluded zones
- [ ] Capture RViz screenshots of both paths side by side

### Success criteria
- Run A: path cuts through obstacles, robot may get stuck or detour significantly
- Run B: path cleanly routes around excluded areas, fewer DWB/RPP failures

---

## Experiment 3 — Partial Area Coverage (Zone-Based)

**Goal:** Demonstrate targeted coverage of specific zones — useful for selective cleaning, inspection of specific areas, or staged coverage.

**What to show:** Three separate coverage goals sent sequentially, each covering a different zone of the room. Simulates a real use case where different areas need different treatment.

### Setup
```python
# Zone definitions
zone_south = [[-3.0, -7.0], [-3.0, -2.0], [3.0, -2.0], [3.0, -7.0], [-3.0, -7.0]]
zone_center = [[-3.0, -2.0], [-3.0, 2.0], [3.0, 2.0], [3.0, -2.0], [-3.0, -2.0]]
zone_north = [[-3.0, 2.0], [-3.0, 7.0], [3.0, 7.0], [3.0, 2.0], [-3.0, 2.0]]

# Run each zone sequentially in main():
for zone in [zone_south, zone_center, zone_north]:
    navigator.navigateCoverage(zone)
    while not navigator.isTaskComplete():
        time.sleep(1)
    print(f"Zone complete: {navigator.getResult()}")
```

### Steps
- [ ] Run the three-zone demo
- [ ] In RViz add a **Marker** display showing zone boundaries in different colors
- [ ] Record the robot transitioning between zones
- [ ] Capture time per zone from navigation_time feedback

### Success criteria
- All three zones complete with `Goal succeeded!`
- Robot visibly transitions from south to center to north
- Total coverage time recorded

---

## Experiment 4 — Row Orientation Comparison

**Goal:** Compare east-west rows vs north-south rows for this specific room and obstacle layout. Shows how swath angle affects path efficiency and number of headland turns.

**What to show:** The same field covered with two different row orientations — demonstrating how orientation choice affects total path length, number of turns, and navigation difficulty.

### Setup — Run A (east-west rows, angle=0)
```yaml
coverage_server:
  ros__parameters:
    default_swath_angle_type: "DIRECTION"
    default_swath_angle: 0.0
```

### Setup — Run B (north-south rows, angle=90°)
```yaml
coverage_server:
  ros__parameters:
    default_swath_angle_type: "DIRECTION"
    default_swath_angle: 1.5708
```

### Metrics to capture for both runs
```bash
# Total path length:
ros2 topic echo /received_global_plan --once | grep -c "position"

# Time to complete:
ros2 topic echo /navigate_complete_coverage/_action/feedback | grep navigation_time | tail -1

# Number of rows (count turns):
ros2 topic echo /received_global_plan --once | grep "z:" | wc -l
```

### Steps
- [ ] Run A, record path in RViz, record completion time
- [ ] Run B, record path in RViz, record completion time
- [ ] Compare: which orientation has fewer turns? Which completes faster?
- [ ] Screenshot both `/received_global_plan` paths in RViz for comparison

---

## Experiment 5 — Dynamic Obstacle Response

**Goal:** Demonstrate that the robot detects and responds to a moving obstacle (the dynamic_box in gravel.world) during coverage.

**What to show:** While coverage is running, push the dynamic box into the robot's path. The robot should slow down or stop, then resume when the path is clear.

### Setup
```bash
# Terminal 1: run coverage normally
ros2 run opennav_coverage_demo demo_coverage

# Terminal 2: wait until robot is on a row heading toward dynamic box area, then push it:
ros2 topic pub /dynamic_box/gazebo_ros_force geometry_msgs/msg/Wrench \
  "force: {x: 8.0, y: 0.0, z: 0.0}" -r 10

# Terminal 3: watch RPP collision detection:
ros2 topic echo /cmd_vel | grep -E "linear|angular"
```

### Steps
- [ ] Start coverage
- [ ] Wait for robot to be on the row passing near `(x=-1.5, y=-1.5)` (dynamic box location)
- [ ] Push the box into the robot's path
- [ ] Observe: does the robot stop? Does it slow down?
- [ ] Stop the force: `ros2 topic pub /dynamic_box/gazebo_ros_force geometry_msgs/msg/Wrench "force: {x: 0.0, y: 0.0, z: 0.0}" --once`
- [ ] Observe: does the robot resume coverage?

### Success criteria
- `RegulatedPurePursuitController detected collision ahead!` appears in nav2 log when box is in path
- Robot velocity drops toward zero
- Robot resumes after obstacle moves away
- Coverage eventually completes

---

## Experiment 6 — Coverage with Different Robot Widths / Overlap

**Goal:** Show the effect of `operation_width` on coverage density. Demonstrates how the parameter can be tuned for different cleaning/inspection requirements.

**What to show:** Three runs with different `operation_width` values — sparse (0.7m), standard (0.47m), and dense overlap (0.3m). Shows trade-off between thoroughness and time.

### Setup — Run A (sparse, 0.7m operation width)
```yaml
coverage_server:
  ros__parameters:
    operation_width: 0.7    # rows spaced wider than robot — gaps possible
    default_allow_overlap: false
```

### Setup — Run B (standard, 0.47m)
```yaml
coverage_server:
  ros__parameters:
    operation_width: 0.47   # rows exactly match robot width — no gaps, no overlap
    default_allow_overlap: false
```

### Setup — Run C (dense, 0.3m with overlap)
```yaml
coverage_server:
  ros__parameters:
    operation_width: 0.3    # rows overlap — each area covered ~1.5x
    default_allow_overlap: true
```

### Steps
- [ ] Run each configuration on the same field
- [ ] Screenshot `/received_global_plan` for each — row spacing visibly different
- [ ] Count rows: `ros2 topic echo /received_global_plan --once | grep -c "frame_id"`
- [ ] Record completion time for each

### Comparison table to fill in during demo

| Config | Operation Width | Row Count | Completion Time | Gaps? |
|--------|----------------|-----------|-----------------|-------|
| Sparse | 0.7m | | | |
| Standard | 0.47m | | | |
| Dense | 0.3m | | | |

---

## Experiment 7 — Headland Tuning Comparison

**Goal:** Show how `min_turning_radius` and `default_headland_width` affect the quality of the headland turns — the most common place where the robot drifts or gets stuck.

**What to show:** Compare tight turns (small radius, narrow headland) vs gentle turns (large radius, wide headland). Shows the trade-off between field utilization and navigation reliability.

### Setup — Run A (tight turns)
```yaml
coverage_server:
  ros__parameters:
    min_turning_radius: 0.5
    default_headland_width: 0.6
```

### Setup — Run B (current)
```yaml
coverage_server:
  ros__parameters:
    min_turning_radius: 0.8
    default_headland_width: 0.85
```

### Setup — Run C (gentle turns)
```yaml
coverage_server:
  ros__parameters:
    min_turning_radius: 1.2
    default_headland_width: 1.4
```

### Steps
- [ ] Run each and record the headland turns in RViz and Gazebo
- [ ] Note: does the robot drift after turns for each config?
- [ ] Note: how much floor area is lost to headland in each config?
- [ ] This experiment directly tests the drift investigation hypothesis

---

## Recording & Presentation Setup

### RViz configuration to save (save as `.rviz` file)
Add these displays before recording:
- **Map** → `/local_costmap/costmap` (alpha 0.7)
- **Map** → `/global_costmap/costmap` (alpha 0.5)
- **Path** (red) → `/received_global_plan`
- **Path** (blue) → `/local_plan`
- **Polygon** (green) → `/local_costmap/published_footprint`
- **PointCloud2** → `/points_raw`
- **TF** (show `map`, `odom`, `base_footprint`, `lidar` only)
- **Marker** → `/field_boundary` (add field publisher to demo script)

### Screen layout for recording
```
+---------------------------+------------------+
|                           |                  |
|     Gazebo (3D view)      |   RViz (top-down)|
|                           |                  |
|                           |                  |
+---------------------------+------------------+
|        Terminal (ETA + distance_remaining)   |
+----------------------------------------------+
```

### Commands to run before each experiment
```bash
# Reset Gazebo to known state
ros2 service call /reset_simulation std_srvs/srv/Empty

# Clear costmaps
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap

# Confirm nav2 is active
ros2 lifecycle get /bt_navigator
```

### Data to record for each experiment
```bash
# Save feedback to file for post-processing
ros2 topic echo /navigate_complete_coverage/_action/feedback > experiment_N_feedback.txt &

# Save cmd_vel for analysis
ros2 topic echo /cmd_vel > experiment_N_cmdvel.txt &
```