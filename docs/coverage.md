# Coverage Navigation

## Working Implementation

A fully working boustrophedon coverage simulation is on the **`feature/coverage-server`** branch.
It uses a boustrophedon (zigzag row-by-row) coverage pattern with Reeds-Shepp headland turns,
running on ROS 2 Humble + Gazebo Classic + Nav2 + opennav_coverage (Fields2Cover) on the
`moborobo_robot` differential-drive platform.

> **Boustrophedon** = the row traversal pattern (row 1 east → row 2 west → row 3 east …).
> **Reeds-Shepp** = the turn path algorithm between rows — allows reversing, giving tighter
> headland maneuvers than forward-only Dubins curves.

See `src/docs/coverage_setups.md` on that branch for full field geometry, parameter rationale,
and experiment configurations (baseline, obstacle exclusion zone, dynamic obstacle).

### What is on `feature/coverage-server`

| Component | Location |
|-----------|----------|
| Gazebo world (4-wall arena, gravel) | `src/moborobo_robot/world/gravel_featureless.world` |
| Robot URDF + diff-drive plugin | `src/moborobo_robot/description/robot_core.xacro` |
| Ground-truth odometry node | `src/moborobo_robot/moborobo_robot/ground_truth_odom.py` |
| Nav2 + coverage launch | `src/nav_launch/launch/nav2_sim.launch.py` |
| All Nav2 + coverage parameters | `src/nav_launch/config/nav_params_sim.yaml` |
| Coverage demo + metrics script | `src/opennav_coverage/opennav_coverage_demo/opennav_coverage_demo/demo_coverage.py` |
| Dynamic obstacle node | `src/moborobo_robot/moborobo_robot/dynamic_obstacle_mover.py` |
| Experiment documentation | `src/docs/coverage_setups.md` |
| opennav_coverage packages (vendored) | `src/opennav_coverage/` |
| Fields2Cover library (vendored) | `src/Fields2Cover/` |

> **Note:** The external packages (`src/Fields2Cover/` and `src/opennav_coverage/`) are
> vendored directly on the feature branch and are not merged into `master` to keep the
> main branch lean. Check out `feature/coverage-server` to get the full working system.

### Quick Start (on `feature/coverage-server`)

```bash
# Terminal 1 — Gazebo + robot
ros2 launch moborobo_robot minimal_gazebo.launch.py

# Terminal 2 — Nav2 + ground truth odom + map→odom TF
ros2 launch nav_launch nav2_sim.launch.py launch_nav2:=true static_map_odom:=true

# Terminal 3 — Run coverage (prints path metrics on completion)
ros2 run opennav_coverage_demo demo_coverage

# Optional Terminal 4 — Dynamic moving obstacle
ros2 run moborobo_robot dynamic_obstacle_mover
```

Build after changing `moborobo_robot` (launch files are not symlinked):
```bash
colcon build --packages-select moborobo_robot
```

`nav_launch` YAML configs are symlinked — edit source and restart Nav2, no rebuild needed.

---

## Key Parameters

| Parameter | Value | Effect |
|-----------|-------|--------|
| `operation_width` | 0.47 m | Row spacing (= robot body width) |
| `default_headland_width` | 0.85 m | Turning margin at field edges |
| `min_turning_radius` | 0.8 m | Minimum Reeds-Shepp arc radius |
| `default_path_type` | REEDS_SHEPP | Allows reversing; tighter headland turns than Dubins |
| `obstacle_max_range` | 1.5 m | Reduced from 3 m to avoid wall detections during turns |

---

## Coverage Metrics (sample run, Setup A — 8 rows, 33 m² field)

| Metric | Value |
|--------|-------|
| Path length | 50.2 m |
| Theoretical minimum (rows only, no turns) | 34.4 m |
| Path efficiency | 68.5% |
| Turns detected | 7 |
| Headland overhead | 31.5% (15.8 m) |
| Avg turn cost per maneuver | 2.3 m |
| Total time | 202 s (3.37 min) |
| Area rate | 9.79 m²/min |

---

## ROS Topics (coverage-relevant)

| Topic | Description |
|-------|-------------|
| `/coverage_server/coverage_plan` | Full coverage path for RViz |
| `/coverage_server/field_boundary` | Field polygon visualisation |
| `/coverage_server/swaths` | Generated row swaths |
| `/received_global_plan` | Active plan from controller server |

---

## Background Reading

- [Fields2Cover tutorials](https://fields2cover.github.io/source/tutorials.html) — parts 3–7 cover headland generation, swath planning, and path planning
- [Nav2 coverage server docs](https://docs.nav2.org/configuration/packages/configuring-coverage-server.html)
