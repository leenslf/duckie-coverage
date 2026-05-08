## Original controller stack had three layers:

```yaml
FollowPath:
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  # ... shim params ...
  primary_controller: "dwb_core::DWBLocalPlanner"
  # ... DWB params ...
```

The **RotationShimController** was a wrapper, not a real controller. Its only job was to intercept the goal, rotate the robot to face the first waypoint, then hand off to the real controller underneath it. Once the robot was roughly aligned, the shim got out of the way and DWB took over.

**DWB (Dynamic Window Based local planner)** was the actual controller. It works by:
1. Sampling hundreds of possible velocity commands `(vx, vy, vtheta)` within the robot's dynamic limits
2. Forward-simulating each sample for `sim_time: 2.0` seconds to generate candidate trajectories
3. Scoring every trajectory with a set of critics (BaseObstacle, PathDist, GoalDist, PathAlign, GoalAlign, ObstacleFootprint, Oscillation, RotateToGoal)
4. Sending the velocity command from the highest-scoring trajectory

DWB is fundamentally a **sampling-based optimizer** — it picks the best velocity from a discrete set of candidates. The critics competed with each other: `GoalDist.scale: 40.0` pulled hard toward the final goal, `PathDist.scale: 15.0` tried to keep the robot on the path. The high `GoalDist` scale was why the robot skipped rows — the end of the coverage path scored much higher than the next waypoint, so DWB always tried to drive toward the final destination.

---

## What we have now

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
```

**RegulatedPurePursuit (RPP)** works completely differently — it's a **geometric controller**, not a sampling optimizer. It:

1. Finds a "carrot" point on the path at exactly `lookahead_dist` ahead of the robot
2. Computes the single arc (circle) that passes through the robot's current pose and the carrot point
3. Outputs exactly one velocity command: `desired_linear_vel` forward, with angular velocity determined by the arc curvature
4. No sampling, no critics, no scoring — just pure geometry

This is fundamentally better for coverage paths because it follows the path **sequentially by construction** — the carrot always moves forward along the path, so the robot physically cannot skip rows the way DWB could.

---

## The current FollowPath has a serious problem

The current YAML still has DWB params mixed into the RPP section:

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  # RPP params (used):
  desired_linear_vel: 0.3
  lookahead_dist: 0.6
  ...
  
  # DWB params (silently ignored by RPP):
  decel_lim_x: -2.5
  acc_lim_x: 2.5
  sim_time: 2.0
  linear_granularity: 0.05
  prune_plane: false
  shorten_transformed_plan: False
  critics: ["BaseObstacle", "GoalAlign", ...]
  BaseObstacle.scale: 60.0
  ...
```

RPP completely ignores all the DWB params — the critics, scales, trajectory sampling params, and pruning params are all dead code. They don't cause errors but they're confusing and could mask issues.

---

## Key behavioral differences for your coverage use case

| Behavior | DWB + Shim | RPP |
|---|---|---|
| Path following | Scores trajectories toward goal — skips rows | Geometric carrot — always sequential |
| Rotation before driving | RotationShim handles it | Built-in `use_rotate_to_heading` |
| Obstacle avoidance | Critics score obstacle proximity | Collision detection stops robot |
| Computation cost | Heavy — 450 samples × 8 critics every 100ms | Lightweight — single geometric calculation |
| Coverage suitability | Poor — goal-seeking fights path-following | Good — designed for path tracking |
| Row skipping | Frequent with high GoalDist scale | Prevented by `max_robot_pose_search_dist` |