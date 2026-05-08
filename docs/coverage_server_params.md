
Current `coverage_server` config:

```yaml
coverage_server:
  ros__parameters:
    use_sim_time: True
    robot_width: 0.47
    operation_width: 0.47
    default_headland_width: 0.85
    min_turning_radius: 0.8
    linear_curv_change: 2.5
    coordinates_in_cartesian_frame: true
    default_allow_overlap: false
    default_swath_angle_type: "SET_ANGLE"
    default_swath_angle: 1.5708
    default_swath_type: "LENGTH"
    default_path_type: "DUBIN"
    default_path_continuity_type: "CONTINUOUS"
    goal_dist_tolerance: 0.3
```

---

## How the coverage server works

The coverage server (`opennav_coverage`) takes a field polygon and generates a complete coverage path — a boustrophedon (lawnmower) pattern of parallel rows with headland turns connecting them. It does this in two steps: first it computes the swath rows (the straight parallel lines covering the field), then it computes the headland paths (the curved turns connecting row ends).

The parameters control each part of this process.

---

## What each parameter does and how it was changed

**`robot_width: 0.47`** — Was `0.55` in the original. This is the robot's physical width from the URDF (`0.47m` collision box). It's used as the default row spacing — rows are spaced `robot_width` apart so the robot just barely covers the field with no gaps. Corrected it to match the actual URDF dimension.

**`operation_width: 0.47`** — Was `0.9` in the original. This is the tool/implement width. For a cleaning or mowing robot this would be the brush width. Since the robot has no tool, we set it equal to `robot_width`. The original `0.9` was nearly double the robot width, causing large gaps between rows and poor coverage.

**`default_headland_width: 0.85`** — Was `0.4` in the original. The headland is the strip along the field boundary where the robot turns between rows. It must be wide enough for the robot to complete its turning arc. Increased it from `0.4` to `0.85` because `default_headland_width` must be greater than or equal to `min_turning_radius` — with the original `0.4` headland and `0.3` turning radius, the robot was trying to turn inside a space too tight for its arc, producing invalid paths. The current value gives a comfortable margin above `min_turning_radius: 0.8`.

**`min_turning_radius: 0.8`** — Was `0.3` in the original. This defines the tightest arc the coverage path generator will produce for headland turns. Increased it from `0.3` to `0.8` for two reasons: first, the robot's casters are at `±0.3m` from center, so a `0.3m` radius turn drags the casters harshly; second, the RegulatedPurePursuit controller tracks gentle curves much better than tight ones. A `0.8m` radius produces smooth arcs that RPP can follow at `0.3 m/s` without overshooting.

**`linear_curv_change: 2.5`** — Unchanged. This controls how quickly the path transitions from straight to curved at row ends. Higher values mean more abrupt transitions. `2.5` is a reasonable default.

**`coordinates_in_cartesian_frame: true`** — Unchanged. Tells the server the field polygon coordinates are in a flat Cartesian frame (map frame), not geographic coordinates. Always `true` for simulation.

**`default_allow_overlap: false`** — Was `true` in the original. When `true`, rows can overlap at field boundaries. Set it to `false` for cleaner coverage without redundant passes.

**`default_swath_angle_type: "SET_ANGLE"`** — Unchanged from original. This means `default_swath_angle` is interpreted as an absolute angle in the field's local frame. The alternative `"DIRECTION"` would interpret it as a map-frame compass direction. Contemplated switching to `"DIRECTION"` to fix diagonal rows, which is still a pending change.

**`default_swath_angle: 1.5708`** — Was `1.57` (same thing, just more precise). This sets the row direction to 90° — intended to produce north-south rows. However because `SET_ANGLE` is relative to the field polygon's orientation rather than the map, this produces diagonal rows when the field polygon isn't axis-aligned. The fix is to change `default_swath_angle_type` to `"DIRECTION"`.

**`default_swath_type: "LENGTH"`** — Unchanged. Rows are computed to maximize length (longest possible straight lines through the field). The alternative `"NUMBER"` lets you specify a fixed number of rows.

**`default_path_type: "DUBIN"`** — Was `"REEDS_SHEPP"` in the original. Dubin paths are forward-only arcs — the robot never reverses in headland turns. Reeds-Shepp allows reversing. We switched to Dubin because the differential drive robot has no need to reverse for headland turns, and forward-only paths are smoother and easier for RPP to follow.

**`default_path_continuity_type: "CONTINUOUS"`** — Was `"DISCONTINUOUS"` in the original. With `DISCONTINUOUS`, the robot stops at each row end and makes a discrete turn. With `CONTINUOUS`, the headland arc smoothly connects rows without stopping. Switched to `CONTINUOUS` because RPP tracks continuous paths much better — discrete stops cause RPP to declare the waypoint reached and potentially skip ahead.

**`goal_dist_tolerance: 0.3`** — Was not set in the original (causing the `default_custom_order` crash in early runs). This tells the coverage navigator how close the robot needs to get to each waypoint before considering it reached and moving to the next one. `0.3m` is tighter than the `0.35` original `xy_goal_tolerance` — may want to increase this to `0.5` if the robot is skipping waypoints or declaring success too early near obstacles.