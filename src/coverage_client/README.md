## Running the Node

```bash
ros2 run coverage_client coverage_client_node
```

---

## Coverage Area — Inline Polygon

### What is an inline polygon?

Unlike `NavigateToPose`, the `NavigateCompleteCoverage` action does not take a goal pose.
Instead, you define **what area to cover** by sending a polygon directly in the action goal.

This is called an **inline polygon** — a `geometry_msgs/Polygon` constructed in code and
sent at runtime, with no external files required.

---

### Which frame does the polygon live in?

**Always `map` frame.**

The Coverage Server tracks the robot's progress by continuously looking up where `base_link`
is relative to the polygon's declared frame via the TF tree. That means the polygon must live
in the same frame that anchors the rest of navigation.

Your TF chain at runtime is:

map → odom → base_link

Here is why the other frames are wrong:

| Frame | Why not to use it |
|---|---|
| `odom` | Drifts over time. The polygon would shift relative to the room as drift accumulates — the robot would cover the wrong area. |
| `base_link` | Moves with the robot. A polygon in this frame would follow the robot around, which makes no sense for a fixed area. |
| `map` | Globally consistent, drift-corrected, stable for the entire session. |

---

### What do the coordinates mean?

Coordinates are in **meters**, in the `map` frame. The origin `(0, 0)` is wherever RTAB-Map
placed the map origin when it first initialised — typically the robot's starting position.

So a point `(2.0, 1.5)` means: 2 metres along X and 1.5 metres along Y from where the robot
started.

---

### Prerequisite before sending a goal

You cannot send a valid coverage goal until `rtabmap` is publishing the `map → odom` transform.

If that transform does not exist yet — for example, because RTAB-Map has not finished
initialising — Nav2 cannot resolve the polygon's frame and the goal will either be rejected
or silently fail.

**Before calling the action, verify the transform exists:**

```bash
ros2 run tf2_ros tf2_echo map odom
```

If this returns a valid transform, the system is ready.
