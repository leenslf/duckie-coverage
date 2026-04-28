# Trajectories

These are the test trajectories for VIO benchmarking.

## Trajectory definitions

| ID | Name             | Description                                                   |
|----|------------------|---------------------------------------------------------------|
| T1 | Straight line    | 4 m forward at ≤ 0.2 m/s, pause, return. Tests linear drift. |
| T2 | Small square     | Four 1 m sides with 90° turns. Tests rotation error.          |
| T4 | Figure-8         | Two overlapping ovals. Primary SLAM loop-closure test.        |
| T5 | Slow random walk | ~2 min hand-guided, ≤ 0.1 m/s. Tests drift over time.        |
| T6 | Fast straight    | 3 m at 0.4 m/s. Stress test for motion blur and tracking.    |

## Ground truth setup

<!-- TODO: add tag layout diagram and world-frame survey results once arena is set up -->

## Notes

- Run each trajectory 3× and record each run as a separate bag.
- T6 is a stress test only — do not use it for ATE/RPE evaluation.
- Keep camera pointing toward tags during T1, T2, T4, T5.
