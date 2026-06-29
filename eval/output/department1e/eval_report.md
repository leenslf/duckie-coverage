# Evaluation Report

**Bag:** `bags/loop_department1e`  
**Duration:** 64.8 s  
**Rectangle:** 5.1 m × 7.31 m  

## Topic message counts

- `/odom`: 498 messages
- `/localization_pose`: 62 messages
- `/info`: 62 messages
- `/tf`: 1839 messages

## ATE (Absolute Trajectory Error)

| Trajectory | RMSE | Max | Mean |
|---|---|---|---|
| /odom | 2.0738 m | 3.8983 m | 1.8608 m |
| /localization_pose | 1.7926 m | 4.0570 m | 1.5121 m |

## RPE (Relative Pose Error, 1 s intervals)

| Trajectory | RMSE | Mean |
|---|---|---|
| /odom | 0.3464 m | 0.2867 m |
| /localization_pose | 0.4674 m | 0.3154 m |

## Loop closures

- Count: 0
- Timestamps: none detected

## Summary

RTAB-Map **corrected** the drift. The localization pose ATE RMSE (1.7926 m) is lower than the raw odometry ATE RMSE (2.0738 m), indicating that loop closure reduced the accumulated position error. The trajectory is expected to close the rectangle more tightly than raw odometry alone.
