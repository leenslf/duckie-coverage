# Eval Scripts Overview

## compare_trajectories.py

Compares `/odom` vs `/localization_pose` against each other. No ground truth needed.

**Usage:**
```bash
python eval/compare_trajectories.py --bag bags/run_001
```

**Outputs:**

| File | What it shows |
|---|---|
| `trajectories.png` | XY plot of both paths shifted to a common origin |
| `divergence.png` | Euclidean distance between the two trajectories over time, with loop closure markers |
| `summary.md` | Max/mean divergence, when divergence first exceeds 0.5 m, loop closure count and effect |

**Answers:** How much do odom and localization_pose disagree with each other, and does loop closure bring them back together?

---

## run_eval_real.py

Evaluates both trajectories against a rectangle ground truth defined by CLI arguments.

**Usage:**
```bash
python eval/run_eval_real.py --bag bags/run_001 --width 4.0 --height 3.0
```

**Outputs:**

| File | What it shows |
|---|---|
| `trajectory_xy.png` | GT rectangle (dashed) overlaid with both aligned trajectories |
| `ate_over_time.png` | Absolute position error vs GT over time for both trajectories |
| `rpe_distribution.png` | Histogram of local drift per ~1 s interval |
| `eval_report.md` | ATE (RMSE, max, mean), RPE (RMSE, mean), loop closure count, plain-English conclusion |

**Answers:** How accurately does each trajectory follow the intended path, and did SLAM correct the drift?

**Notes:**
- Requires non-zero width and height for full Umeyama alignment. A degenerate GT (e.g. width=0) falls back to origin-only alignment.
- `/info` loop closure events require `rtabmap_msgs` to be deserializable — if not, loop closures are skipped with a warning and the rest of the evaluation continues.
