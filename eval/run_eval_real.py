#!/usr/bin/env python3
"""
run_eval_real.py — offline evaluation of ROS 2 SLAM/VIO against rectangle ground truth.

Usage:python3 eval/run_eval_real.py --bag bags/loop_department1e --width 5.1 --height 7.31
    
"""

import argparse
import math
import os
import sys
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

warnings.filterwarnings("ignore")

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from evo.core.metrics import APE, RPE, PoseRelation, StatisticsType
from evo.core.units import Unit
from evo.core import sync
from evo.core.trajectory import PoseTrajectory3D

OUTPUT_DIR = Path(__file__).parent / "output"

REQUIRED_TOPICS = {
    "/odom": "nav_msgs/msg/Odometry",
    "/localization_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/info": "rtabmap_msgs/msg/Info",
    "/tf": "tf2_msgs/msg/TFMessage",
}


# ── Bag validation ────────────────────────────────────────────────────────

def validate_bag(bag_path: str) -> dict:
    """Check every required topic is present and non-empty. SystemExit on failure."""
    missing = []
    empty = []
    topic_info = {}

    with Reader(bag_path) as reader:
        topics = reader.topics
        for topic, msgtype in REQUIRED_TOPICS.items():
            if topic not in topics:
                missing.append(f"  {topic}  ({msgtype})  — not found in bag")
            elif topics[topic].msgcount == 0:
                empty.append(f"  {topic}  — present but 0 messages")
            else:
                topic_info[topic] = topics[topic].msgcount

    errors = []
    if missing:
        errors.append("Missing topics:\n" + "\n".join(missing))
    if empty:
        errors.append("Empty topics (0 messages):\n" + "\n".join(empty))

    if errors:
        print("ERROR: bag validation failed.\n" + "\n\n".join(errors), file=sys.stderr)
        raise SystemExit(1)

    print("Bag OK:")
    for topic, count in topic_info.items():
        print(f"  {topic}: {count} messages")
    return topic_info


# ── Trajectory extraction ─────────────────────────────────────────────────

def extract_odom(bag_path: str, typestore) -> list:
    """Read /odom and return [(t_sec, x, y, z, qx, qy, qz, qw)]."""
    poses = []
    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic == "/odom"]
        for conn, ts_ns, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((ts_ns * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    poses.sort(key=lambda r: r[0])
    return poses


def extract_localization(bag_path: str, typestore) -> list:
    """Read /localization_pose and return [(t_sec, x, y, z, qx, qy, qz, qw)]."""
    poses = []
    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic == "/localization_pose"]
        for conn, ts_ns, raw in reader.messages(connections=conns):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((ts_ns * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    poses.sort(key=lambda r: r[0])
    return poses


# ── Loop closure extraction ───────────────────────────────────────────────

def extract_loop_closures(bag_path: str, typestore) -> list:
    """Return list of timestamps (sec) where loop_closure_id != 0.
    Prints a warning and returns [] if /info cannot be deserialized."""
    lc_times = []
    try:
        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections if c.topic == "/info"]
            for conn, ts_ns, raw in reader.messages(connections=conns):
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                # field may be loop_closure_id or loopClosureId depending on definition
                lc_id = getattr(msg, "loop_closure_id",
                                getattr(msg, "loopClosureId", 0))
                if lc_id != 0:
                    lc_times.append(ts_ns * 1e-9)
    except Exception as exc:
        print(
            f"WARNING: could not deserialize /info messages ({exc}).\n"
            "  Loop closure events will not be shown.",
            file=sys.stderr,
        )
        return []
    return lc_times


# ── Rectangle ground truth ────────────────────────────────────────────────

def build_rectangle_gt(
    width: float,
    height: float,
    t_start: float,
    t_end: float,
    num_points: int = 500,
) -> list:
    """
    Sample a closed rectangle uniformly by arc length into num_points poses.
    Path: (0,0)→(W,0)→(W,H)→(0,H)→(0,0).
    Timestamps span [t_start, t_end].
    Returns [(t_sec, x, y, z=0, qx=0, qy=0, qz, qw)].
    """
    perimeter = 2.0 * (width + height)

    # Each segment: (start_x, start_y, direction_x, direction_y, heading_rad, length)
    segments = [
        (0.0,   0.0,    1,  0,  0.0,           width),
        (width, 0.0,    0,  1,  math.pi / 2,   height),
        (width, height, -1, 0,  math.pi,        width),
        (0.0,   height, 0,  -1, 3 * math.pi / 2, height),
    ]
    # Cumulative arc-length boundaries
    bounds = [0.0]
    for *_, seg_len in segments:
        bounds.append(bounds[-1] + seg_len)

    arc_samples = np.linspace(0.0, perimeter, num_points, endpoint=True)
    timestamps = np.linspace(t_start, t_end, num_points)

    poses = []
    for i in range(num_points):
        s = arc_samples[i]
        # Find segment (search from the end for last boundary <= s)
        seg_idx = 0
        for k in range(len(segments) - 1, -1, -1):
            if s >= bounds[k]:
                seg_idx = k
                break
        sx, sy, dx, dy, heading, _ = segments[seg_idx]
        offset = s - bounds[seg_idx]
        x = sx + dx * offset
        y = sy + dy * offset
        qz = math.sin(heading / 2.0)
        qw = math.cos(heading / 2.0)
        poses.append((timestamps[i], x, y, 0.0, 0.0, 0.0, qz, qw))

    return poses


def _to_evo_traj(poses: list) -> PoseTrajectory3D:
    """Convert [(t, x, y, z, qx, qy, qz, qw)] to evo PoseTrajectory3D.
    evo quaternion order is [w, x, y, z]."""
    ts = np.array([p[0] for p in poses])
    xyz = np.array([[p[1], p[2], p[3]] for p in poses])
    # input: qx qy qz qw  →  evo wants: qw qx qy qz
    quats_wxyz = np.array([[p[7], p[4], p[5], p[6]] for p in poses])
    return PoseTrajectory3D(
        positions_xyz=xyz,
        orientations_quat_wxyz=quats_wxyz,
        timestamps=ts,
    )


# ── Metrics ───────────────────────────────────────────────────────────────

def compute_metrics(est_poses: list, gt_poses: list, label: str) -> dict:
    """
    Align est to gt via Umeyama, then compute ATE (APE) and RPE.
    Returns dict with scalar stats and per-timestamp error arrays.
    """
    traj_est = _to_evo_traj(est_poses)
    traj_gt = _to_evo_traj(gt_poses)

    # Time-synchronise: match GT timestamps to estimated timestamps
    try:
        traj_gt_sync, traj_est_sync = sync.associate_trajectories(
            traj_gt, traj_est, max_diff=0.5
        )
    except Exception as exc:
        print(f"WARNING [{label}]: trajectory sync failed ({exc}). Skipping metrics.",
              file=sys.stderr)
        return None

    if len(traj_est_sync.timestamps) < 4:
        print(f"WARNING [{label}]: fewer than 4 matched poses after sync. Skipping.",
              file=sys.stderr)
        return None

    # Umeyama alignment (rigid, no scale correction).
    # Falls back to translation-only if GT is degenerate (e.g. a straight line).
    try:
        traj_est_sync.align(traj_gt_sync, correct_scale=False)
    except Exception as align_exc:
        print(f"WARNING [{label}]: Umeyama failed ({align_exc}). "
              "Falling back to origin alignment (shift to GT start).",
              file=sys.stderr)
        traj_est_sync.align_origin(traj_gt_sync)

    # ATE — Absolute Pose Error (translation)
    ape = APE(PoseRelation.translation_part)
    ape.process_data((traj_gt_sync, traj_est_sync))
    ate_rmse = ape.get_statistic(StatisticsType.rmse)
    ate_max = ape.get_statistic(StatisticsType.max)
    ate_mean = ape.get_statistic(StatisticsType.mean)
    ate_timestamps = traj_est_sync.timestamps
    ate_errors = np.array(ape.error)

    # RPE — Relative Pose Error at ~1-second intervals.
    # Unit.seconds is not implemented in this evo build; convert to frames instead.
    n_poses = len(traj_est_sync.timestamps)
    duration_sync = traj_est_sync.timestamps[-1] - traj_est_sync.timestamps[0]
    fps = n_poses / max(duration_sync, 1e-6)
    delta_frames = max(1, round(fps))  # frames per ~1 second
    try:
        rpe = RPE(
            PoseRelation.translation_part,
            delta=delta_frames,
            delta_unit=Unit.frames,
            all_pairs=False,
        )
        rpe.process_data((traj_gt_sync, traj_est_sync))
        rpe_rmse = rpe.get_statistic(StatisticsType.rmse)
        rpe_mean = rpe.get_statistic(StatisticsType.mean)
        rpe_errors = np.array(rpe.error)
    except Exception as exc:
        print(f"WARNING [{label}]: RPE computation failed ({exc}).", file=sys.stderr)
        rpe_rmse = rpe_mean = float("nan")
        rpe_errors = np.array([])

    return {
        "ate_rmse": ate_rmse,
        "ate_max": ate_max,
        "ate_mean": ate_mean,
        "ate_timestamps": ate_timestamps,
        "ate_errors": ate_errors,
        "rpe_rmse": rpe_rmse,
        "rpe_mean": rpe_mean,
        "rpe_errors": rpe_errors,
        "traj_aligned": traj_est_sync,  # for XY plot
    }


# ── Plots ─────────────────────────────────────────────────────────────────

def plot_trajectory_xy(
    gt_poses, odom_poses, loc_poses, lc_timestamps,
    odom_metrics, loc_metrics,
):
    """XY trajectory: GT (dashed grey) + odom (orange) + localization (blue)."""
    fig = plt.figure(figsize=(10, 10))

    has_lc = len(lc_timestamps) > 0
    if has_lc:
        ax_xy = fig.add_subplot(2, 1, 1)
        ax_lc = fig.add_subplot(2, 1, 2)
    else:
        ax_xy = fig.add_subplot(1, 1, 1)

    # Ground truth rectangle
    gt_x = [p[1] for p in gt_poses] + [gt_poses[0][1]]
    gt_y = [p[2] for p in gt_poses] + [gt_poses[0][2]]
    ax_xy.plot(gt_x, gt_y, "--", color="grey", linewidth=1.5, label="GT rectangle")

    # Aligned trajectories from evo (already in GT frame after Umeyama)
    if odom_metrics is not None:
        traj = odom_metrics["traj_aligned"]
        ax_xy.plot(traj.positions_xyz[:, 0], traj.positions_xyz[:, 1],
                   color="darkorange", linewidth=1.2, label="/odom (aligned)")

    if loc_metrics is not None:
        traj = loc_metrics["traj_aligned"]
        ax_xy.plot(traj.positions_xyz[:, 0], traj.positions_xyz[:, 1],
                   color="steelblue", linewidth=1.2, label="/localization_pose (aligned)")

    # Start marker
    ax_xy.plot(0, 0, "ko", markersize=8, label="start (0,0)")

    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_title("Trajectory XY — GT vs estimated")
    ax_xy.legend(loc="best")
    ax_xy.grid(True, alpha=0.3)

    # Loop closure events (secondary subplot)
    if has_lc:
        t0 = lc_timestamps[0] if lc_timestamps else 0
        lc_rel = [t - t0 for t in lc_timestamps]
        ax_lc.vlines(lc_rel, 0, 1, colors="crimson", linewidth=1.5, label="loop closure")
        ax_lc.set_xlim(0, max(lc_rel) * 1.1 if lc_rel else 1)
        ax_lc.set_ylim(0, 1)
        ax_lc.set_xlabel("Time (s)")
        ax_lc.set_ylabel("")
        ax_lc.set_title("Loop closure events")
        ax_lc.set_yticks([])
        ax_lc.legend(loc="best")
        ax_lc.grid(True, axis="x", alpha=0.3)

    plt.tight_layout()
    out = OUTPUT_DIR / "trajectory_xy.png"
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")


def plot_ate_over_time(odom_metrics, loc_metrics):
    """ATE error vs time for both trajectories."""
    fig, ax = plt.subplots(figsize=(10, 4))

    if odom_metrics is not None and len(odom_metrics["ate_errors"]) > 0:
        t = odom_metrics["ate_timestamps"]
        t_rel = t - t[0]
        ax.plot(t_rel, odom_metrics["ate_errors"],
                color="darkorange", linewidth=1.0, label="/odom ATE")

    if loc_metrics is not None and len(loc_metrics["ate_errors"]) > 0:
        t = loc_metrics["ate_timestamps"]
        t_rel = t - t[0]
        ax.plot(t_rel, loc_metrics["ate_errors"],
                color="steelblue", linewidth=1.0, label="/localization_pose ATE")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("ATE (m)")
    ax.set_title("Absolute Trajectory Error over time")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = OUTPUT_DIR / "ate_over_time.png"
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")


def plot_rpe_distribution(odom_metrics, loc_metrics):
    """RPE histogram for both trajectories."""
    fig, ax = plt.subplots(figsize=(8, 4))

    plotted = False
    if odom_metrics is not None and len(odom_metrics["rpe_errors"]) > 0:
        ax.hist(odom_metrics["rpe_errors"], bins=30, color="darkorange",
                alpha=0.6, label="/odom RPE")
        plotted = True

    if loc_metrics is not None and len(loc_metrics["rpe_errors"]) > 0:
        ax.hist(loc_metrics["rpe_errors"], bins=30, color="steelblue",
                alpha=0.6, label="/localization_pose RPE")
        plotted = True

    if not plotted:
        ax.text(0.5, 0.5, "No RPE data", ha="center", va="center",
                transform=ax.transAxes)

    ax.set_xlabel("RPE (m / 1 s interval)")
    ax.set_ylabel("Count")
    ax.set_title("Relative Pose Error distribution")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    out = OUTPUT_DIR / "rpe_distribution.png"
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")


# ── Report ────────────────────────────────────────────────────────────────

def write_report(
    bag_path: str,
    width: float,
    height: float,
    duration: float,
    topic_counts: dict,
    odom_metrics,
    loc_metrics,
    lc_timestamps: list,
):
    def _fmt(metrics, key):
        if metrics is None:
            return "N/A"
        v = metrics.get(key, float("nan"))
        return f"{v:.4f} m" if not math.isnan(v) else "N/A"

    odom_ate_rmse = odom_metrics["ate_rmse"] if odom_metrics else float("nan")
    loc_ate_rmse = loc_metrics["ate_rmse"] if loc_metrics else float("nan")
    slam_corrected = (
        not math.isnan(loc_ate_rmse)
        and not math.isnan(odom_ate_rmse)
        and loc_ate_rmse < odom_ate_rmse
    )

    if slam_corrected:
        conclusion = (
            f"RTAB-Map **corrected** the drift. "
            f"The localization pose ATE RMSE ({loc_ate_rmse:.4f} m) is lower than "
            f"the raw odometry ATE RMSE ({odom_ate_rmse:.4f} m), indicating that "
            f"loop closure reduced the accumulated position error. "
            f"The trajectory is expected to close the rectangle more tightly than "
            f"raw odometry alone."
        )
    elif math.isnan(loc_ate_rmse):
        conclusion = (
            "Localization pose data could not be evaluated (no valid poses or "
            "sync failed). Cannot determine whether RTAB-Map corrected the drift."
        )
    else:
        conclusion = (
            f"RTAB-Map did **not** clearly correct the drift in this run. "
            f"The localization pose ATE RMSE ({loc_ate_rmse:.4f} m) is not lower "
            f"than the raw odometry ATE RMSE ({odom_ate_rmse:.4f} m). "
            f"This may indicate that loop closure did not fire, or that the "
            f"localization pose was not significantly different from raw odometry."
        )

    lc_count = len(lc_timestamps)
    lc_str = (
        ", ".join(f"{t:.2f} s" for t in lc_timestamps)
        if lc_timestamps
        else "none detected"
    )

    lines = [
        "# Evaluation Report\n",
        f"**Bag:** `{bag_path}`  ",
        f"**Duration:** {duration:.1f} s  ",
        f"**Rectangle:** {width} m × {height} m  ",
        "",
        "## Topic message counts",
        "",
    ]
    for topic, count in topic_counts.items():
        lines.append(f"- `{topic}`: {count} messages")

    lines += [
        "",
        "## ATE (Absolute Trajectory Error)",
        "",
        f"| Trajectory | RMSE | Max | Mean |",
        f"|---|---|---|---|",
        f"| /odom | {_fmt(odom_metrics,'ate_rmse')} | {_fmt(odom_metrics,'ate_max')} | {_fmt(odom_metrics,'ate_mean')} |",
        f"| /localization_pose | {_fmt(loc_metrics,'ate_rmse')} | {_fmt(loc_metrics,'ate_max')} | {_fmt(loc_metrics,'ate_mean')} |",
        "",
        "## RPE (Relative Pose Error, 1 s intervals)",
        "",
        f"| Trajectory | RMSE | Mean |",
        f"|---|---|---|",
        f"| /odom | {_fmt(odom_metrics,'rpe_rmse')} | {_fmt(odom_metrics,'rpe_mean')} |",
        f"| /localization_pose | {_fmt(loc_metrics,'rpe_rmse')} | {_fmt(loc_metrics,'rpe_mean')} |",
        "",
        "## Loop closures",
        "",
        f"- Count: {lc_count}",
        f"- Timestamps: {lc_str}",
        "",
        "## Summary",
        "",
        conclusion,
        "",
    ]

    out = OUTPUT_DIR / "eval_report.md"
    out.write_text("\n".join(lines))
    print(f"Saved {out}")


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Evaluate ROS 2 SLAM/VIO bag against rectangle ground truth."
    )
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag directory")
    parser.add_argument("--width", type=float, required=True, help="Rectangle width (m)")
    parser.add_argument("--height", type=float, required=True, help="Rectangle height (m)")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    bag_path = args.bag
    width = args.width
    height = args.height

    print(f"\n=== Evaluating bag: {bag_path} ===")
    print(f"Rectangle: {width} m × {height} m\n")

    # Validate
    topic_counts = validate_bag(bag_path)

    # Typestore
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Extract trajectories
    print("\nExtracting /odom...")
    odom_poses = extract_odom(bag_path, typestore)
    print(f"  {len(odom_poses)} poses")

    print("Extracting /localization_pose...")
    loc_poses = extract_localization(bag_path, typestore)
    print(f"  {len(loc_poses)} poses")

    print("Extracting loop closures from /info...")
    lc_timestamps = extract_loop_closures(bag_path, typestore)
    print(f"  {len(lc_timestamps)} loop closure(s)")

    # Bag duration from /odom timestamps
    t_start = odom_poses[0][0]
    t_end = odom_poses[-1][0]
    duration = t_end - t_start
    print(f"\nBag duration (from /odom): {duration:.1f} s")

    # Build GT
    print("\nBuilding rectangle GT...")
    gt_poses = build_rectangle_gt(width, height, t_start, t_end)
    print(f"  {len(gt_poses)} GT poses sampled")

    # Compute metrics
    print("\nComputing metrics for /odom...")
    odom_metrics = compute_metrics(odom_poses, gt_poses, "/odom")
    if odom_metrics:
        print(f"  ATE RMSE: {odom_metrics['ate_rmse']:.4f} m  "
              f"max: {odom_metrics['ate_max']:.4f} m")
        print(f"  RPE RMSE: {odom_metrics['rpe_rmse']:.4f} m")

    print("Computing metrics for /localization_pose...")
    loc_metrics = compute_metrics(loc_poses, gt_poses, "/localization_pose")
    if loc_metrics:
        print(f"  ATE RMSE: {loc_metrics['ate_rmse']:.4f} m  "
              f"max: {loc_metrics['ate_max']:.4f} m")
        print(f"  RPE RMSE: {loc_metrics['rpe_rmse']:.4f} m")

    # Plots
    print("\nGenerating plots...")
    plot_trajectory_xy(gt_poses, odom_poses, loc_poses, lc_timestamps,
                       odom_metrics, loc_metrics)
    plot_ate_over_time(odom_metrics, loc_metrics)
    plot_rpe_distribution(odom_metrics, loc_metrics)

    # Report
    write_report(bag_path, width, height, duration, topic_counts,
                 odom_metrics, loc_metrics, lc_timestamps)

    print("\nDone. Output files in:", OUTPUT_DIR)


if __name__ == "__main__":
    main()
