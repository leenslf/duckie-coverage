#!/usr/bin/env python3
"""
compare_trajectories.py — compare /odom vs /localization_pose from a ROS 2 bag.
No ground truth. No Umeyama. Visual drift inspection only.

Usage:
    python3 eval/compare_trajectories.py --bag bags/loop_department1e
"""

import argparse
import sys
import warnings
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

warnings.filterwarnings("ignore")

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

OUTPUT_DIR = Path(__file__).parent / "output"

REQUIRED_TOPICS = {
    "/odom": "nav_msgs/msg/Odometry",
    "/localization_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
}


# ── Bag validator ──────────────────────────────────────────────────────────────

def validate_bag(bag_path: str):
    """
    Check required topics. SystemExit if /odom or /localization_pose are missing.
    Returns (topic_counts dict, info_available bool).
    """
    missing = []
    empty = []
    topic_info = {}
    info_available = False

    with Reader(bag_path) as reader:
        topics = reader.topics
        for topic, msgtype in REQUIRED_TOPICS.items():
            if topic not in topics:
                missing.append(f"  {topic}  ({msgtype})  — not found in bag")
            elif topics[topic].msgcount == 0:
                empty.append(f"  {topic}  — present but 0 messages")
            else:
                topic_info[topic] = topics[topic].msgcount

        if "/info" in topics and topics["/info"].msgcount > 0:
            info_available = True
            topic_info["/info"] = topics["/info"].msgcount
        else:
            print(
                "WARNING: /info not found or empty in bag — "
                "loop closure events will not be shown.",
                file=sys.stderr,
            )

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

    return topic_info, info_available


# ── Trajectory extractors (reused from run_eval_real.py) ──────────────────────

def extract_odom(bag_path: str, typestore) -> list:
    """Read /odom, return [(t_sec, x, y, z, qx, qy, qz, qw)]."""
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
    """Read /localization_pose, return [(t_sec, x, y, z, qx, qy, qz, qw)]."""
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


# ── Loop closure extractor (reused from run_eval_real.py) ─────────────────────

def extract_loop_closures(bag_path: str, typestore) -> list:
    """Return timestamps (sec) where loopClosureId != 0. Returns [] on failure."""
    lc_times = []
    try:
        with Reader(bag_path) as reader:
            conns = [c for c in reader.connections if c.topic == "/info"]
            for conn, ts_ns, raw in reader.messages(connections=conns):
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
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


# ── Plots ──────────────────────────────────────────────────────────────────────

def plot_trajectories(odom_poses, loc_poses, lc_timestamps):
    """
    trajectories.png: both trajectories shifted to (0,0) for visual comparison.
    Loop closure events marked as red X on localization_pose.
    """
    odom_t = np.array([p[0] for p in odom_poses])
    odom_x = np.array([p[1] for p in odom_poses]) - odom_poses[0][1]
    odom_y = np.array([p[2] for p in odom_poses]) - odom_poses[0][2]

    loc_t = np.array([p[0] for p in loc_poses])
    loc_x = np.array([p[1] for p in loc_poses]) - loc_poses[0][1]
    loc_y = np.array([p[2] for p in loc_poses]) - loc_poses[0][2]

    fig, ax = plt.subplots(figsize=(10, 10))

    ax.plot(odom_x, odom_y, color="darkorange", linewidth=1.2, label="/odom", zorder=2)
    ax.plot(loc_x, loc_y, color="steelblue", linewidth=1.2,
            label="/localization_pose", zorder=2)
    ax.plot(0, 0, "ko", markersize=8, label="start (0, 0)", zorder=3)

    if lc_timestamps:
        # Clamp to valid time range before interpolating
        t_min, t_max = loc_t[0], loc_t[-1]
        valid_lc = [t for t in lc_timestamps if t_min <= t <= t_max]
        if valid_lc:
            lc_x_marks = np.interp(valid_lc, loc_t, loc_x)
            lc_y_marks = np.interp(valid_lc, loc_t, loc_y)
            ax.scatter(lc_x_marks, lc_y_marks, marker="x", color="crimson",
                       s=80, linewidths=2,
                       label=f"loop closure ({len(lc_timestamps)})", zorder=4)

    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Trajectory Comparison: /odom vs /localization_pose")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = OUTPUT_DIR / "trajectories.png"
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")


def plot_divergence(odom_poses, loc_poses, lc_timestamps):
    """
    divergence.png: Euclidean distance between trajectories over time.
    Both trajectories shifted to (0,0) before distance computation.
    Returns (t_rel array, divergence array) with t_rel relative to run start.
    """
    t0 = min(odom_poses[0][0], loc_poses[0][0])

    odom_t = np.array([p[0] for p in odom_poses])
    odom_x = np.array([p[1] for p in odom_poses]) - odom_poses[0][1]
    odom_y = np.array([p[2] for p in odom_poses]) - odom_poses[0][2]

    loc_t = np.array([p[0] for p in loc_poses])
    loc_x = np.array([p[1] for p in loc_poses]) - loc_poses[0][1]
    loc_y = np.array([p[2] for p in loc_poses]) - loc_poses[0][2]

    # Interpolate sparser onto denser timestamps
    if len(odom_t) >= len(loc_t):
        ref_t = odom_t
        ref_x = odom_x
        ref_y = odom_y
        other_x = np.interp(ref_t, loc_t, loc_x)
        other_y = np.interp(ref_t, loc_t, loc_y)
    else:
        ref_t = loc_t
        ref_x = loc_x
        ref_y = loc_y
        other_x = np.interp(ref_t, odom_t, odom_x)
        other_y = np.interp(ref_t, odom_t, odom_y)

    divergence = np.sqrt((ref_x - other_x) ** 2 + (ref_y - other_y) ** 2)
    t_rel = ref_t - t0

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t_rel, divergence, color="purple", linewidth=1.0,
            label="divergence (m)", zorder=2)

    if lc_timestamps:
        lc_rel = [t - t0 for t in lc_timestamps]
        for lc_t_r in lc_rel:
            if t_rel[0] <= lc_t_r <= t_rel[-1]:
                ax.axvline(lc_t_r, color="crimson", linestyle="--",
                           linewidth=0.8, alpha=0.7, zorder=1)
        # Proxy artist for legend
        handles, labels = ax.get_legend_handles_labels()
        handles.append(Line2D([0], [0], color="crimson", linestyle="--", linewidth=0.8))
        labels.append(f"loop closure ({len(lc_timestamps)})")
        ax.legend(handles=handles, labels=labels, loc="best")
    else:
        ax.legend(loc="best")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.set_title("Divergence: /odom vs /localization_pose")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = OUTPUT_DIR / "divergence.png"
    plt.savefig(out, dpi=150)
    plt.close()
    print(f"Saved {out}")

    return t_rel, divergence


# ── Summary ────────────────────────────────────────────────────────────────────

def _path_length(poses) -> float:
    xs = np.array([p[1] for p in poses])
    ys = np.array([p[2] for p in poses])
    return float(np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2).sum())


def write_summary(bag_path, odom_poses, loc_poses, lc_timestamps, t_rel, divergence):
    bag_name = Path(bag_path).name
    t0 = min(odom_poses[0][0], loc_poses[0][0])
    duration = max(odom_poses[-1][0], loc_poses[-1][0]) - t0
    total_dist = _path_length(loc_poses)
    max_div = float(divergence.max())
    mean_div = float(divergence.mean())
    lc_count = len(lc_timestamps)

    # When does divergence first exceed 0.5 m?
    threshold = 0.5
    exceeds = np.where(divergence > threshold)[0]
    if len(exceeds) > 0:
        drift_sentence = (
            f"Divergence first exceeds {threshold} m at approximately "
            f"{t_rel[exceeds[0]]:.1f} s into the run."
        )
    else:
        drift_sentence = f"Divergence stays below {threshold} m throughout the run."

    # Loop closure effect
    if lc_count == 0:
        lc_sentence = "No loop closures were detected, so no corrections were applied."
        lc_effect = ""
    else:
        lc_sentence = (
            f"{lc_count} loop closure(s) were detected "
            "(shown as red dashed lines on the divergence plot and red X markers on the trajectory)."
        )
        first_lc_rel = lc_timestamps[0] - t0
        idx_lc = int(np.searchsorted(t_rel, first_lc_rel))
        if 0 < idx_lc < len(divergence) - 10:
            pre_mean = divergence[:idx_lc].mean()
            post_mean = divergence[idx_lc:].mean()
            if post_mean < pre_mean * 0.8:
                lc_effect = (
                    " The mean divergence decreases after the first loop closure "
                    f"(pre: {pre_mean:.2f} m → post: {post_mean:.2f} m), "
                    "suggesting the localization pose reconverged toward the odom path."
                )
            else:
                lc_effect = (
                    " Divergence does not clearly decrease after loop closures "
                    f"(pre: {pre_mean:.2f} m, post: {post_mean:.2f} m), "
                    "indicating the two sources remain separated even after corrections."
                )
        else:
            lc_effect = ""

    paragraph = (
        f"The bag `{bag_name}` spans {duration:.1f} s and covers approximately "
        f"{total_dist:.1f} m of travel (measured along the /localization_pose path). "
        f"The maximum divergence between /odom and /localization_pose is {max_div:.2f} m, "
        f"with a mean of {mean_div:.2f} m over the run. "
        f"{drift_sentence} "
        f"{lc_sentence}"
        f"{lc_effect}"
    )

    lines = [
        "# Trajectory Comparison Summary\n",
        f"**Bag:** `{bag_name}`  ",
        f"**Duration:** {duration:.1f} s  ",
        f"**Total distance (localization_pose path length):** {total_dist:.2f} m  ",
        "",
        "## Divergence (/odom vs /localization_pose)",
        "",
        f"- Max: {max_div:.3f} m",
        f"- Mean: {mean_div:.3f} m",
        "",
        "## Loop Closures",
        "",
        f"- Count: {lc_count}",
        "",
        "## Description",
        "",
        paragraph,
        "",
    ]

    out = OUTPUT_DIR / "summary.md"
    out.write_text("\n".join(lines))
    print(f"Saved {out}")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare /odom vs /localization_pose from a ROS 2 bag (no ground truth)."
    )
    parser.add_argument("--bag", required=True, help="Path to ROS 2 bag directory")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    bag_path = args.bag
    print(f"\n=== Comparing trajectories: {bag_path} ===\n")

    _topic_counts, info_available = validate_bag(bag_path)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    print("\nExtracting /odom...")
    odom_poses = extract_odom(bag_path, typestore)
    print(f"  {len(odom_poses)} poses")

    print("Extracting /localization_pose...")
    loc_poses = extract_localization(bag_path, typestore)
    print(f"  {len(loc_poses)} poses")

    lc_timestamps = []
    if info_available:
        print("Extracting loop closures from /info...")
        lc_timestamps = extract_loop_closures(bag_path, typestore)
        print(f"  {len(lc_timestamps)} loop closure(s)")

    print("\nGenerating trajectories.png...")
    plot_trajectories(odom_poses, loc_poses, lc_timestamps)

    print("Generating divergence.png...")
    t_rel, divergence = plot_divergence(odom_poses, loc_poses, lc_timestamps)

    print("Writing summary.md...")
    write_summary(bag_path, odom_poses, loc_poses, lc_timestamps, t_rel, divergence)

    print(f"\nDone. Output files in: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
