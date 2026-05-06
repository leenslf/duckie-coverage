#!/usr/bin/env python3
"""
Parse an odom topic dump and save a trajectory plot alongside it.

Usage:
    python3 plot_trajectory.py analysis/odom_20260506_143021.txt
    python3 plot_trajectory.py analysis/odom_20260506_143021.txt --no-altitude
"""

import sys
import re
from pathlib import Path
import matplotlib.pyplot as plt

show_altitude = '--no-altitude' not in sys.argv
if not show_altitude:
    sys.argv.remove('--no-altitude')

def parse_odom(path):
    xs, ys, zs = [], [], []
    text = Path(path).read_text()

    # Each pose block looks like:
    #   pose:\n    position:\n      x: ...\n      y: ...\n      z: ...
    pattern = re.compile(
        r'pose:\s+position:\s+x:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)'
        r'\s+y:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)'
        r'\s+z:\s*([+-]?\d+\.?\d*(?:e[+-]?\d+)?)',
        re.IGNORECASE,
    )
    for m in pattern.finditer(text):
        xs.append(float(m.group(1)))
        ys.append(float(m.group(2)))
        zs.append(float(m.group(3)))

    return xs, ys, zs


def plot(path):
    xs, ys, zs = parse_odom(path)
    if not xs:
        print("No pose/position data found.")
        sys.exit(1)

    print(f"Parsed {len(xs)} poses from {path}")

    ncols = 2 if show_altitude else 1
    fig, axes = plt.subplots(1, ncols, figsize=(6 * ncols, 5))
    if ncols == 1:
        axes = [axes]
    fig.suptitle(Path(path).name)

    # XY top-down view
    axes[0].plot(xs, ys, linewidth=0.8)
    axes[0].scatter(xs[0], ys[0], color='green', s=40, zorder=5, label='start')
    axes[0].scatter(xs[-1], ys[-1], color='red', s=40, zorder=5, label='end')
    axes[0].set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].set_title('Top-down (XY)')
    axes[0].set_aspect('equal')
    axes[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1), borderaxespad=0)
    axes[0].grid(True, linewidth=0.4)

    # Z over time (optional)
    if show_altitude:
        axes[1].plot(zs, linewidth=0.8, color='steelblue')
        axes[1].set_xlabel('sample index')
        axes[1].set_ylabel('z (m)')
        axes[1].set_title('Altitude (Z)')
        axes[1].grid(True, linewidth=0.4)

    out = Path(path).with_suffix('.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', bbox_extra_artists=axes[0].get_legend() and [axes[0].get_legend()])
    print(f"Saved → {out}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_trajectory.py <odom_dump.txt>")
        sys.exit(1)
    plot(sys.argv[1])
