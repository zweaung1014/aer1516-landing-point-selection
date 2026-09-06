#!/usr/bin/env python3
"""
Visualize hop tracking odometry: pose position and linear velocity over time.

Usage:
    python visualize_hop_tracking.py                    # latest run folder's cf_0_odom.csv
    python visualize_hop_tracking.py path/to/run_folder  # specific run folder
    python visualize_hop_tracking.py path/to/cf_0_odom.csv  # specific CSV

Produces one window with two stacked subplots sharing a time axis:
    1. Pose position x, y, z (m)
    2. Linear twist x, y, z — i.e. linear velocity (m/s)
"""

import sys
import os
import glob
import argparse
import pandas as pd
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
HOP_TRACKING_DIR = os.path.join(REPO_ROOT, "data_ballistic_planner", "hop_tracking")


# ──────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────

def latest_run_dir(directory: str) -> str:
    """Return the run folder with the lexicographically greatest timestamp name."""
    runs = [d for d in glob.glob(os.path.join(directory, "*")) if os.path.isdir(d)]
    if not runs:
        raise FileNotFoundError(f"No run folders found in {directory}")
    return max(runs, key=os.path.basename)


def find_odom_csv(run_dir: str) -> str:
    """Return the path to cf_0_odom.csv inside *run_dir*."""
    csv_path = os.path.join(run_dir, "cf_0_odom.csv")
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"No cf_0_odom.csv found in {run_dir}")
    return csv_path


def load_odom(filepath: str) -> pd.DataFrame:
    """Load cf_0_odom.csv and return time, position, and linear velocity columns."""
    df = pd.read_csv(filepath)
    t = df["header.stamp.sec"] + df["header.stamp.nanosec"] * 1e-9
    t -= t.iloc[0]  # zero-base relative to the start of the run
    return pd.DataFrame({
        "t": t,
        "pos_x": df["pose.pose.position.x"],
        "pos_y": df["pose.pose.position.y"],
        "pos_z": df["pose.pose.position.z"],
        "vel_x": df["twist.twist.linear.x"],
        "vel_y": df["twist.twist.linear.y"],
        "vel_z": df["twist.twist.linear.z"],
    })


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Visualize hop tracking pose position and linear velocity over time.")
    parser.add_argument("path", nargs="?", default=None,
                        help="Path to a run folder or a cf_0_odom.csv file. "
                             "Defaults to the latest run folder in "
                             "data_ballistic_planner/hop_tracking/.")
    args = parser.parse_args()

    try:
        if args.path is None:
            run_dir = latest_run_dir(HOP_TRACKING_DIR)
            print(f"Using latest run folder: {run_dir}")
            csv_path = find_odom_csv(run_dir)
        elif os.path.isdir(args.path):
            csv_path = find_odom_csv(args.path)
        else:
            csv_path = args.path
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(csv_path):
        print(f"Error: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Using CSV: {csv_path}")
    df = load_odom(csv_path)

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))

    ax1.plot(df["t"], df["pos_x"], label="x")
    ax1.plot(df["t"], df["pos_y"], label="y")
    ax1.plot(df["t"], df["pos_z"], label="z")
    ax1.set_ylabel("Position (m)")
    ax1.set_title("Pose Position")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(df["t"], df["vel_x"], label="x")
    ax2.plot(df["t"], df["vel_y"], label="y")
    ax2.plot(df["t"], df["vel_z"], label="z")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Linear velocity (m/s)")
    ax2.set_title("Linear Twist (Velocity)")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
