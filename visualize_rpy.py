#!/usr/bin/env python3
"""
Graph commanded vs actual roll/pitch/yaw over time from hopcopter attitude logs.

Usage:
    python visualize_rpy.py                               # graphs latest file in position_attitude_tracking/
    python visualize_rpy.py attitude_<timestamp>.csv      # a file in that directory, by name
    python visualize_rpy.py path/to/specific.csv          # the specified file

Faint vertical lines mark touchdowns (jumping_state 1 -> 2).
"""

import os
import sys
import glob
import argparse
import pandas as pd
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
ATT_DIR = os.path.join(REPO_ROOT, "data", "data_ballistic_planner", "position_attitude_tracking")

# ──────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────

def latest_csv(directory: str) -> str:
    """Return the path to the most recently modified .csv in *directory*."""
    csvs = glob.glob(os.path.join(directory, "*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No CSV files found in {directory}")
    return max(csvs, key=os.path.getmtime)


def resolve_csv(arg: str) -> str:
    """Return *arg* if it is an existing path, else look for it by name in ATT_DIR."""
    if os.path.isfile(arg):
        return arg
    candidate = os.path.join(ATT_DIR, arg)
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(f"{arg} not found (also looked in {ATT_DIR})")


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Graph commanded vs actual roll/pitch/yaw.")
    parser.add_argument("file", nargs="?", default=None,
                        help="CSV path or filename in position_attitude_tracking/ (default: latest)")
    args = parser.parse_args()

    try:
        filepath = resolve_csv(args.file) if args.file else latest_csv(ATT_DIR)
    except FileNotFoundError as e:
        sys.exit(f"Error: {e}")

    df = pd.read_csv(filepath)
    t = df["timestamp"] - df["timestamp"].iloc[0]

    # Touchdown times: jumping_state goes 1 -> 2
    state = df["jumping_state"]
    touchdown_t = t[(state.shift() == 1) & (state == 2)]

    # Pitch: forward tilt is positive (right-hand rule about +y, which points left).
    # des_pitch already follows this; the logged act_pitch has the opposite sign.
    act_sign = {"roll": 1.0, "pitch": -1.0, "yaw": 1.0}

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
    for ax, name in zip(axes, ["roll", "pitch", "yaw"]):
        ax.plot(t, df[f"des_{name}_deg"], "--", label=f"desired {name}")
        ax.plot(t, act_sign[name] * df[f"act_{name}_deg"], "-", label=f"actual {name}")
        for td in touchdown_t:
            ax.axvline(td, color="gray", alpha=0.3, linewidth=0.8)
        ax.set_ylabel(f"{name} [deg]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(os.path.basename(filepath))

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
