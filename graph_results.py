#!/usr/bin/env python3
"""
Graph RRT* (and future planner) path results on an XY plane.

Usage:
    python graph_results.py                          # graphs latest file in data/data_rrt_star/
    python graph_results.py path/to/specific.csv     # graphs the specified file

Axes convention:
    +x  →  vertical up
    +y  →  horizontal left

Designed to be extended: add more data sources below (Section: ADDITIONAL DATA SOURCES)
to overlay paths from different planners on the same graph for comparison.
"""

import sys
import os
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ──────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
RRT_STAR_DIR = os.path.join(REPO_ROOT, "data", "data_rrt_star")
HOPCOPTER_DIR = os.path.join(REPO_ROOT, "data", "data_hopcopter")
HOPCOPTER_NO_LOCAL_DIR = os.path.join(REPO_ROOT, "data", "data_hopcopter_no_local")

# ──────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────

def latest_csv(directory: str) -> str:
    """Return the path to the most recently modified .csv in *directory*."""
    csvs = glob.glob(os.path.join(directory, "*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No CSV files found in {directory}")
    return max(csvs, key=os.path.getmtime)


def latest_n_csvs(directory: str, n: int = 2, offset: int = 0) -> list:
    """Return up to *n* most recently modified .csv paths in *directory* (newest first),
    skipping the first *offset* files.  offset=0 → latest, offset=1 → second-latest, etc."""
    csvs = glob.glob(os.path.join(directory, "*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No CSV files found in {directory}")
    csvs.sort(key=os.path.getmtime, reverse=True)
    return csvs[offset:offset + n]


def load_xy(filepath: str) -> pd.DataFrame:
    """Load a CSV and return a DataFrame with at least columns x, y."""
    df = pd.read_csv(filepath)
    if "x" not in df.columns or "y" not in df.columns:
        raise ValueError(f"{filepath} must contain 'x' and 'y' columns")
    return df


def load_hopcopter_xy(filepath: str) -> pd.DataFrame:
    """Load a hopcopter landing CSV and return a DataFrame with x, y from foot_x, foot_y."""
    df = pd.read_csv(filepath)
    if "foot_x" not in df.columns or "foot_y" not in df.columns:
        raise ValueError(f"{filepath} must contain 'foot_x' and 'foot_y' columns")
    return pd.DataFrame({"x": df["foot_x"], "y": df["foot_y"]})


def plot_path(ax, df: pd.DataFrame, *, label: str = "", color: str = None,
              marker: str = "o", linestyle: str = "-", linewidth: float = 1.5,
              markersize: float = 5):
    """
    Plot a path on the given axes.

    Because the desired orientation is +x up / +y left, we map:
        horizontal axis  ←  y   (inverted so positive y goes left)
        vertical axis    ←  x   (positive x goes up)
    """
    ax.plot(df["y"], df["x"], marker=marker, linestyle=linestyle,
            linewidth=linewidth, markersize=markersize, label=label,
            color=color)


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Graph path comparison on XY plane.")
    parser.add_argument("--offset", type=int, default=0,
                        help="Skip the N most recent files (0=latest, 1=second-latest, ...)")
    args = parser.parse_args()
    offset = args.offset

    fig, ax = plt.subplots(figsize=(10, 8))

    # --- RRT* paths ---
    try:
        rrt_files = latest_n_csvs(RRT_STAR_DIR, 1, offset)
        for i, f in enumerate(rrt_files):
            print(f"RRT* file [{i}]: {f}")
            rrt_df = load_xy(f)
            plot_path(ax, rrt_df, label="RRT*" if i == 0 else "_nolegend_",
                      color="red", marker="o")
    except FileNotFoundError:
        print("No RRT* CSV found; skipping.")

    # ══════════════════════════════════════════════════════════════════
    # SECTION: ADDITIONAL DATA SOURCES
    # Add more paths here to overlay on the same graph for comparison.
    # ══════════════════════════════════════════════════════════════════

    # --- Hopcopter landing points ---
    try:
        hop_files = latest_n_csvs(HOPCOPTER_DIR, 1, offset)
        for i, f in enumerate(hop_files):
            print(f"Hopcopter file [{i}]: {f}")
            hop_df = load_hopcopter_xy(f)
            plot_path(ax, hop_df, label="RRT* + landing point selection" if i == 0 else "_nolegend_",
                      color="green", marker="s")
    except FileNotFoundError:
        print("No hopcopter CSV found; skipping.")

    # --- Hopcopter NO local planner ---
    try:
        hop_nl_files = latest_n_csvs(HOPCOPTER_NO_LOCAL_DIR, 1, offset)
        for i, f in enumerate(hop_nl_files):
            print(f"Hopcopter no-local file [{i}]: {f}")
            hop_nl_df = load_hopcopter_xy(f)
            plot_path(ax, hop_nl_df, label="RRT* only" if i == 0 else "_nolegend_",
                      color="black", marker="s")
    except FileNotFoundError:
        print("No hopcopter_no_local CSV found; skipping.")

    # --- Obstacle from SDF (test_obstacle) ---
    # Obstacle center: (x=0.5, y=-0.2), box 0.15x0.15 → half-width 0.075m
    # Inflation radius from rrt_star.cpp: robot.radius = 0.35m
    # On graph axes: horizontal=y, vertical=x
    obs_x, obs_y = 0.5, -0.2
    obs_half = 0.075       # half of 0.15m box
    inflate_r = 0.35       # robot.radius from rrt_star.cpp

    # Solid black circle for the physical obstacle
    obstacle_circle = mpatches.Circle((obs_y, obs_x), obs_half,
                                      color="black", fill=True, zorder=5,
                                      label="Obstacle")
    ax.add_patch(obstacle_circle)

    # Dotted circle for the inflated exclusion zone
    inflation_circle = mpatches.Circle((obs_y, obs_x), obs_half + inflate_r,
                                       color="black", fill=False,
                                       linestyle="--", linewidth=1.5, zorder=5,
                                       label="Inflation zone")
    ax.add_patch(inflation_circle)

    # --- Axes & labels ---
    ax.set_xlabel("Y (m, positive ← left)", fontsize=24)
    ax.set_ylabel("X (m, positive ↑ up)", fontsize=24)
    ax.set_xlim(0.4, -0.7)     # horizontal = y, positive left
    ax.set_ylim(-0.2, 1.3)      # vertical = x, positive up
    ax.set_aspect("equal")
    ax.tick_params(axis="both", labelsize=20)
    ax.legend(fontsize=22, loc="upper left", bbox_to_anchor=(1.02, 1), borderaxespad=0)
    ax.grid(True, alpha=0.3)
    ax.set_title("Path Comparison (XY Plane)", fontsize=28)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
