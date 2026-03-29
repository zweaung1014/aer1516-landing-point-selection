#!/usr/bin/env python3
"""
Visualize candidate landing point scores from the local planner.

Usage:
    python visualize_local_planner.py -l 5                                  # latest CSV, line 5
    python visualize_local_planner.py data/data_local_planner/output.csv -l 5  # specific CSV, line 5

Line numbering is 1-based as seen in a text editor (line 1 = header).

Axes convention (same as graph_results.py):
    +x  →  vertical up
    +y  →  horizontal left

Candidate colors: red (low score) → green (high score), matching local_planner.cpp.
"""

import sys
import os
import glob
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.cm import ScalarMappable

# ──────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
LOCAL_PLANNER_DIR = os.path.join(REPO_ROOT, "data", "data_local_planner")


# ──────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────

def latest_csv(directory: str) -> str:
    """Return the path to the most recently modified .csv in *directory*."""
    csvs = glob.glob(os.path.join(directory, "*.csv"))
    if not csvs:
        raise FileNotFoundError(f"No CSV files found in {directory}")
    return max(csvs, key=os.path.getmtime)


def score_to_rgba(score: float) -> tuple:
    """
    Convert a score in [0, 1] to an RGBA colour.
    Matches local_planner.cpp:
        R = 1.0 - score,  G = score,  B = 0.0,  A = 0.7
    """
    s = float(np.clip(score, 0.0, 1.0))
    return (1.0 - s, s, 0.0, 0.7)


# ──────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Visualize local planner candidate scores on an XY plane.")
    parser.add_argument("csv_file", nargs="?", default=None,
                        help="Path to a local_planner CSV. "
                             "Defaults to the latest file in data/data_local_planner/.")
    parser.add_argument("-l", "--line", type=int, required=True,
                        help="1-based line number to visualize (line 1 = header).")
    args = parser.parse_args()

    # Resolve CSV path
    csv_path = args.csv_file
    if csv_path is None:
        csv_path = latest_csv(LOCAL_PLANNER_DIR)
        print(f"Using latest CSV: {csv_path}")

    if not os.path.isfile(csv_path):
        print(f"Error: file not found: {csv_path}", file=sys.stderr)
        sys.exit(1)

    # Load CSV (treat "-" as NaN)
    df = pd.read_csv(csv_path, na_values=["-"])

    # Detect max candidate count from CSV columns
    max_cand = 0
    for col in df.columns:
        if col.startswith("cand") and col.endswith("_x"):
            n = int(col[len("cand"):-len("_x")])
            if n > max_cand:
                max_cand = n
    if max_cand == 0:
        print("Error: no candidate columns found in CSV.", file=sys.stderr)
        sys.exit(1)

    # Convert 1-based text-editor line to 0-based DataFrame row
    # Line 1 = header, Line 2 = first data row → row index 0
    row_idx = args.line - 2
    if row_idx < 0 or row_idx >= len(df):
        print(f"Error: line {args.line} is out of range. "
              f"Valid data lines are 2–{len(df) + 1}.", file=sys.stderr)
        sys.exit(1)

    row = df.iloc[row_idx]

    # Check that candidate data exists
    first_cand_score = row.get("cand1_score")
    if pd.isna(first_cand_score):
        print(f"Error: line {args.line} has no candidate data "
              f"(all candidates are empty / dashes).", file=sys.stderr)
        sys.exit(1)

    # ── Extract candidates ────────────────────────────────────────────
    cand_x, cand_y, cand_scores = [], [], []
    for i in range(1, max_cand + 1):
        cx = row.get(f"cand{i}_x")
        cy = row.get(f"cand{i}_y")
        cs = row.get(f"cand{i}_score")
        if pd.notna(cx) and pd.notna(cy) and pd.notna(cs):
            cand_x.append(float(cx))
            cand_y.append(float(cy))
            cand_scores.append(float(cs))

    if not cand_scores:
        print(f"Error: no valid candidate points found on line {args.line}.",
              file=sys.stderr)
        sys.exit(1)

    cand_x = np.array(cand_x)
    cand_y = np.array(cand_y)
    cand_scores = np.array(cand_scores)
    cand_colors = [score_to_rgba(s) for s in cand_scores]

    # ── Extract original & selected waypoints ─────────────────────────
    orig_x = float(row["original_x"])
    orig_y = float(row["original_y"])

    sel_x = row.get("selected_x")
    sel_y = row.get("selected_y")
    has_selected = pd.notna(sel_x) and pd.notna(sel_y)
    if has_selected:
        sel_x, sel_y = float(sel_x), float(sel_y)

    total_score = row.get("total_score")
    timestamp = row.get("timestamp", "")

    # ── Plot ──────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 8))

    # Candidates: scatter with red-to-green color by score
    # Axis mapping: horizontal = y (inverted), vertical = x
    ax.scatter(cand_y, cand_x, s=150, c=cand_colors, edgecolors="black",
               linewidths=0.5, zorder=3, label="Candidates")

    # Original waypoint (blue ×)
    ax.scatter([orig_y], [orig_x], s=200, c="blue", marker="x",
               linewidths=2.5, zorder=5, label="Original waypoint")

    # Selected (best) waypoint (blue star)
    if has_selected:
        score_text = f" (score={float(total_score):.3f})" if pd.notna(total_score) else ""
        ax.scatter([sel_y], [sel_x], s=250, c="blue", marker="*",
                   linewidths=0.8, edgecolors="black", zorder=6,
                   label=f"Selected waypoint{score_text}")

    # ── Axes (same convention as graph_results.py, bounds auto-fitted) ─
    # Gather all plotted points to compute bounds
    all_x = list(cand_x) + [orig_x]
    all_y = list(cand_y) + [orig_y]
    if has_selected:
        all_x.append(sel_x)
        all_y.append(sel_y)
    pad = 0.15  # padding around data extent
    x_min, x_max = min(all_x) - pad, max(all_x) + pad
    y_min, y_max = min(all_y) - pad, max(all_y) + pad
    # Keep equal aspect by expanding the smaller range
    x_range = x_max - x_min
    y_range = y_max - y_min
    if x_range > y_range:
        mid_y = (y_min + y_max) / 2
        y_min, y_max = mid_y - x_range / 2, mid_y + x_range / 2
    else:
        mid_x = (x_min + x_max) / 2
        x_min, x_max = mid_x - y_range / 2, mid_x + y_range / 2

    ax.set_xlabel("Y (m, positive ← left)", fontsize=24)
    ax.set_ylabel("X (m, positive ↑ up)", fontsize=24)
    ax.set_xlim(y_max, y_min)   # inverted: positive y = left
    ax.set_ylim(x_min, x_max)
    ax.set_aspect("equal")
    ax.tick_params(axis="both", labelsize=20)
    ax.legend(fontsize=22, loc="upper left", bbox_to_anchor=(1.02, 1), borderaxespad=0, labelspacing=1.2)
    ax.grid(True, alpha=0.3)

    title = f"Local Planner Candidates — line {args.line}"
    if timestamp:
        title += f"\n{timestamp}"
    ax.set_title(title, fontsize=28)

    # ── Colorbar (red→green) ──────────────────────────────────────────
    # Build a custom red-to-green LinearSegmentedColormap
    cmap = mcolors.LinearSegmentedColormap.from_list(
        "score_rg", [(1, 0, 0), (1, 1, 0), (0, 1, 0)])
    norm = mcolors.Normalize(vmin=0, vmax=1)
    sm = ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, shrink=0.4, pad=0.02, anchor=(0.0, 0.4))
    cbar.set_label("Score", fontsize=20)
    cbar.ax.tick_params(labelsize=16)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
