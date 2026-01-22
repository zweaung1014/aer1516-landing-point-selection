#!/usr/bin/env python3
"""
Plot Crazyflie-style data directly from a CSV.

Usage examples
--------------
# Plot everything the CSV supports
python plot_from_csv.py --csv clean_log.csv --plots all

# Only position + velocity, limit x-axis to first 6 seconds
python plot_from_csv.py --csv clean_log.csv --plots position velocity --xlimit 6

# RPYT + motors only
python plot_from_csv.py --csv clean_log.csv --plots rpyt motors
"""

import argparse
from pathlib import Path
from typing import Iterable, List

import pandas as pd
import matplotlib.pyplot as plt


def load_csv(path: Path) -> pd.DataFrame:
    """Read CSV and ensure an elapsed-time column 't' exists (build from 'UNIX Time' if needed)."""
    df = pd.read_csv(path)

    if "t" not in df.columns:
        if "UNIX Time" in df.columns:
            df["t"] = df["UNIX Time"] - df["UNIX Time"].iloc[0]
        else:
            raise KeyError("CSV must contain 't' or 'UNIX Time' to build elapsed time.")

    # Put 't' first for convenience
    cols = ["t"] + [c for c in df.columns if c != "t"]
    return df[cols]


def missing(df: pd.DataFrame, cols: Iterable[str]) -> List[str]:
    return [c for c in cols if c not in df.columns]


def plot_position(df: pd.DataFrame, x_limit: float | None = None):
    req = ["current_x", "desired_x", "current_y", "desired_y", "current_z", "desired_z"]
    miss = missing(df, req)
    if miss:
        print(f"⚠ Skipping position: missing {miss}")
        return

    t = df["t"]
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(t, df["current_x"], label="current_x")
    axs[0].plot(t, df["desired_x"], "--", label="desired_x")
    axs[0].set_ylabel("X (m)"); axs[0].legend(); axs[0].grid()

    axs[1].plot(t, df["current_y"], label="current_y")
    axs[1].plot(t, df["desired_y"], "--", label="desired_y")
    axs[1].set_ylabel("Y (m)"); axs[1].legend(); axs[1].grid()

    ax_z = axs[2]
    ax_z.plot(t, df["current_z"], label="current_z")
    ax_z.plot(t, df["desired_z"], "--", label="desired_z")
    ax_z.set_ylabel("Z (m)"); ax_z.set_xlabel("Time (s)")
    ax_z.legend(); ax_z.grid()

    # Optional overlay of discrete jumping_state
    if "jumping_state" in df.columns:
        ax_js = ax_z.twinx()
        ax_js.step(t, df["jumping_state"], where="post", label="jumping_state", linewidth=1.2)
        ax_js.set_ylabel("Jump State")
        states = sorted(pd.Series(df["jumping_state"]).dropna().unique())
        if len(states) > 0:
            ax_js.set_yticks(states)
            ax_js.set_ylim(min(states) - 0.2, max(states) + 0.2)

    if x_limit:
        for ax in axs: ax.set_xlim(0, x_limit)

    plt.suptitle("Position (XYZ) vs Time")
    plt.tight_layout()


def plot_velocity(df: pd.DataFrame, x_limit: float | None = None):
    req = ["vel_x", "vel_y", "vel_z"]
    miss = missing(df, req)
    if miss:
        print(f"⚠ Skipping velocity: missing {miss}")
        return

    t = df["t"]
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t, df["vel_x"], label="vel_x"); axs[0].set_ylabel("Vel X (m/s)"); axs[0].legend(); axs[0].grid()
    axs[1].plot(t, df["vel_y"], label="vel_y"); axs[1].set_ylabel("Vel Y (m/s)"); axs[1].legend(); axs[1].grid()
    axs[2].plot(t, df["vel_z"], label="vel_z"); axs[2].set_ylabel("Vel Z (m/s)"); axs[2].set_xlabel("Time (s)"); axs[2].legend(); axs[2].grid()

    if x_limit:
        for ax in axs: ax.set_xlim(0, x_limit)

    plt.suptitle("Velocity vs Time")
    plt.tight_layout()


def plot_rpyt(df: pd.DataFrame, x_limit: float | None = None):
    req = ["current_roll", "Roll", "current_pitch", "Pitch", "current_yaw", "Desired_yaw", "Thrust"]
    miss = missing(df, req)
    if miss:
        print(f"⚠ Skipping RPYT: missing {miss}")
        return

    t = df["t"]
    fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    axs[0].plot(t, df["current_roll"], label="current_roll")
    axs[0].plot(t, df["Roll"], "--", label="roll_cmd"); axs[0].set_ylabel("Roll"); axs[0].legend(); axs[0].grid()

    axs[1].plot(t, df["current_pitch"], label="current_pitch")
    axs[1].plot(t, df["Pitch"], "--", label="pitch_cmd"); axs[1].set_ylabel("Pitch"); axs[1].legend(); axs[1].grid()

    axs[2].plot(t, df["current_yaw"], label="current_yaw")
    axs[2].plot(t, df["Desired_yaw"], "--", label="yaw_cmd"); axs[2].set_ylabel("Yaw"); axs[2].legend(); axs[2].grid()

    axs[3].plot(t, df["Thrust"], label="Thrust"); axs[3].set_ylabel("Thrust"); axs[3].set_xlabel("Time (s)")
    axs[3].legend(); axs[3].grid()

    if x_limit:
        for ax in axs: ax.set_xlim(0, x_limit)

    plt.suptitle("RPYT vs Time")
    plt.tight_layout()


def plot_accel(df: pd.DataFrame, x_limit: float | None = None):
    req = ["acc_x", "current_x", "acc_y", "current_y", "acc_z", "current_z"]
    miss = missing(df, req)
    if miss:
        print(f"⚠ Skipping accel: missing {miss}")
        return

    t = df["t"]
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # X
    ax1 = axs[0]; ax2 = ax1.twinx()
    ax1.plot(t, df["acc_x"], label="acc_x (G)"); ax2.plot(t, df["current_x"], "--", label="current_x (m)")
    ax1.set_ylabel("Acc X (G)"); ax2.set_ylabel("Pos X (m)"); ax1.legend(loc="upper left"); ax2.legend(loc="upper right"); ax1.grid()

    # Y
    ax1 = axs[1]; ax2 = ax1.twinx()
    ax1.plot(t, df["acc_y"], label="acc_y (G)"); ax2.plot(t, df["current_y"], "--", label="current_y (m)")
    ax1.set_ylabel("Acc Y (G)"); ax2.set_ylabel("Pos Y (m)"); ax1.legend(loc="upper left"); ax2.legend(loc="upper right"); ax1.grid()

    # Z
    ax1 = axs[2]; ax2 = ax1.twinx()
    ax1.plot(t, df["acc_z"], label="acc_z (G)"); ax2.plot(t, df["current_z"], "--", label="current_z (m)")
    ax1.set_ylabel("Acc Z (G)"); ax2.set_ylabel("Pos Z (m)"); ax1.set_xlabel("Time (s)")
    ax1.legend(loc="upper left"); ax2.legend(loc="upper right"); ax1.grid()

    if x_limit:
        for ax in axs: ax.set_xlim(0, x_limit)

    plt.suptitle("Acceleration and Position vs Time")
    plt.tight_layout()


def plot_motors(df: pd.DataFrame, x_limit: float | None = None):
    req = ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]
    miss = missing(df, req)
    if miss:
        print(f"⚠ Skipping motors: missing {miss}")
        return

    t = df["t"]
    fig, ax1 = plt.subplots(figsize=(10, 6))
    ax1.plot(t, df["motor_m1"], label="Motor 1")
    ax1.plot(t, df["motor_m2"], label="Motor 2")
    ax1.plot(t, df["motor_m3"], label="Motor 3")
    ax1.plot(t, df["motor_m4"], label="Motor 4")
    ax1.set_xlabel("Time (s)"); ax1.set_ylabel("Motor value"); ax1.legend(loc="upper left"); ax1.grid()

    # Optional: overlay JSTO.powered_climbing_end_flag if present
    if "JSTO.powered_climbing_end_flag" in df.columns:
        ax2 = ax1.twinx()
        ax2.step(t, df["JSTO.powered_climbing_end_flag"], where="post", label="Powered Climbing End", linewidth=1.2)
        ax2.set_ylabel("Powered Climbing End"); ax2.set_yticks([0, 1]); ax2.set_yticklabels(["False", "True"])
        # merge legends
        l1, lab1 = ax1.get_legend_handles_labels()
        l2, lab2 = ax2.get_legend_handles_labels()
        ax1.legend(l1 + l2, lab1 + lab2, loc="upper right")

    if x_limit:
        ax1.set_xlim(0, x_limit)

    plt.title("Motor Commands vs Time")
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description="Plot Crazyflie-style CSV.")
    parser.add_argument("--csv", required=True, help="input CSV file")
    parser.add_argument("--plots", nargs="+", default=["all"],
                        choices=["all", "position", "velocity", "rpyt", "accel", "motors"],
                        help="which plots to show")
    parser.add_argument("--xlimit", type=float, default=None, help="limit x-axis to N seconds")
    args = parser.parse_args()

    csv_path = Path(args.csv).expanduser()
    if not csv_path.is_file():
        parser.error(f"CSV not found: {csv_path}")

    df = load_csv(csv_path)
    print(f"✔ Loaded CSV: {csv_path}  ({len(df)} rows)")

    do_all = "all" in args.plots
    if do_all or "position" in args.plots: plot_position(df, args.xlimit)
    if do_all or "velocity" in args.plots: plot_velocity(df, args.xlimit)
    if do_all or "rpyt"     in args.plots: plot_rpyt(df, args.xlimit)
    if do_all or "accel"    in args.plots: plot_accel(df, args.xlimit)
    if do_all or "motors"   in args.plots: plot_motors(df, args.xlimit)

    plt.show()


if __name__ == "__main__":
    main()
