#!/usr/bin/env python3
"""
Convert Crazyflie-style log → CSV (+ optional plots).

Usage
-----
Run with explicit files …

    python log2csv_and_plots_with_velocity.py --log raw_log.txt --csv clean_log.csv --plots

… or just let the script pick the **most-recent .txt file** in the current
directory and make a matching CSV:

    python log2csv_and_plots_with_velocity.py --plots

New in this version
-------------------
* Gracefully handles *additional* key:value pairs in each log line (e.g.,
  new `vel_x`, `vel_y`, `vel_z` fields). The parser was already generic; no
  structural change was required—just documentation and a couple of defensive
  checks.
* Added `LogPlotter.plot_velocity()` to show X/Y/Z velocities vs time.
* CLI `--plots` now shows Position, Velocity (if present), RPYT, and Accel.

Assumptions about the log format
--------------------------------
Each line contains comma-separated `key: value` pairs, e.g.::

    2025-07-17 21:14:03,123 - UNIX Time: 1.234, current_x: 0.1, vel_x: 0.02, ...

Everything *after* the first literal `" - "` is parsed. Unknown extra fields
are captured automatically and become new DataFrame columns. Therefore adding
`vel_x`, `vel_y`, `vel_z` (or anything else) is fine as long as the syntax is
`name: value` and fields are separated by commas.
"""

import re
import csv
import argparse
from pathlib import Path
from typing import Iterable

import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# 1. ---------------  LOG-FILE ➜  DATA-FRAME  -------------------------------
# ---------------------------------------------------------------------------

PAIR_RE = re.compile(r"([^:,]+):\s*([^:,]+)")  # key: value  (comma-separated)


def load_log(path: Path) -> pd.DataFrame:
    """Parse the log, preserve key order, add elapsed-time column 't'.

    The parser is *field-agnostic*: any `key: value` pair will be captured.
    Thus, adding new variables (e.g., vel_x/vel_y/vel_z) requires no change.
    """
    ordered_keys: list[str] = []
    records: list[dict[str, str]] = []

    with path.open() as fh:
        for raw in fh:
            raw = raw.strip()
            if not raw:
                continue

            # Strip the leading timestamp (“YYYY-MM-DD … - ”)
            if " - " in raw:
                raw = raw.split(" - ", maxsplit=1)[1]

            kv_pairs = PAIR_RE.findall(raw)
            if not kv_pairs:
                continue  # skip malformed lines

            # Track first-appearance order
            for k, _ in kv_pairs:
                k = k.strip()
                if k not in ordered_keys:
                    ordered_keys.append(k)

            records.append({k.strip(): v.strip() for k, v in kv_pairs})

    if not records:
        raise ValueError(f"No parsable data in log: {path}")

    # Build DataFrame with columns in captured order
    df = pd.DataFrame(records, columns=ordered_keys)

    # Convert numerics where possible
    # NOTE: errors='ignore' is deprecated; 'coerce' turns non-numeric into NaN.
    df = df.apply(lambda s: pd.to_numeric(s, errors="coerce"))

    # Validate required column(s)
    if "UNIX Time" not in df.columns:
        raise KeyError("Required column 'UNIX Time' not found in log.")

    # Elapsed-time column (seconds)
    df["t"] = df["UNIX Time"] - df["UNIX Time"].iloc[0]

    # Put 't' first, drop absolute UNIX Time
    cols = ["t"] + [c for c in ordered_keys if c != "UNIX Time"]
    df = df[cols]
    return df


# ---------------------------------------------------------------------------
# 2. -------------------  PLOTTER CLASS  ------------------------------------
# ---------------------------------------------------------------------------

class LogPlotter:
    """Quick-look plots for Position, Velocity, RPYT, and Acceleration."""

    def __init__(self, df: pd.DataFrame):
        self.d = df
        # Always keep a NumPy copy for plotting to avoid pandas [:, None] issues
        self.t = df["t"].to_numpy()

    # Small helper ---------------------------------------------------------
    def _missing(self, cols: Iterable[str]) -> list[str]:
        return [c for c in cols if c not in self.d.columns]

    def _warn_if_missing(self, cols: Iterable[str]) -> bool:
        miss = self._missing(cols)
        if miss:
            print(f"⚠ Skipping plot; missing columns: {', '.join(miss)}")
            return True
        return False

    def _np(self, col: str):
        return self.d[col].to_numpy()

    # ----- Position -------------------------------------------------------
    def plot_position(self, x_limit: float | None = None):
        """
        Plot current vs desired XYZ.  
        If a 'jumping_state' column exists, overlay it (step plot) on the Z-axis panel.
        """
        req = ["current_x", "desired_x",
               "current_y", "desired_y",
               "current_z", "desired_z"]
        if self._warn_if_missing(req):
            return

        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        # -------- X ------------------------------------------------------
        axs[0].plot(self.t, self._np("current_x"), label="current_x")
        axs[0].plot(self.t, self._np("desired_x"), "--", label="desired_x")
        axs[0].set_ylabel("X (m)")
        axs[0].legend(); axs[0].grid()

        # -------- Y ------------------------------------------------------
        axs[1].plot(self.t, self._np("current_y"), label="current_y")
        axs[1].plot(self.t, self._np("desired_y"), "--", label="desired_y")
        axs[1].set_ylabel("Y (m)")
        axs[1].legend(); axs[1].grid()

        # -------- Z  (+ Jumping-State) -----------------------------------
        ax_z = axs[2]
        ax_z.plot(self.t, self._np("current_z"), label="current_z")
        ax_z.plot(self.t, self._np("desired_z"), "--", label="desired_z")
        ax_z.set_ylabel("Z (m)")
        ax_z.set_xlabel("Time (s)")
        ax_z.legend(loc="upper left"); ax_z.grid()

        # Overlay jumping_state if present
        if "jumping_state" in self.d.columns:
            ax_js = ax_z.twinx()                      # secondary y-axis
            ax_js.step(self.t, self._np("jumping_state"),
                       where="post", label="jumping_state", linewidth=1.2)
            ax_js.set_ylabel("Jump State")
            # Tidy the y-ticks so they show discrete states
            unique_states = sorted(pd.Series(self.d["jumping_state"]).dropna().unique())
            if len(unique_states):
                ax_js.set_yticks(unique_states)
                ax_js.set_ylim(min(unique_states) - 0.2,
                               max(unique_states) + 0.2)
            # Combine legends (left = position traces, right = state)
            ax_js.legend(loc="upper right")

        # -------- Common tweaks -----------------------------------------
        if x_limit:
            for ax in axs:
                ax.set_xlim(0, x_limit)

        plt.suptitle("Current Position (XYZ) + Jumping State vs Time")
        plt.tight_layout()

    # ----- Velocity -------------------------------------------------------
    def plot_velocity(self, x_limit: float | None = None):
        """Plot vel_x/vel_y/vel_z vs time (if present)."""
        req = ["vel_x", "vel_y", "vel_z"]
        if self._warn_if_missing(req):
            return
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        axs[0].plot(self.t, self._np("vel_x"), label="vel_x")
        axs[0].set_ylabel("Vel X (m/s)"); axs[0].legend(); axs[0].grid()
        axs[1].plot(self.t, self._np("vel_y"), label="vel_y")
        axs[1].set_ylabel("Vel Y (m/s)"); axs[1].legend(); axs[1].grid()
        axs[2].plot(self.t, self._np("vel_z"), label="vel_z")
        axs[2].set_ylabel("Vel Z (m/s)"); axs[2].set_xlabel("Time (s)")
        axs[2].legend(); axs[2].grid()
        if x_limit:
            for ax in axs: ax.set_xlim(0, x_limit)
        plt.suptitle("Velocity vs Time"); plt.tight_layout()

    # ----- RPYT -----------------------------------------------------------
    def plot_rpyt(self, x_limit: float | None = None):
        req = ["current_roll", "Roll", "current_pitch", "Pitch", "current_yaw", "Desired_yaw", "Thrust"]
        if self._warn_if_missing(req):
            return
        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        axs[0].plot(self.t, self._np("current_roll"), label="current_roll")
        axs[0].plot(self.t, self._np("Roll"), "--", label="roll_command")
        axs[0].set_ylabel("Roll"); axs[0].legend(); axs[0].grid()
        axs[1].plot(self.t, self._np("current_pitch"), label="current_pitch")
        axs[1].plot(self.t, self._np("Pitch"), "--", label="pitch_command")
        axs[1].set_ylabel("Pitch"); axs[1].legend(); axs[1].grid()
        axs[2].plot(self.t, self._np("current_yaw"), label="current_yaw")
        axs[2].plot(self.t, self._np("Desired_yaw"), "--", label="yaw_command")
        axs[2].set_ylabel("Yaw"); axs[2].legend(); axs[2].grid()
        axs[3].plot(self.t, self._np("Thrust"), label="thrust_command")
        axs[3].set_ylabel("Thrust"); axs[3].set_xlabel("Time (s)")
        axs[3].legend(); axs[3].grid()
        if x_limit:
            for ax in axs: ax.set_xlim(0, x_limit)
        plt.suptitle("RPYT vs Time"); plt.tight_layout()

    # ----- Acceleration ---------------------------------------------------
    def plot_accel(self, x_limit: float | None = None):
        req = ["acc_x", "current_x", "acc_y", "current_y", "acc_z", "current_z"]
        if self._warn_if_missing(req):
            return
        fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

        # Acceleration and Position X
        ax1 = axs[0]
        ax2 = ax1.twinx()  # Create secondary y-axis
        ax1.plot(self.t, self._np("acc_x"), label="acc_x (G's)")
        ax2.plot(self.t, self._np("current_x"), "--", label="current_x (m)")
        ax1.set_ylabel("Acc X (G's)")
        ax2.set_ylabel("Pos X (m)")
        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.grid()

        # Acceleration and Position Y
        ax1 = axs[1]
        ax2 = ax1.twinx()  # Create secondary y-axis
        ax1.plot(self.t, self._np("acc_y"), label="acc_y (G's)")
        ax2.plot(self.t, self._np("current_y"), "--", label="current_y (m)")
        ax1.set_ylabel("Acc Y (G's)")
        ax2.set_ylabel("Pos Y (m)")
        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.grid()

        # Acceleration and Position Z
        ax1 = axs[2]
        ax2 = ax1.twinx()  # Create secondary y-axis
        ax1.plot(self.t, self._np("acc_z"), label="acc_z (G's)")
        ax2.plot(self.t, self._np("current_z"), "--", label="current_z (m)")
        ax1.set_ylabel("Acc Z (G's)")
        ax2.set_ylabel("Pos Z (m)")
        ax1.set_xlabel("Time (s)")
        ax1.legend(loc="upper left")
        ax2.legend(loc="upper right")
        ax1.grid()

        if x_limit:
            for ax in axs:
                ax.set_xlim(0, x_limit)

        plt.suptitle("Acceleration and Position vs Time")
        plt.tight_layout()

    # ----- Motors --------------------------------------------------------
    def plot_motors(self, x_limit: float | None = None):
        """Plot all four motor command/Power traces on one graph, with JSTO.powered_climbing_end_flag."""
        req = ["motor_m1", "motor_m2", "motor_m3", "motor_m4", "JSTO.powered_climbing_end_flag"]
        if self._warn_if_missing(req):
            return

        fig, ax1 = plt.subplots(figsize=(10, 6))

        # Plot motor values on the primary y-axis
        ax1.plot(self.t, self._np("motor_m1"), label="Motor 1")
        ax1.plot(self.t, self._np("motor_m2"), label="Motor 2")
        ax1.plot(self.t, self._np("motor_m3"), label="Motor 3")
        ax1.plot(self.t, self._np("motor_m4"), label="Motor 4")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Motor value (units as in log)")
        ax1.legend(loc="upper left")
        ax1.grid()

        # Add a secondary y-axis for JSTO.powered_climbing_end_flag
        ax2 = ax1.twinx()
        ax2.step(self.t, self._np("JSTO.powered_climbing_end_flag"), where="post", label="Powered Climbing End Flag")
        ax2.set_ylabel("Powered Climbing End Flag")
        ax2.set_yticks([0, 1])  # True/False as 0/1
        ax2.set_yticklabels(["False", "True"])

        # Combine legends
        lines_1, labels_1 = ax1.get_legend_handles_labels()
        lines_2, labels_2 = ax2.get_legend_handles_labels()
        ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="upper right")

        # Set x-axis limit if specified
        if x_limit is not None:
            ax1.set_xlim(0, x_limit)

        plt.title("Motor Commands and Powered Climbing End Flag vs Time")
        plt.tight_layout()

    # ----- Jumping-State -----------------------------------------------
    def plot_jumping_state(self, x_limit: float | None = None):
        """Discrete jumping_state (e.g. 0 = stance, 1 = flight) vs time."""
        req = ["jumping_state"]
        if self._warn_if_missing(req):
            return

        plt.figure(figsize=(10, 4))
        # Step plot so the state looks like a digital signal
        plt.step(self.t, self._np("jumping_state"), where="post", label="jumping_state")

        plt.xlabel("Time (s)")
        plt.ylabel("State")
        plt.title("Jumping State vs Time")
        ticks = sorted(pd.Series(self.d["jumping_state"]).dropna().unique())
        if len(ticks):
            plt.yticks(ticks)  # tidy y-axis
        plt.legend()
        plt.grid()

        if x_limit is not None:
            plt.xlim(0, x_limit)

        plt.tight_layout()

# ---------------------------------------------------------------------------
# 3. ------------------------------  CLI  -----------------------------------
# ---------------------------------------------------------------------------

def find_latest_txt() -> Path | None:
    """Return the most recently modified *.txt file in cwd, or None."""
    txts = list(Path.cwd().glob("*.txt"))
    return max(txts, key=lambda p: p.stat().st_mtime) if txts else None


def main():
    parser = argparse.ArgumentParser(
        description="Convert Crazyflie log ➜ CSV (and plots)."
    )
    parser.add_argument("--log", help="input log (.txt) file")
    parser.add_argument("--csv", help="output CSV (for Excel)")
    parser.add_argument("--plots", action="store_true",
                        help="display position + velocity + RPYT + accel plots")
    args = parser.parse_args()

    # -------- Determine log file ----------------------------------------
    if args.log:
        log_path = Path(args.log).expanduser()
        if not log_path.is_file():
            parser.error(f"Log file not found: {log_path}")
    else:
        latest = find_latest_txt()
        if latest is None:
            parser.error("No .txt log file found in current directory "
                         "and --log not supplied.")
        log_path = latest
        print(f"🛈 No --log supplied → using latest file: {log_path.name}")

    # -------- Determine CSV name ----------------------------------------
    if args.csv:
        csv_path = Path(args.csv).expanduser()
    else:
        csv_path = log_path.with_suffix(".csv")
        print(f"🛈 No --csv supplied → writing to: {csv_path.name}")

    # -------- Convert ----------------------------------------------------
    df = load_log(log_path)
    df.to_csv(csv_path, index=False, quoting=csv.QUOTE_MINIMAL)
    print(f"✔ CSV written: {csv_path}  ({len(df)} rows)")

    # -------- Optional plots --------------------------------------------
    if args.plots:
        lp = LogPlotter(df)
        lp.plot_position()
        lp.plot_velocity()  # <-- new
        lp.plot_rpyt()
        lp.plot_accel()
        lp.plot_motors()
        # lp.plot_jumping_state()
        plt.show()


if __name__ == "__main__":
    main()
