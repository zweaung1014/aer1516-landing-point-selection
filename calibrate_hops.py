#!/usr/bin/env python3
"""
Stage 0 calibration from one hopcopter run (see docs/follow_planner_plan.md, stage 0).

Reads the per-tick CSV that hopcopter writes to position_attitude_tracking/ and reports:
  - real-time factor: sim time (odom/IMU stamps) against wall time
  - g': effective gravity in unpowered flight (idle thrust), from quadratic fits of z(t)
  - eta: stance energy ratio (v_liftoff / v_touchdown)^2, per stance, with the landing
    speed, landing tilt and redirection angle
  - odom twist frame and window: which of world / heading / body frame, differentiated over
    which window W, matches the twist
  - push thrust: IMU specific force during pushes against the motor model
  - command-to-thrust latency: push cutoff tick against the IMU specific-force drop
  - /cf_1/pose against Gazebo ground truth
  - the matching apex_decisions_<ts>.csv, one row per apex

Usage:
    python calibrate_hops.py                          # latest file in position_attitude_tracking/
    python calibrate_hops.py attitude_<timestamp>.csv # a file in that directory, by name
    python calibrate_hops.py path/to/specific.csv     # the specified file

Contacts come from the IMU: the leg spring is preloaded (~40 N), so stance reads ~11 g or
more while a push reads under 1 g. All kinematics use sim-time stamps, not wall time.
"""

import os
import sys
import glob
import argparse
from datetime import datetime

import numpy as np
import pandas as pd

# ──────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────
REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
ATT_DIR = os.path.join(REPO_ROOT, "data", "data_ballistic_planner", "position_attitude_tracking")
DEC_DIR = os.path.join(REPO_ROOT, "data", "data_ballistic_planner", "hop_tracking")

G = 9.8              # Gazebo gravity [m/s^2]
MASS = 0.3609        # robot mass in the model [kg]
K_MOTOR = 28e-8      # motorConstant in model.sdf.jinja
STANCE_AZ = 50.0     # [m/s^2] stance reads >= ~110 (preloaded leg), a push < 10
MIN_STANCE_TICKS = 3
SPIN_DOWN = 0.08     # [s] skipped after a push before fitting unpowered flight
ODOM_DT = 0.005      # odom period (200 Hz); diff_v spans one period
TWIST_WINDOW = 0.05  # longest twist window tested (10 odom periods)


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


def file_time(path: str):
    """Parse the <YYYY-mm-dd_HH-MM-SS> suffix hopcopter puts in its CSV names."""
    stem = os.path.splitext(os.path.basename(path))[0]
    try:
        return datetime.strptime(stem[-19:], "%Y-%m-%d_%H-%M-%S")
    except ValueError:
        return None


def matching_decisions(att_path: str):
    """The apex_decisions CSV opened by the same hopcopter run (names within 2 s)."""
    t_att = file_time(att_path)
    if t_att is None:
        return None
    best = None
    for p in glob.glob(os.path.join(DEC_DIR, "apex_decisions_*.csv")):
        t = file_time(p)
        if t is not None and abs((t - t_att).total_seconds()) <= 2:
            best = p
    return best


def motor_force(pwm):
    """Thrust of one motor [N] from its PWM (Gazebo MulticopterMotorModel)."""
    pwm = np.asarray(pwm, dtype=float)
    omega = 0.04076521 * pwm + 380.8359
    return np.where(pwm >= 1000, K_MOTOR * omega * omega, 0.0)


def stats(values, fmt="{:.3f}"):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if len(v) == 0:
        return "n=0"
    return (f"median {fmt.format(np.median(v))}, mean {fmt.format(v.mean())}, "
            f"min {fmt.format(v.min())}, max {fmt.format(v.max())}, n={len(v)}")


def rot_zyx(yaw, pitch, roll):
    """Body-to-world rotation for scipy's as_euler('zyx') angles [rad]."""
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


# ──────────────────────────────────────────────────────────────────────
# ANALYSES
# ──────────────────────────────────────────────────────────────────────

def find_stances(az):
    """(first stance tick, first airborne tick after it) for each contact."""
    st = az > STANCE_AZ
    events, dropped = [], 0
    i, n = 1, len(az)
    while i < n:
        if st[i] and not st[i - 1]:
            j = i
            while j < n and st[j]:
                j += 1
            if j >= n:
                break
            if j - i >= MIN_STANCE_TICKS:
                events.append((i, j))
            else:
                dropped += 1
            i = j
        i += 1
    return events, dropped


def main():
    parser = argparse.ArgumentParser(description="Stage 0 calibration from a hopcopter run.")
    parser.add_argument("file", nargs="?", default=None,
                        help="CSV path or filename in position_attitude_tracking/ (default: latest)")
    args = parser.parse_args()

    try:
        path = resolve_csv(args.file) if args.file else latest_csv(ATT_DIR)
    except FileNotFoundError as e:
        sys.exit(f"Error: {e}")

    df = pd.read_csv(path)
    if "odom_stamp" not in df.columns:
        sys.exit(f"{os.path.basename(path)} predates the stage 0 columns (no odom_stamp).")
    df = df[(df.odom_stamp > 0) & (df.imu_stamp > 0)].reset_index(drop=True)
    if len(df) < 200:
        sys.exit("Too few ticks with odom and IMU data.")
    print(f"File: {path}  ({len(df)} ticks)")

    wall = df.timestamp.to_numpy()
    t_imu = df.imu_stamp.to_numpy()
    az = df.imu_az.to_numpy()
    push = df.push.to_numpy().astype(bool)
    roll = np.radians(df.act_roll_deg.to_numpy())
    pitch = np.radians(-df.act_pitch_deg.to_numpy())   # logged act_pitch is -euler_y
    yaw = np.radians(df.act_yaw_deg.to_numpy())
    cos_tilt = np.cos(roll) * np.cos(pitch)

    # Unique odom samples (a tick repeats the previous one when no new odom arrived)
    first = ~df.odom_stamp.duplicated().to_numpy()
    od_tick = np.flatnonzero(first)
    od_t = df.odom_stamp.to_numpy()[first]
    od_p = df[["pos_x", "pos_y", "pos_z"]].to_numpy()[first]
    od_dv = df[["diff_vx", "diff_vy", "diff_vz"]].to_numpy()[first]
    od_tw = df[["odom_vx", "odom_vy", "odom_vz"]].to_numpy()[first]

    # ── Real-time factor ──
    print("\n== Real-time factor (sim / wall)")
    rtf = (od_t[-1] - od_t[0]) / (wall[od_tick[-1]] - wall[od_tick[0]])
    win = []
    for k in range(0, len(od_tick) - 200, 200):
        dw = wall[od_tick[k + 200]] - wall[od_tick[k]]
        if dw > 0:
            win.append((od_t[k + 200] - od_t[k]) / dw)
    print(f"  whole run {rtf:.3f};  per ~2 s window: {stats(win)}")

    # ── Contacts ──
    stances, dropped = find_stances(az)
    js = df.jumping_state.to_numpy()
    n_12 = int(np.sum((js[:-1] == 1) & (js[1:] == 2)))
    n_32 = int(np.sum((js[:-1] == 3) & (js[1:] == 2)))
    print(f"\n== Contacts: {len(stances)} stances from the IMU "
          f"({dropped} blips under {MIN_STANCE_TICKS} ticks dropped); "
          f"JSTO logged {n_12} landings (1->2) and {n_32} 3->2 transitions")
    if len(stances) < 2:
        sys.exit("Need at least two stances.")

    def t_td(s):   # touchdown: between the last airborne and the first stance IMU sample
        return 0.5 * (t_imu[s[0] - 1] + t_imu[s[0]])

    def t_lo(s):   # liftoff: between the last stance and the first airborne IMU sample
        return 0.5 * (t_imu[s[1] - 1] + t_imu[s[1]])

    def push_end_before(i0, i1):
        """Sim time a push ended in ticks [i0, i1), or None without a push."""
        idx = np.flatnonzero(push[i0:i1])
        return None if len(idx) == 0 else t_imu[min(i0 + idx[-1] + 1, len(t_imu) - 1)]

    # ── g' and touchdown velocities, per flight ──
    g_fit, g_model, g_tilt, flights, rejected = [], [], [], [], 0
    for k in range(len(stances)):
        td = stances[k][0]
        if k == 0:
            lo_tick, t_start = 0, t_imu[0]
        else:
            lo_tick, t_start = stances[k - 1][1], t_lo(stances[k - 1])
        pe = push_end_before(lo_tick, td)
        t_a = max(t_start + 0.02, (pe + SPIN_DOWN) if pe is not None else -np.inf)
        t_b = t_imu[td - 1]                       # last IMU sample known airborne
        sel = (od_t >= t_a) & (od_t <= t_b)
        if np.sum(sel) < 10 or t_b - t_a < 0.12:
            flights.append(None)
            continue
        tt = od_t[sel] - t_td(stances[k])
        cz = np.polyfit(tt, od_p[sel, 2], 2)
        cx = np.polyfit(tt, od_p[sel, 0], 2)
        cy = np.polyfit(tt, od_p[sel, 1], 2)
        # Not ballistic (e.g. lying on the ground after a crash, which reads ~1 g): skip
        resid = od_p[sel, 2] - np.polyval(cz, tt)
        if not 7.0 < -2.0 * cz[0] < 12.0 or np.sqrt(np.mean(resid ** 2)) > 0.005:
            flights.append(None)
            rejected += 1
            continue
        g_fit.append(-2.0 * cz[0])
        rows = od_tick[sel]
        f_up = motor_force(df.loc[rows, ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]].to_numpy()).sum(axis=1)
        g_model.append(G - np.mean(f_up * cos_tilt[rows]) / MASS)
        g_tilt.append(np.degrees(np.arccos(np.clip(cos_tilt[rows].min(), -1, 1))))
        # velocity at touchdown, from the fit (unpowered all the way down)
        flights.append(np.array([cx[1], cy[1], cz[1]]))

    print("\n== g' (unpowered flight, quadratic fit of z on sim time)")
    print(f"  fitted:          {stats(g_fit)}   ({rejected} non-ballistic segments rejected)")
    print(f"  from motor PWMs: {stats(g_model)}   (G - idle thrust * cos(tilt) / m)")
    g_fit, g_tilt = np.array(g_fit), np.array(g_tilt)
    print(f"  level falls (tilt < 5 deg):    {stats(g_fit[g_tilt < 5])}")
    print(f"  steered falls (tilt >= 5 deg): {stats(g_fit[g_tilt >= 5])}   "
          f"(attitude control adds thrust)")

    # ── eta per stance ──
    print("\n== Stance: eta = (v_liftoff / v_touchdown)^2")
    print(f"  {'hop':>3} {'t_sim':>8} {'v_td':>6} {'v_lo':>6} {'eta':>6} "
          f"{'tilt_td':>7} {'redir':>6} {'v_h_td':>6} {'v_h_lo':>6} {'stance_s':>8}")
    etas, rows_out = [], []
    for k, s in enumerate(stances):
        v_td = flights[k]
        if v_td is None:
            continue
        tl = t_lo(s)
        cand = np.flatnonzero(od_t >= t_imu[s[1]] + ODOM_DT)
        if len(cand) == 0:
            continue
        j = cand[0]
        if od_t[j] - tl > 0.025:
            continue
        tick = od_tick[j]
        v_lo = od_dv[j].copy()                   # mean over [stamp - 5 ms, stamp]
        a_z = az[tick] * cos_tilt[tick] - G      # IMU says how much thrust is already on
        v_lo[2] -= a_z * (od_t[j] - 0.5 * ODOM_DT - tl)
        s_td, s_lo = np.linalg.norm(v_td), np.linalg.norm(v_lo)
        eta = (s_lo / s_td) ** 2
        redir = np.degrees(np.arccos(np.clip(np.dot(-v_td, v_lo) / (s_td * s_lo), -1, 1)))
        tilt = np.degrees(np.arccos(np.clip(cos_tilt[s[0]], -1, 1)))
        etas.append(eta)
        rows_out.append((k + 1, t_td(s), s_td, s_lo, eta, tilt, redir,
                         np.hypot(v_td[0], v_td[1]), np.hypot(v_lo[0], v_lo[1]), tl - t_td(s)))
        print(f"  {k + 1:>3} {t_td(s):8.2f} {s_td:6.2f} {s_lo:6.2f} {eta:6.3f} "
              f"{tilt:7.1f} {redir:6.1f} {rows_out[-1][7]:6.2f} {rows_out[-1][8]:6.2f} "
              f"{rows_out[-1][9]:8.3f}")
    print(f"  eta: {stats(etas)}")
    if rows_out:
        r = np.array(rows_out)
        fast, slow = r[:, 2] >= np.median(r[:, 2]), r[:, 2] < np.median(r[:, 2])
        tilted = r[:, 5] >= 5.0
        print(f"  eta, faster half of landings: {stats(r[fast, 4])}")
        print(f"  eta, slower half of landings: {stats(r[slow, 4])}")
        print(f"  eta, landings tilted >= 5 deg: {stats(r[tilted, 4])}")

    # ── Odom twist frame and window ──
    print("\n== Odom twist vs velocity differentiated over the last W (airborne, >= 55 ms after liftoff)")
    air = np.zeros(len(od_t), dtype=bool)
    bounds = [(t_imu[0], t_td(stances[0]))]
    bounds += [(t_lo(stances[k - 1]), t_td(stances[k])) for k in range(1, len(stances))]
    for (a, b), v_td in zip(bounds, flights):
        if v_td is not None:   # ballistic flights only
            air |= (od_t >= a + TWIST_WINDOW + ODOM_DT) & (od_t <= b - ODOM_DT)
    idx = np.flatnonzero(air)
    if len(idx) > 20:
        tw = od_tw[idx]
        ticks = od_tick[idx]
        rot = [rot_zyx(yaw[tk], pitch[tk], roll[tk]) for tk in ticks]
        print("  RMS residual [m/s]   world frame (x y z)        heading frame (x y z)      body frame (x y z)")
        for w in (ODOM_DT, 2 * ODOM_DT, 4 * ODOM_DT, TWIST_WINDOW):
            # a window mean of 5 ms differences telescopes to (p(t) - p(t - W)) / W
            p_back = np.column_stack([np.interp(od_t[idx] - w, od_t, od_p[:, c]) for c in range(3)])
            v_w = (od_p[idx] - p_back) / w
            v_h = np.array([[np.cos(yaw[tk]) * v[0] + np.sin(yaw[tk]) * v[1],
                             -np.sin(yaw[tk]) * v[0] + np.cos(yaw[tk]) * v[1], v[2]]
                            for v, tk in zip(v_w, ticks)])
            v_b = np.array([r.T @ v for r, v in zip(rot, v_w)])
            cells = []
            for ref in (v_w, v_h, v_b):
                rms = np.sqrt(np.mean((tw - ref) ** 2, axis=0))
                cells.append(f"{rms[0]:.4f} {rms[1]:.4f} {rms[2]:.4f}")
            print(f"  W = {w * 1000:2.0f} ms          " + "     ".join(cells))
        tilt_air = np.degrees(np.arccos(np.clip(cos_tilt[ticks], -1, 1)))
        print(f"  (over {len(idx)} samples; tilt {stats(tilt_air, '{:.1f}')} deg; "
              f"|yaw| max {np.degrees(np.max(np.abs(yaw[ticks]))):.1f} deg)")

    # ── Push thrust and cutoff latency ──
    print("\n== Pushes")
    starts = np.flatnonzero(push[1:] & ~push[:-1]) + 1
    ends = np.flatnonzero(~push[1:] & push[:-1]) + 1
    acc_meas, acc_cmd, acc_mot, lat_imu, lat_pwm, lens = [], [], [], [], [], []
    for s0 in starts:
        e = ends[ends > s0]
        if len(e) == 0:
            continue
        e = e[0]
        lens.append(t_imu[e] - t_imu[s0])
        steady = np.arange(s0 + 3, e)                 # skip the spin-up
        if len(steady) < 3:
            continue
        a_push = np.median(az[steady])
        acc_meas.append(a_push)
        cmd = df.thrust_cmd.to_numpy()[steady[0]]
        acc_cmd.append(4 * motor_force(cmd) / MASS)
        acc_mot.append(np.median(motor_force(
            df.loc[steady, ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]].to_numpy()).sum(axis=1)) / MASS)
        # cutoff latency: first IMU sample below halfway to idle, unless a stance comes first
        a_idle = 4 * motor_force(1000) / MASS
        mid = 0.5 * (a_push + a_idle)
        after = np.arange(e, min(e + 15, len(az)))
        below = after[az[after] < mid]
        if len(below) and below[0] > e and not np.any(az[e:below[0] + 1] > STANCE_AZ):
            j = below[0]
            lat_imu.append(0.5 * (t_imu[j - 1] + t_imu[j]) - t_imu[e])
        pwm_mean = df.loc[after, ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]].to_numpy().mean(axis=1)
        pwm_mid = 0.5 * (cmd + 1000)
        lowp = after[pwm_mean < pwm_mid]
        if len(lowp):
            lat_pwm.append(wall[lowp[0]] - wall[e])
    print(f"  {len(starts)} pushes; length [s]: {stats(lens)}")
    print(f"  specific force during push [m/s^2]: IMU {stats(acc_meas, '{:.2f}')}")
    print(f"      motor model at the commanded PWM: {stats(acc_cmd, '{:.2f}')}")
    print(f"      motor model at the logged PWMs:   {stats(acc_mot, '{:.2f}')}")
    print(f"  cutoff tick -> IMU halfway to idle [s, sim]: {stats(lat_imu)}   "
          f"(10 ms resolution per push)")
    print(f"  cutoff tick -> logged PWM halfway [s, wall]: {stats(lat_pwm)}   "
          f"(includes the telemetry return path)")

    # ── /cf_1/pose vs ground truth ──
    print("\n== /cf_1/pose vs Gazebo ground truth")
    pose_rows = np.flatnonzero(~df.cf_pose_stamp.duplicated().to_numpy() & (df.cf_pose_stamp > 0).to_numpy())
    pose_rows = pose_rows[pose_rows >= 20]
    if len(pose_rows) > 5:
        # The pose is older than the tick that receives it: compare against the ground
        # truth from `lag` ticks earlier, with the lag that fits best
        cf = df[["cf_x", "cf_y", "cf_z"]].to_numpy()
        gt = df[["pos_x", "pos_y", "pos_z"]].to_numpy()
        rms = [np.sqrt(np.mean(np.sum((cf[pose_rows] - gt[pose_rows - lag]) ** 2, axis=1)))
               for lag in range(20)]
        lag = int(np.argmin(rms))
        print(f"  best lag {lag * 10} ms (3D RMS {rms[lag]:.3f} m; {rms[0]:.3f} m at no lag)")
        e_xy = np.hypot(cf[pose_rows, 0] - gt[pose_rows - lag, 0], cf[pose_rows, 1] - gt[pose_rows - lag, 1])
        e_z = cf[pose_rows, 2] - gt[pose_rows - lag, 2]
        e_roll = (df.cf_roll_deg - df.act_roll_deg).to_numpy()[pose_rows]
        e_pitch = (df.cf_pitch_deg - df.act_pitch_deg).to_numpy()[pose_rows]
        print(f"  horizontal error [m]: {stats(e_xy)}")
        print(f"  vertical error [m]:   {stats(e_z)}")
        print(f"  at the end: horizontal {e_xy[-1]:.3f} m, vertical {e_z[-1]:+.3f} m")
        print(f"  roll error [deg]:  {stats(e_roll, '{:.1f}')}")
        print(f"  pitch error [deg]: {stats(e_pitch, '{:.1f}')}")
        print(f"  ({len(pose_rows)} pose messages; the firmware publishes /cf_1/pose at 10 Hz)")

    # ── Apex decisions ──
    dec_path = matching_decisions(path)
    print("\n== Apex decisions" + (f": {dec_path}" if dec_path else ": no matching apex_decisions CSV"))
    if dec_path:
        dec = pd.read_csv(dec_path)
        cols = ["hop_number", "steering", "target_idx", "target_x", "target_y", "target_apex",
                "pred_land_x", "pred_land_y", "takeoff_elev_deg", "unreachable", "cmd_tilt_deg",
                "jhc_nominal_h", "push_time"]
        with pd.option_context("display.width", 200, "display.max_rows", 200):
            print(dec[cols].to_string(index=False))


if __name__ == "__main__":
    main()
