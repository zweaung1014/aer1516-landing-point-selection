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
  - stage 2 push cutoff: the lead each push implies, apex error per target, 3->2 transitions
    during a push, injection available within the 0.4 s cap against liftoff speed, and the
    horizontal speed a push adds after a tilted liftoff
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
# hopcopter's PushController settings, to read its decisions back
PUSH_G_COAST = 9.55  # g' its cutoff assumes for the coast
Z_LO = 0.366         # liftoff CoM height; apex heights are above it
TICK = 0.01          # control period
PUSH_CAP = 0.4       # t_max


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
    flight_g = []    # g' per flight, aligned with flights (None when not fitted)
    flight_td = []   # CoM position at touchdown, from the same fit
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
            flight_g.append(None)
            flight_td.append(None)
            continue
        tt = od_t[sel] - t_td(stances[k])
        cz = np.polyfit(tt, od_p[sel, 2], 2)
        cx = np.polyfit(tt, od_p[sel, 0], 2)
        cy = np.polyfit(tt, od_p[sel, 1], 2)
        # Not ballistic (e.g. lying on the ground after a crash, which reads ~1 g): skip
        resid = od_p[sel, 2] - np.polyval(cz, tt)
        if not 7.0 < -2.0 * cz[0] < 12.0 or np.sqrt(np.mean(resid ** 2)) > 0.005:
            flights.append(None)
            flight_g.append(None)
            flight_td.append(None)
            rejected += 1
            continue
        g_fit.append(-2.0 * cz[0])
        rows = od_tick[sel]
        f_up = motor_force(df.loc[rows, ["motor_m1", "motor_m2", "motor_m3", "motor_m4"]].to_numpy()).sum(axis=1)
        g_model.append(G - np.mean(f_up * cos_tilt[rows]) / MASS)
        g_tilt.append(np.degrees(np.arccos(np.clip(cos_tilt[rows].min(), -1, 1))))
        # velocity at touchdown, from the fit (unpowered all the way down)
        flights.append(np.array([cx[1], cy[1], cz[1]]))
        flight_g.append(-2.0 * cz[0])
        flight_td.append(np.array([cx[2], cy[2], cz[2]]))

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
    lo_vel = {}      # stance index -> (liftoff time, liftoff velocity)
    lo_pos = {}      # stance index -> CoM position at liftoff
    for k, s in enumerate(stances):
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
        lo_vel[k] = (tl, v_lo)
        lo_pos[k] = od_p[j] - v_lo * (od_t[j] - tl)
        v_td = flights[k]
        if v_td is None:
            continue
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
        steady = np.arange(s0 + 4, e)                 # skip the spin-up (94% thrust at 40 ms)
        if len(steady) < 2:
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

    # ── Stage 2: push cutoff ──
    dec_path = matching_decisions(path)
    dec = pd.read_csv(dec_path) if dec_path else None
    print("\n== Push cutoff (stage 2)")
    n_32_push = int(np.sum((js[:-1] == 3) & (js[1:] == 2) & push[:-1]))
    print(f"  3->2 transitions during a push: {n_32_push}")
    a_cmd = 4 * motor_force(np.median(df.thrust_cmd.to_numpy()[push])) / MASS if push.any() else np.nan
    if dec is None or "push_end" not in dec.columns:
        print("  no stage 2 columns in the apex decisions")
    else:
        c = dec[dec.push_end.notna()]
        print("  how pushes ended: " + ", ".join(f"{k} {v}" for k, v in c.push_end.value_counts().items()))
        tgt = c[c.push_end == "target"]
        if len(tgt):
            # the thrust that kept coming after each cut, in seconds of full push, under the
            # controller's own model (motor-model thrust, g' PUSH_G_COAST)
            pred = tgt.cut_h + tgt.cut_vz ** 2 / (2 * PUSH_G_COAST)
            lead = (tgt.climb_apex - pred) * PUSH_G_COAST / (tgt.cut_vz * a_cmd * tgt.cut_cos_tilt)
            print(f"  lead each cut implies [s]: {stats(lead)}")
            print(f"    -> t_lead = median + half a tick = {np.median(lead) + 0.5 * TICK:.3f} s")
            err = tgt.climb_apex - tgt.climb_target
            print(f"  apex - target, pushes cut at the target [m]: {stats(err)}")
            for h, e in err.groupby(tgt.climb_target.round(2)):
                print(f"    target {h:.2f}: {stats(e)}")
        leg = c[c.push_end == "leg"]
        if len(leg):
            print(f"  apex - target, leg alone over the target (no push) [m]: "
                  f"{stats(leg.climb_apex - leg.climb_target)}")

    # ── Push capacity ──
    print("\n== Push capacity: injection above idle, g'_flight * h_apex - v_z0^2 / 2 [J/kg]")
    caps = []
    for s0 in starts:
        e = ends[ends > s0]
        ks = [k for k, st in enumerate(stances) if t_imu[st[1]] <= t_imu[s0] + 1e-9]
        if not len(e) or not ks or ks[-1] not in lo_vel or ks[-1] + 1 >= len(stances):
            continue
        k, e = ks[-1], e[0]
        g_f = flight_g[k + 1]
        if g_f is None:
            continue
        tl, v_lo = lo_vel[k]
        sel = (od_t > tl) & (od_t < t_td(stances[k + 1]))
        h = od_p[sel, 2].max() - Z_LO
        caps.append((v_lo[2], t_imu[e] - t_imu[s0], h, g_f, g_f * h - 0.5 * v_lo[2] ** 2))
    if len(caps) >= 3:
        cp = np.array(caps)

        def injection(v0, t_push, g_f, a_t, delta):
            # level push at a_t from v0 for t_push + delta (spin-up, spin-down), above idle
            t_e = np.minimum(t_push + delta, v0 / (G - a_t))   # it cannot push past the apex
            return (a_t - (G - g_f)) * (v0 * t_e - 0.5 * (G - a_t) * t_e ** 2)

        # Effective thrust and timing, in terms of the same liftoff speed as eta above (which is
        # how the planner will get v_z0); the steady thrust itself is the motor model's
        best = (np.inf, None, None)
        for a_t in np.arange(6.5, 8.51, 0.01):
            for delta in np.arange(-0.03, 0.0301, 0.001):
                rms = np.sqrt(np.mean((injection(cp[:, 0], cp[:, 1], cp[:, 3], a_t, delta) - cp[:, 4]) ** 2))
                if rms < best[0]:
                    best = (rms, a_t, delta)
        rms, a_t, delta = best
        print(f"  {len(cp)} pushes, {cp[:, 1].min():.3f}-{cp[:, 1].max():.3f} s, v_z0 {cp[:, 0].min():.2f}-"
              f"{cp[:, 0].max():.2f} m/s")
        print(f"  fit: level push at {a_t:.2f} m/s^2 (motor model {a_cmd:.2f}) for the push time "
              f"{delta * 1000:+.0f} ms; RMS {rms:.2f} J/kg")
        g_lvl = np.median(g_fit[g_tilt < 5]) if np.any(g_tilt < 5) else PUSH_G_COAST
        print(f"  available within the {PUSH_CAP} s cap (g' {g_lvl:.2f}):")
        print(f"    {'v_z0':>5} {'J/kg':>6} {'apex_m':>6}")
        for v0 in np.arange(1.0, 5.01, 0.5):
            inj = injection(v0, PUSH_CAP, g_lvl, a_t, delta)
            print(f"    {v0:5.1f} {inj:6.2f} {(0.5 * v0 * v0 + inj) / g_lvl:6.2f}")
        hit = cp[cp[:, 1] >= PUSH_CAP - 0.5 * TICK]
        for v0, t_p, h, g_f, inj in hit:
            print(f"  measured at the cap: v_z0 {v0:.2f}, {t_p:.3f} s -> {inj:.2f} J/kg, apex {h:.2f} m "
                  f"(fit {injection(v0, t_p, g_f, a_t, delta):.2f})")

    # ── Push after a tilted liftoff (D2) ──
    print("\n== Push after a tilted liftoff: horizontal speed it adds (liftoff -> 60 ms after the cut)")
    rows_dv = []
    for s0 in starts:
        e = ends[ends > s0]
        ks = [k for k, st in enumerate(stances) if t_imu[st[1]] <= t_imu[s0] + 1e-9]
        if not len(e) or not ks or ks[-1] not in lo_vel:
            continue
        k, e = ks[-1], e[0]
        v_h = lo_vel[k][1][:2]
        tilt_lo = np.degrees(np.arccos(np.clip(cos_tilt[s0], -1, 1)))
        j = np.flatnonzero(od_t >= t_imu[e] + 0.06)
        if np.hypot(*v_h) < 0.3 or not len(j) or (k + 1 < len(stances) and od_t[j[0]] >= t_td(stances[k + 1])):
            continue
        u = v_h / np.hypot(*v_h)
        dv = od_dv[j[0], :2] - v_h
        # model: the push's horizontal thrust along the logged attitude
        model = sum(a_cmd * rot_zyx(yaw[i], pitch[i], roll[i])[:2, 2] * TICK for i in range(s0, e))
        rows_dv.append((t_imu[s0], tilt_lo, t_imu[e] - t_imu[s0], np.hypot(*v_h),
                        dv @ u, u[0] * dv[1] - u[1] * dv[0], model @ u))
    if rows_dv:
        print(f"  {'t_sim':>8} {'tilt_lo':>7} {'push_s':>6} {'v_h_lo':>6} {'dv_along':>8} {'dv_cross':>8} "
              f"{'model':>6}")
        for r in rows_dv:
            print(f"  {r[0]:8.2f} {r[1]:7.1f} {r[2]:6.3f} {r[3]:6.2f} {r[4]:8.3f} {r[5]:8.3f} {r[6]:6.3f}")
        r = np.array(rows_dv)
        print(f"  dv_along [m/s]: {stats(r[:, 4])};  model: {stats(r[:, 6])}")
    else:
        print("  no pushes after a steered landing")

    # ── Stage 4: each steered hop against the takeoff solve and the plan ──
    if dec is not None and "solve" in dec.columns:
        print("\n== Takeoff solve (stage 4): each steered hop against the model and the plan")
        td_t = np.array([t_td(s) for s in stances])
        body_z = lambda i: rot_zyx(yaw[i], pitch[i], roll[i])[:, 2]  # noqa: E731

        def angle(a, b):
            return np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1, 1)))

        hops = []
        dec_s = dec[(dec.steering == 1) & dec.solve.notna() & (dec.solve != "warmup")]
        for r, d in dec_s.iterrows():
            k = int(np.searchsorted(td_t, d.odom_stamp))
            if k + 1 >= len(stances) or k not in lo_vel or flight_td[k] is None or flight_td[k + 1] is None:
                continue
            a, b = stances[k]
            v_in, (tl, v_lo) = flights[k], lo_vel[k]
            z_td, z_lo = body_z(a - 1), body_z(b)
            foot = flight_td[k][:2] - Z_LO * z_td[:2]
            foot2 = flight_td[k + 1][:2] - Z_LO * body_z(stances[k + 1][0] - 1)[:2]
            target = np.array([d.target_x, d.target_y])
            u = target - np.array([d.foot_land_x, d.foot_land_y])
            X = np.linalg.norm(u)
            u = u / X if X > 0.05 else None
            th_l, th_v, th_t = angle(-v_in, z_td), angle(-v_in, v_lo), angle(-v_in, z_lo)
            nxt = dec.loc[r + 1] if r + 1 in dec.index else None
            e = foot2 - target
            hops.append(dict(
                t=d.odom_stamp, mode=d["mode"], solve=d.solve, X=X,
                pred=np.linalg.norm(foot - [d.foot_land_x, d.foot_land_y]),
                tilt_cmd=d.cmd_tilt_deg, tilt=np.degrees(np.arccos(np.clip(z_td[2], -1, 1))),
                att_err=angle(z_td, np.array([d.cmd_axis_x, d.cmd_axis_y, d.cmd_axis_z])),
                thv_cmd=d.theta_v_deg, thv=th_v,
                k1=th_t / th_l if th_l > 6 else np.nan, k2=th_v / th_l if th_l > 6 else np.nan,
                beta=np.degrees(np.arctan2(-v_in[2], np.hypot(v_in[0], v_in[1]))),
                eta=(np.linalg.norm(v_lo) / np.linalg.norm(v_in)) ** 2,
                kick=(flights[k + 1][:2] - v_lo[:2]) @ u if u is not None else np.nan,
                kick_pred=d.get("pred_kick_v", np.nan),
                lift=(lo_pos[k][:2] - flight_td[k][:2]) @ u if u is not None and k in lo_pos else np.nan,
                lift_pred=d.pred_lift_dx,
                T=t_td(stances[k + 1]) - tl, T_pred=d.pred_t_up + d.pred_t_down,
                apex_err=(nxt.climb_apex - nxt.climb_target) if nxt is not None and pd.notna(nxt.climb_target) else np.nan,
                along=e @ u if u is not None else np.nan, across=(u[0] * e[1] - u[1] * e[0]) if u is not None else np.nan,
                miss=np.linalg.norm(e), model=(foot2 - [d.pred_next_x, d.pred_next_y]) @ u if u is not None else np.nan,
                tilt_next=np.degrees(np.arccos(np.clip(body_z(stances[k + 1][0] - 1)[2], -1, 1)))))
        H = pd.DataFrame(hops)
        if len(H):
            moving = H[H.X > 0.3]
            with pd.option_context("display.width", 240, "display.max_rows", 300):
                print(moving[["t", "mode", "solve", "X", "pred", "tilt_cmd", "tilt", "att_err", "thv_cmd", "thv",
                              "eta", "kick", "kick_pred", "lift", "lift_pred", "T", "T_pred", "apex_err",
                              "along", "across", "model"]].round(3).to_string(index=False))
            print("  (X: hop length; pred: this landing against its apex prediction; att_err: touchdown attitude "
                  "against the command [deg]; thv: takeoff angle from the arrival [deg]; kick, lift: the push's "
                  "horizontal kick [m/s] and the stance's CoM shift [m] along the hop; along, across: the next "
                  "landing against its target; model: against the solve's own prediction [m])")
            print(f"  landing prediction at the apex [m]: {stats(H.pred)}")
            print(f"  touchdown attitude against the command [deg]: {stats(H.att_err)}")
            print(f"  landing tilt as flown [deg]: max {H.tilt_next.max():.1f}")
            print("  landing against the target, by hop length (D6):")
            for lo_x, hi_x in ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 3.0)):
                b = moving[(moving.X >= lo_x) & (moving.X < hi_x)]
                if len(b):
                    print(f"    {lo_x:.1f}-{hi_x:.1f} m: n={len(b):2d}, along {b.along.mean():+.3f} "
                          f"(sd {b.along.std():.3f}), across sd {b.across.std():.3f}, "
                          f"miss median {b.miss.median():.3f} max {b.miss.max():.3f}")
            st = H[H.k2.notna()]
            vert = st.beta > 85
            print(f"  stance model, measured: k1 {stats(st.k1)}")
            print(f"    k2 after vertical arrivals: {stats(st.k2[vert])}")
            print(f"    k2 otherwise:               {stats(st.k2[~vert])}")
            for lo_v, hi_v in ((0, 30), (30, 45), (45, 60), (60, 180)):
                b = H[(H.thv >= lo_v) & (H.thv < hi_v)]
                if len(b):
                    print(f"    eta at theta_v {lo_v}-{hi_v} deg: {stats(b.eta)}")
            kk = H[(np.abs(H.kick_pred) > 0.02) & H.kick.notna()]
            if len(kk):
                print(f"  push kick, measured / predicted: {stats(kk.kick / kk.kick_pred)}")

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
    print("\n== Apex decisions" + (f": {dec_path}" if dec_path else ": no matching apex_decisions CSV"))
    if dec is not None:
        cols = ["hop_number", "decided", "mode", "land_idx", "target_idx", "retries",
                "foot_land_x", "foot_land_y", "target_x", "target_y", "target_apex",
                "solve", "alpha_leg_deg", "takeoff_elev_deg", "unreachable",
                "planned_tilt_deg", "tilt_needed_deg", "cmd_tilt_deg", "plan_tilt_deg",
                "climb_target", "climb_apex", "push_end", "push_s",
                "jhc_nominal_h", "push_time"]
        cols = [c for c in cols if c in dec.columns]   # older logs lack the stage 1 columns
        with pd.option_context("display.width", 220, "display.max_rows", 200):
            print(dec[cols].to_string(index=False))


if __name__ == "__main__":
    main()
