# Yaw fix report (branch `yawFix`)

## Summary

- **The problem:** in simulation, commanding a yaw made Hopcopter flip over. Yaw held at 0 was always fine.
- **The cause:** a bug in the simulated Crazyflie firmware's state estimator (`mm_pose.c`), not in the PID
  gains or the Gazebo model.
  - CrazySim feeds Gazebo's pose, orientation included, into the firmware's Kalman filter at 200 Hz.
  - The filter applied the orientation correction in the wrong frame (world instead of body). The
    roll/pitch corrections were therefore rotated by the heading:
    - correct at yaw 0;
    - absent at ±90°;
    - inverted beyond ±90°.
  - Past ~±90° the filter's roll/pitch estimate diverged, and the attitude controller tipped the real
    robot over while chasing a tilt that did not exist.
  - Bitcraze fixed this upstream in September 2025; this repo's vendored firmware predates the fix.
- **The fix:**
  - The firmware residual (one line), plus two smaller yaw-only firmware defects.
  - The hopcopter changes needed to command yaw without hurting the hop:
    - roll/pitch recomputed for the current heading;
    - yaw commanded only while the push is on;
    - a yaw-rate limit.
- **What it can do now:**
  - Yaw to any heading, including across ±180°, without flipping.
  - Face the waypoint it is hopping toward (`face_waypoint`, on by default).
  - Set `face_waypoint false` to go back to the old fixed heading (`desired_yaw_deg`, default 0).

## 1. What was happening

I ran the full pipeline headless: Gazebo with `-H`, the cflib server, `lidar_mapping.launch.py`
(FAST-LIO and the LiDAR mapper), the planner and hopcopter, on `low_wall`. I then commanded yaw
through a new hopcopter parameter. All of the following are with the **original** firmware:

| Run | Yaw command | Result |
|---|---|---|
| A | in place: 45°, 90°, back to 0° | no flip (max tilt 0.3°) |
| B | goal (4, 0), then 90° while following the path | no flip, goal reached |
| C1, three times | in place: 179° | **flipped** each time, 3.2–3.5 s after the command, as the heading passed ~115–135° |
| D1 | goal (4, 0) and back, facing 180° on the way back | **flipped** as the heading passed −155° |
| C2 | in place, fast steps (gain 150, then 400 °/s per rad) | **flipped** |

So you were right that yaw flips the robot. It only does so once the heading gets past about ±90°.

**Why ±90°.** In the logs before each flip, the firmware's own roll/pitch estimate became the mirror
image of the true attitude. For example, the estimate read +7°, +12°, +17° of roll while Gazebo read
−6°, −11°, −19°. The controller then "corrected" a tilt in the wrong direction until the robot went over.

`kalmanCoreUpdateWithPose` (`crazyflie-firmware/src/modules/src/kalman_core/mm_pose.c`) computed the
orientation residual as `qqmul(qinv(q_ekf), q_measured)`:

- This copy of `math3d.h`'s `qqmul(q, p)` actually returns the Hamilton product p·q, although its
  comment said q·p.
- So the residual was q_meas·q_ekf⁻¹, the **world-frame** error.
- The filter's attitude-error states are **body-frame**.

How much of a roll/pitch correction still points the right way, by heading:

| heading | 0° | 45° | 90° | 120° | 150° | 180° |
|---|---|---|---|---|---|---|
| correction along the true error | +1.00 | +0.71 | 0.00 | −0.50 | −0.87 | −1.00 |

With 200 Hz pose updates, the inverted correction makes the estimate diverge within ~0.1–0.2 s.
Upstream Bitcraze fixed exactly this in commit `c21368d2` ("Fix q_residual calculation", Sep 2025),
and later corrected `qqmul` itself (`a7becf0`, Jul 2026).

**Why hardware is fine.** The bug needs an external *orientation* fused into the filter and a heading
beyond ±90°. The hardware most likely runs firmware with the fix, or does not stream orientation into
the filter. I did not check the hardware.

**Why yaw 0 always worked.** At heading 0 the wrong frame and the right frame coincide.

Two smaller firmware defects, both zero at heading 0:

1. **Reset kick.** hopcopter sends zero thrust for one tick at every apex, which resets the firmware's
   attitude PIDs. `pidReset` set the stored previous measurement to 0. The yaw-angle PID's D term
   (on the measurement, kd 0.35, 500 Hz) then read the whole heading as a one-tick jump:
   - a kick of −175 × heading °/s (−15,400 °/s at 88°);
   - a saturated yaw output for one step;
   - −515 PWM left in the yaw-rate integrator for the whole hop, which switched one diagonal motor
     pair off during the falls.
2. **±180° wrap.** The same D term saw a 360° step whenever the heading crossed ±180° (−63,000 °/s).

Too-fast yaw commands are a separate failure (C2 at gain 400, up to ~630 °/s commanded). The yaw
output saturates, one diagonal motor pair stops, and roll/pitch lose authority. That is a
command-limit problem, handled in hopcopter by `yaw_rate_max`.

## 2. The firmware fix

The SITL firmware must be rebuilt for any of these to take effect:
`make -C crazyflie-firmware/sitl_make/build`.

| File | Change |
|---|---|
| `src/modules/src/kalman_core/mm_pose.c` | Residual is now `qqmul(q_measured, qinv(q_ekf))`, the body frame (upstream c21368d2). **This is the root-cause fix.** |
| `src/utils/src/pid.c`, `src/utils/interface/pid.h` | `pidReset(pid, actual)` stores the current measurement (upstream backport): no D kick after a reset. |
| `src/modules/src/controller/attitude_pid_controller.c`, `.../interface/controller/attitude_controller.h` | Attitude PIDs reset at the current roll/pitch/yaw; rate PIDs at 0 (as upstream). The yaw D term is kept continuous across ±180°. |
| `src/modules/src/controller/controller_pid.c` | Passes the current attitude to those resets. |
| `controller_indi.c`, `position_controller_pid.c` | New signatures only; behavior unchanged (they pass 0 as before). |
| `src/modules/interface/math3d.h` | Comment only: `qqmul` returns p·q (the misleading comment is how the bug got in). |

At heading 0 these are corrections, not behavior changes:

- The roll/pitch attitude PIDs have kd 0, so resetting them at the current angle changes nothing.
- The two residual orders are identical when the body is level. When tilted, the old order also leaked
  part of a pitch error into the yaw correction (about 34% at 20°).
- The removed yaw kick was a few °/s at yaw 0.

**Verification (same runs, fixed firmware):**

- **C1** (179°, then back and forth across ±180°): no flip, max tilt 0.2°. The firmware's roll/pitch
  estimate matched Gazebo within 0.01° at heading ±180°.
- **D1:** no flip (max tilt 5.3°).
- **Final defaults, in place** (90°, −90°, 180°, −179°, 0°):
  - every step converged, with no overshoot and a final error ≤ 0.1°;
  - max tilt 0.3°;
  - 90° took 2.7 s and 180° about 5 s.

## 3. Facing the waypoint (hopcopter)

### How it works

- **Heading choice:** at each steering apex, the waypoint decision sets the heading toward the hop's
  target. It uses `atan2` from the predicted foot touchdown to the aim point.
  - Hops in place (`goal`, `hold`, `idle`) keep the current heading.
  - Hops shorter than 0.5 m keep the current heading. Without this, short repositioning hops caused
    120° swings.
  - The robot starts facing +x, as before.
- **Landing attitude at any heading:**
  - The takeoff solve, the touchdown march and the planner were already world-frame. The heading does
    not enter the hop: the leg is along body z.
  - The only place heading matters is turning the solved world-frame landing axis into the firmware's
    roll/pitch, which are Euler angles under the heading.
  - Before, that conversion happened once at the apex, so any yaw during the fall swung the landing
    tilt around.
  - Now the landing axis is kept and converted every tick at the current heading (`tilt_rp`).
  - The heading is the intrinsic ZYX yaw, the firmware's convention. scipy's lowercase
    `as_euler('zyx')`, used before, is extrinsic and is off by 3.5° at a 14°/14° tilt.
- **When yaw is commanded (`yaw_gate`, default `push`):** yaw is commanded only while the push is on,
  and in falls that land level (hops in place). Otherwise the yaw-rate command is 0. The reason:
  - In CrazySim a motor commanded below PWM 1000 stops (`PWM2OMEGA`), and hopcopter falls at exactly
    PWM 1000. Any yaw torque in a fall therefore switches a diagonal motor pair off, which takes the
    roll/pitch authority a tilted landing needs.
  - The push (PWM 30000) has thrust to spare, and the level coast after it settles the turn before
    the apex.
- **Yaw-rate command:** `yaw_kp` (90 °/s per rad) × heading error, capped at `yaw_rate_max` (60 °/s).
  - The cap keeps the spin small enough to stop in the coast.
  - The firmware integrates the command (legacy commander, yaw is a rate).

### What the gate choice did (flat world, LiDAR map, goal (4, 0) then (1, 1.5): a 156° turn)

These runs used gain 90, a 90 °/s cap and a 0.3 m minimum hop:

| `yaw_gate` | Landing error, steered hops | 156°-turn hop | Max tilt | Turn time (150°) |
|---|---|---|---|---|
| yaw off (baseline) | 0.03–0.22 m, median 0.04 | 0.03 m | 20.4° | — |
| `always` | up to 0.81 m | **0.81 m** (0.73 m in a second run) | 27.2° | ~2.5 s |
| `climb` (push + coast) | 0.02–0.09 m | 0.04 m | 20.1° | ~4.5 s |
| `push` | 0.03–0.06 m | 0.03 m | 19.9° | ~6 s |

On `low_wall` out and back, both goals were reached with heading on (early version, gain 30). The
return leg, turning 180°, landed within 0.04–0.15 m.

`climb` turns faster but reached apexes still spinning at ~45 °/s. On the stairs, stopping that spin
in a tilted fall tipped the robot over; the yaw-off stairs run did not flip. That is why the defaults
are `push`, 60 °/s and 0.5 m.

### Limits

- **Turning while travelling is slow:** about 10–15° per hop with `push`, because pushes last only
  0.04–0.3 s. Hops in place turn faster. A 90° change finishes over a few hops, so on a zig-zag the
  heading lags behind the path.
- **Faster turning trades landing accuracy:** `ros2 param set /hopcopter yaw_gate climb` (or `always`).
- **Making the falls turn safely would need a falling idle above the motor cut-off.** That would mean
  recalibrating the stage 0–4 fall constants, so it was out of scope.
- **The final defaults were tested only in place.** `push` with a 60 °/s cap and a 0.5 m minimum was
  verified by the manual yaw steps above. The flat comparison used a 90 °/s cap and 0.3 m. As agreed,
  I did not sweep all maps.
- **Not yaw issues:**
  - `slanted_pads` also fails with yaw held at 0. On the known map it fell after a 1.17 m miss on hop 8;
    on the LiDAR map the planner finds no path from the start platform.
  - The LiDAR stairs goal is still unreachable, as before.

### Parameters (all settable at runtime with `ros2 param set /hopcopter ...`)

| Parameter | Default | Meaning |
|---|---|---|
| `face_waypoint` | `true` | Face each hop's target. `false` returns to `desired_yaw_deg`. |
| `desired_yaw_deg` | `0.0` | Fixed heading [deg, world] when `face_waypoint` is false. |
| `yaw_kp` | `90.0` | Yaw rate [°/s] per rad of heading error. Must be ≥ 0. |
| `yaw_rate_max` | `60.0` | Yaw-rate cap [°/s]. Must be > 0. |
| `yaw_gate` | `push` | `push`, `climb` or `always` (see above). |

## 4. Other changes

- **`ros2_ws/src/hopping_robot/hopping_robot/hopcopter.py`:**
  - the parameters above, validated at startup and at runtime;
  - the heading choice, the per-tick landing attitude, the intrinsic heading and the phase-gated yaw
    rate;
  - **log changes:**
    - per-tick CSV: new last column `yaw_rate_cmd`;
    - apex-decision CSV: new last columns `heading_deg` and `heading_cmd_deg`;
    - logged Euler angles (`act_*`, `cf_*`, landing and hop CSVs) are now intrinsic ZYX. At heading 0
      they match the old ones to second order. With a heading, the old ones would have misled
      `calibrate_hops.py` and `visualize_rpy.py`.
- **`ros2_ws/src/hopping_robot/hopping_robot/JumpLib/heading.py` (new):** `tilt_rp`, `face_heading`,
  `heading_of`, `wrap_pi`, `yaw_rate_cmd`.
- **`ros2_ws/src/hopping_robot/test/test_heading.py` (new):** 91 tests. The hopping_robot suite is
  179 tests, all passing:
  `cd ros2_ws/src/hopping_robot && /usr/bin/python3 -B -m pytest -p no:cacheprovider test/test_waypoint_tracker.py test/test_push_controller.py test/test_takeoff_solver.py test/test_touchdown.py test/test_heading.py`
- **`calibrate_hops.py`:** docstring only. `rot_zyx` is exact for the new intrinsic angles.
- **`CLAUDE.md`:** test command and count, and a short yaw note.

To use it:

1. Rebuild the SITL firmware: `make -C crazyflie-firmware/sitl_make/build`.
2. Rebuild the workspace: `colcon build`.
3. Run as usual. The robot faces where it hops.
