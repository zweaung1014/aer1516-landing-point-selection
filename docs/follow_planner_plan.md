# Plan: make the controller fly the planner's plan

Status: stages 0–4 implemented and tested 2026-10-29 (results in each stage), and problem 2
fixed along the way (stage 2 results).
Written 2026-10-29; decisions D1–D6 made the same day. Follows [diagnosis.md](diagnosis.md).

All four problems in the diagnosis come down to one thing: the controller does not fly what the
planner plans. This plan fixes problems 1, 3 and 4 in five stages. Problem 2 (the WP1 crash and
motor cut-out) was deferred, then found and fixed in stage 2: it was a SITL accelerometer
artifact that tripped the firmware's tumble check (see Deferred).

| Stage | Problem | Code | Change | Verified by |
|---|---|---|---|---|
| 0 | all | hopcopter, planner build | Instrument; calibrate g', latency, η; optimized planner build | logs |
| 1 | 1 | hopcopter | One waypoint per hop, decided at the apex, with guards; braking tilt guard | sim, flat world |
| 2 | 3 | hopcopter, JumpLib | Lead-compensated push cutoff; thrust 30000 | sim, in place |
| 3 | 4 | planner | Model the real push; momentum-aware search; publish each hop's takeoff | offline harness |
| 4 | 4 | hopcopter | Fly the published takeoff as feed-forward, corrected from the measured arrival | sim, flat then low_wall |

Order: 0 → 1 → 2 → 4. Stage 3 is offline work and can run alongside stages 1–2. Stage 4 needs
both 2 and 3.

## Decisions (2026-10-29)

| | Decision | What it means for the plan |
|---|---|---|
| D1 | Thrust 30000 (0.81 g, below hover) | The push still ends by the apex on its own, so it cannot run away. Planned pushes are ≤ 0.2 s; in-place pushes are 42–74 ms, longer than the motor time constants. No false-touchdown risk (acc_z 0.83 g). Budget about 4 J/kg per hop. |
| D2 | Push starts at liftoff | The still-tilted body adds horizontal speed. Stage 2 measures it, and stage 4's solve must correct for it. |
| D3 | min_clearance 0.3 m | low_wall plans become three straight hops within the 30000 budget. |
| D4 | τ_max 20° | Revisit after stage 4's redirection tests and after problem 2. |
| D5 | Planner publishes each hop's takeoff (`HopPlan`); the controller flies it as feed-forward | See item 5 below. |
| D6 | No hop-length cap yet | Stage 4 reports landing error against hop length; decide then. |

## What the investigation changed

Checked against the 2026-10-27 logs, the console logs, and offline runs of the C++ planner core.
The offline harness reproduces the 10:53 plan bit for bit. Two independent reviewers re-derived
the numbers below.

1. **Problem 1: a bigger tolerance cannot work.** To switch between the right two apexes, the
   radius must exceed ~0.46·X of the hop landing on that waypoint and stay below the waypoint's
   distance from the previous apex. With a 0.2 m margin the per-plan windows are 1.35–2.30 m,
   1.29–1.90 m, 1.21–2.00 m and 1.02–1.32 m, and 10:32:19 has none. No single value fits.
   Switching at the apex is right, but it needs the guards in stage 1.
2. **Problem 3: t_p_high is not a lever, thrust is.** t_p_high is already 1.5 s. JHC never asked for
   more than ~0.43 s, and all 26 pushed climbs ended before the apex. JHC also asks for too little:
   - It compares a vertical target_apex with a total-energy estimate.
   - It assumes 0.7 g of thrust, where the real value is 0.48 g.
   - Its energy estimate is one hop old.

   But even a perfect JHC cannot reach the targets at PWM 21000. A push held to the apex adds at
   most 0.94·½v_z0² J/kg: 2.0–3.2 J/kg for the first steered climbs, which need 3.4–4.7, and
   1.4 J/kg for the 10:53 wall hop, which needs 9.8.
3. **Problem 4: taking off at the planner's α lands short.** The planner assumes the push adds
   speed along α. The robot pushes level (roll = pitch = 0 in state 3), after liftoff, so the leg
   must supply all the horizontal speed. Launching the leg at α lands the two 10:53 hops 8% and
   23% short (2.66 m against 3.47 m). The angle that matters is the leg angle
   α_leg = acos(v_h / v_leg): 43.0° and 25.8° for those hops. The 10:53 wall hop is infeasible with
   a level push. It is outside the friction cone and over the energy budget.
4. **Energy survives a change of direction, as far as the logs show.** The only direction changes
   on record are the nine WP1 brakes. In each, about 3 m/s of horizontal arrival was turned into a
   near-vertical takeoff.
   - The robot landed at 3.7–4.2 m/s and left at 3.1–3.7 m/s. That keeps 71–79% of the energy,
     the same as hopping in place.
   - If the horizontal part had been cancelled, the robot would have left at 2.1–2.8 m/s.
   - The three runs that kept hopping peaked at 0.66–1.10 m, against 0.24–0.39 m (plus the push)
     for a vertical-only carry-over.

   The leg redirects horizontal energy rather than cancelling it; it even over-redirected, into
   the bounce-backs. So the planner keeps carrying the full landing energy. Turns at speed and
   straight running have not been measured yet. Stage 3 makes the carry-over a function of the
   redirection, and stage 4 measures it.

   Where the intuition is right is the push. A level push adds only vertical energy, so the next
   hop's horizontal speed must all come from the leg, out of the redirected landing energy.
   Stage 3's leg-angle check enforces that.
5. **Who cancels the incoming momentum: both the planner and the controller.**
   - **The planner** does it for the plan. From the planned arrival it computes the takeoff
     (heading, α_leg, v_h) and the landing attitude that redirects the robot into it. It rejects
     any redirection that needs more than 20° of landing tilt, and publishes the result.
   - **The controller** redoes the same arithmetic (`JumpingModel3D.fcn1`) at every apex, from the
     arrival it actually measures.

   Leaving it to the planner alone would be open loop. Each hop's error (apex, landing point,
   arrival speed) would turn into a wrong landing attitude, then a wrong takeoff, then a bigger
   error on the next hop. With the controller recomputing, errors get corrected one hop later
   instead of compounding. The planner's numbers are the feed-forward and the yardstick: logging
   planned against commanded landing attitude shows how far off-plan the robot is.

   The planner has to do it too. Even flown perfectly, the 10:53 plan lands on WP1 at 29.7° of
   tilt, against 7.7° if it went straight. Landings at 30–35° ended 11 of 14 runs. The detour
   itself comes from the A* flying only the least-injection angle per hop: it burns spare energy
   with a sideways hop. Steeper "banking" candidates fix that.

   Caveat: every tilt number comes from the controller's stance model (k1 = 1.688, k2 = 2.469). It
   is validated only for vertical arrivals and for brakes, which over-cancelled: the achieved tilt
   was 111% of command.
6. **Attitude tracking is not the bottleneck.** The September plan measured 47% of the commanded
   tilt on one hop. Over 45 steered falls the robot reaches 95–117% by touchdown (t90
   0.20–0.35 s). No gain change is needed first.
7. **The planner node cannot start with a realistic e_inject_max.** `validate_params` throws in the
   node constructor for any value below 5.89 J/kg (at min_apex 0.7, h_initial 0.8).

## Stage 0: instrument and calibrate

hopcopter.py:
- **Per-tick CSV.** Add:
  - pos_x/y/z;
  - world-frame velocity from differentiated pose, with the odom twist and odom header stamp next
    to it;
  - thrust command, motor_m1..m4 and the push-active flag;
  - `/cf_1/pose` position and attitude. This is diagnosis item 4. It tests the int16-accelerometer
    suspicion and shows whether `/cf_1/pose` tracks position, which matters because it is the
    planner's start.
- **Per apex decision, one row.** Time, path id, target index, predicted landing point, chosen
  heading and leg angle, planned and commanded landing attitude, push target, and flags
  (unreachable, tilt-capped, retry, hold).
- **Make `desired_rel_z` a ROS parameter**, for stage 2's sweep.

Calibrate from one in-place run plus one steered hop:
- **Odom twist frame.** It needs the steered hop; vertical hops cannot show it. LSE and LJC treat
  it as world-frame. (Measured: body frame, ~10 ms window; see the results below.)
- **g'**, the effective gravity in unpowered flight. Idle PWM 1000 should make 0.2 N, about 5.6% of
  the weight, which would give about 9.3 m/s². (Measured: 9.55; see the results below.)
- **Command-to-motor latency**, from the thrust command vs the PWM timestamps.
- **η across landing speeds and tilts.** Today's 0.769 comes from the same vertical bounce
  repeated in every run. Speed ratios give 0.71–0.85.

Planner build: `cd ~/CrazySim/ros2_ws && colcon build --packages-select ballistic_motion_planner
--cmake-args -DCMAKE_BUILD_TYPE=Release`. Also set a default build type in its CMakeLists.txt so a
clean build keeps it. It is built unoptimized today: the 10:53 goal takes 10.6 s that way and 3.1 s
at -O2. Re-run `ballistic_parity_test` afterwards.

### Stage 0 results (2026-10-29)

Done: the per-tick columns, `hop_tracking/apex_decisions_*.csv`, the `desired_rel_z` parameter,
`calibrate_hops.py`, and Release as the planner's default build. `ballistic_parity_test` output is
byte-identical; it runs in 0.17 s instead of 0.60 s.

Calibration run (headless, flat world, 15:34): in place at 0.8, then 1.0, 0.6 and 0.8 via
`ros2 param set`, then goal (4,0). Release planned (4,0) in 1.8 s. 63 stances, 64 pushes.
Re-run with `python calibrate_hops.py`.

| Quantity | Measured | Was assumed | Consequence |
|---|---|---|---|
| Real-time factor | 0.98 | 1 | Wall-clock timers run 2% long in sim time. Fit kinematics on the odom stamps. |
| g′, level falls | 9.55 (9.551–9.553, n = 61) | ~9.3 | Idle PWM 1000 sits on the plugin's cut-off (`PWM2OMEGA` is 0 below 1000), so the motors are only partly on: 0.25 m/s² instead of 0.55. |
| g′, steered falls | 9.33 at 19°, 8.99 at 30° | — | Attitude control raises the mean PWM to 1760–4120. Stage 2's cutoff and stage 4's flight time must use the g′ of the fall being flown, not one constant. |
| η, in place | 0.767 median (0.71–0.81, n = 61) | 0.769 | No trend with landing speed (3.5–4.7 m/s). Stage 3 uses 0.77. |
| η, steered | 0.71 (19.7° landing, 53° redirection), 0.78 (30° brake) | — | Only two points. Stage 4's turn tests still fit η(θ). |
| Odom twist | body frame, ~10 ms window: RMS 0.003 m/s in x, 0.005 in z | world frame, 50 ms mean | Rotated to world, it is a usable v̂ from the second tick after liftoff. The first tick is off by up to 0.47 m/s. LSE and LJC read it as world-frame: right at a level apex, wrong when tilted. |
| Push thrust at 21000 | 4.73 m/s² (IMU) | 4.75 (motor model) | The motor model holds, so 30000 should give 7.98 m/s². |
| Cutoff latency | 15 ms from the cutoff tick to half thrust (10 ms resolution) | 25 ms τ_down plus latency | Transport latency is near zero; spin-down dominates. Stage 2 calibrates t_lead end to end. |
| Stance | 30–48 ms; every stance peaks at 23–30 g | ~0.1 s, ~11 g | All 63 stances exceed the plugin's int16 ±16 g range. |
| `/cf_1/pose` | ~90 ms behind truth. x/y within 1 mm at rest, up to 0.26 m in fast motion. z −0.18 m median, down to −1.2 m | — | The planner uses only x/y. A mid-run replan has to allow for the lag (deferred). |

Problem 2, diagnosis item 4, is confirmed. In the push after the 29° WP1 brake landing, all four
PWMs read 0 while 21000 is commanded. Two motors were already at 0 during the stance. The IMU
reads zero thrust, and the robot was on its side 0.9 s later. The int16 overflow is not enough to
explain it on its own: the other 61 stances also exceed 16 g and kept their motors.

The first climb is unpowered for a different reason. The motors first turn on 1.15 s of sim time
after Play, mid-fall, whatever is commanded.

The decision log reproduces problem 1. Apex 62 still aims at WP1, from a predicted landing 0.43 m
short. That gives an 82.6° takeoff and a 29.2° braking landing.

## Stage 1: one waypoint per hop, decided at the apex (problem 1)

The planner plans exactly one hop per waypoint. LJC and JHC commit to the next landing and push at
the apex, so the target must change there.

Replace the distance pop with an index `land`: the waypoint the current flight is meant to land
on. At each steering apex, with at most one decision per real touchdown:

1. **Predict the foot landing point P.** Take LSE's CoM prediction and subtract the leg offset
   along the commanded attitude (0.13–0.19 m at 20–30° of tilt).
2. **On a new path,** set `land` to the path point nearest P. If no point is within R_far, hop in
   place and log it.
3. **Catch up:** while P is already past p[land] along the segment toward p[land+1], `land += 1`.
4. **Advance or retry.** If P is within R_far of p[land], aim at p[land+1] with target apex
   H[land+1], then `land += 1`. Otherwise aim at p[land] again (a retry). After 2 retries, hop in
   place and log it.
5. **Past the last waypoint,** hop in place at p[N] with `desired_rel_z`, under the braking guard
   below.

Start with R_far = 0.8 m. A close waypoint is never skipped unless P is past it: stage 3 plans
contain short hops that build energy on purpose.

Each failure of a bare per-apex pop, and its guard:

| Failure (seen in the logs) | What a bare pop does | Guard |
|---|---|---|
| Warm-up apex. The path arrives 4–20 s before Play in 8 of 9 runs. | Uses up path[0] at the warm-up apex and WP1 at the first steering apex, so the first steered hop aims at WP2. | Only steering apexes decide. The target comes from where the robot will land (step 2: P ≈ path[0], so it aims at WP1), not from a counter. |
| Missed apex: 3→2 without 3→1 (09:44, 10:30). | A counter stays one behind for the rest of the path. | There is no counter. Step 3 re-derives progress from P at every steering apex, so a missed apex costs one uncontrolled hop. |
| Double or false trigger. | Pops twice. | One decision per real touchdown. A touchdown after a fall shorter than 0.15 s does not count. |
| Short or long landing. 09:42:55 landed 0.35 m short, and WP1 never popped. | Advances anyway and aims from the wrong place. | Within R_far, advance and re-aim from P (stage 4 adds a clearance re-check). Beyond R_far, retry at most twice, then hold and log. |
| Overshoot past WP_k. | Aims back at WP_{k+1} from beyond it. | Step 3 catches up. |
| New path mid-run (09:43:33, with a stale start). | Index points into the wrong path. | Step 2 re-syncs to the nearest point and holds if nothing is within R_far. DELETE no longer clears the list; the robot holds if no ADD arrives within 0.5 s. |

Remove:
- the distance pop (hopcopter.py L679–691); the index rule replaces `_load_next_waypoint`;
- `apply_follower_gating`. It never uses trajectory_start_x/y. It ran on the first plan in only
  2 of 9 runs, because /trajectory_start_position arrives after the marker. It drops points on
  turning paths.
- `pending_new_traj`, which is written but never read.

**Touchdown-height fix.** `leg_length` is the filtered Z at the first touchdown, 0.420 m, while the
real touchdown CoM is at 0.366 m upright. That bias makes LSE predict landings 0.13 m short and
makes braking commands 1.6° too steep (L745, L804–820).

**Braking tilt guard.** If the commanded landing tilt exceeds τ_max (20°), brake only partly.
Rotate the takeoff direction toward continuing the arrival until the tilt fits, and finish braking
over the next hop. It is needed in stage 1 because the stage 1 test ends in a brake. Today's
flat-world plan for (4,0) stops at a planned 20.3°, which is 24–27° as flown, before the 11%
overshoot.

**Verify:**
- Replay the 10:52 timeline. Apex 790.42 should aim at WP1 with 0.70; apex 791.23 at the goal with
  1.151.
- Run the flat world: `sitl_singleagent.sh -m crazyflie -x 0 -y 0 -w crazysim_default`,
  `elevation_map_publisher_node --ros-args -p scenario:=flat`, goal (4,0). Today's planner gives
  (0.05,0.05)→(2.15,0.05)→(4.05,0.05) at 0.70 m apexes.
- Pass when:
  - every decision row before a WP_k landing targets WP_{k+1};
  - the WP1 landing is a continuation (< 15°);
  - no landing exceeds 20°.
- The low_wall detour still needs stages 3–4.

### Stage 1 results (2026-10-29)

Implemented as `WaypointTracker` (`JumpLib/waypoint_tracker.py`) and
`LinearJumpingController.limit_landing_tilt`. Unit tests in `hopping_robot/test/test_waypoint_tracker.py`
(17) cover:
- the guard table;
- the 10:52 replay: apex 790.42 aims at WP1 with 0.70, and apex 791.23 aims at the goal with
  1.151;
- the tilt guard against fcn1's closed form.

Flat-world run (headless, 16:04): goal (4,0), then goal (1.0,1.5) mid-run. The robot flew both
paths and settled within 1 cm of each goal. It never crashed, against 11 of 14 runs ending at
WP1 before.

| Check | Result |
|---|---|
| Decision before each WP_k landing targets WP_{k+1} | Pass on both paths. No retries, holds or skipped decisions. |
| No landing above 20° | Pass, max 19.9° (the first goal brake, commanded 17.5°). Little margin, and the guard never fired. |
| WP1 landing < 15° | Fail: 18.6° on path 1, 15.2° on path 2. Both are continuations. |
| Foot landing prediction P | 0.021 m median error (max 0.061) against the actual landing; the CoM prediction was 0.105 m. |

Why WP1 exceeds 15°: neither cause is in stage 1's scope.
- On flat ground the planner planned a dogleg, (−0.05,−0.05) → (2.05,−0.65) → (4.05,0.05), so
  WP1 needs a 29° heading change. Stage 3's banking candidates should straighten it.
- LJC picks its own takeoff angle: the 45° "unreachable" fallback on path 1, and a steep 67° on
  path 2's short hop. Stage 4 flies the planner's α_leg instead.

Open (D4): achieved tilt runs 98–114% of commanded, so a landing capped at 20° commanded can
reach ~23°. Commanding at most ~18° would keep achieved landings under 20°.

## Stage 2: deliver the energy (problem 3)

**2a. Cut the push off at the target, with lead.** Replace JHC's push-time formula. Stop the push
when:

`(z − z_lo) + v̂²/(2g') + v̂·(a_T/g')·t_lead ≥ H`

- **v̂** is the world-frame vertical speed, estimated only from samples taken after liftoff. Stage 0
  found the odom twist is body-frame with a ~10 ms window, not a 50 ms mean. Rotated to world by the
  odom attitude, it matches the differentiated pose within 0.05 m/s from the second tick after
  liftoff on; the first tick is off by up to 0.47 m/s. So use the rotated twist and skip the first
  tick, or use `diff_v`.
- **t_lead** covers motor spin-down (τ_down 25 ms), the command latency and half a 10 ms tick.
  Stage 0 measured 15 ms from the cutoff tick to half thrust. Calibrate it in the first in-place
  run. Uncompensated, the apex overshoots by ~3–11 cm at 30000 (simulated).
- **g′** in the formula is the coast's own: 9.55 for a level coast at idle (stage 0). The motors
  spin down over the first ~50 ms of it.
- **Arm the push once per real stance:** on the first 2→3 after acc_z exceeded ~5 g for at least
  10 ms, and only with v̂ above ~1 m/s. Never re-arm in state 1. Today JSTO re-arms on any acc_z
  crossing, and spurious 3→2→3 transitions are logged at 09:44 and 10:30.
- **t_p_high** stays only as a cap.

This replaces all of JHC's faults, including the `apex_time` epoch bug that sends its first
estimate into the error branch.

**2b. Thrust 30000 (D1).** Thrust per PWM, from the Gazebo motor model at 0.3609 kg:

| PWM | thrust/weight | note |
|---|---|---|
| 21000 | 0.48 | today |
| 30000 | 0.81 | chosen |
| 34265 | 1.00 | hover |
| 51674 | 1.96 | JSTO reads acc_z 2.0 g during the push: false touchdown |
| 60000 | 2.53 | clamp in crazyswarm2 and in the firmware |

Below hover the push ends by the apex on its own. But a push held to the apex from v_z0 ≈ 3 m/s
would last ~1.6 s and peak ~2.5 m high, so keep a cap. Change in the same commit:
- t_p_high: 1.5 s → 0.4 s. That still delivers ≥ 3.6 J/kg for v_z0 ≥ 1.5 m/s, and ~8.4 J/kg at
  3 m/s.
- t_p_low: 0.04 s → 0. At 30000, 0.04 s alone adds ~1.1 J/kg, about 11 cm of apex.
- JSTO's initial `powered_climbing_end_timer`: 0.2 s → 0. It would inject ~6 J/kg on the first
  climb.

**2c. Measure the push-at-liftoff side effect (D2).** After a forward liftoff the body is still
tilted 12–34° along the motion, and levels with τ ≈ 0.11 s. At 30000 a push in that window adds
roughly 0.1–0.4 m/s of horizontal speed, i.e. up to ~0.3 m of range. Measure Δv_h against liftoff
tilt and push length on steered hops. Stage 4 corrects for it.

**Verify:**
- In-place sweep at targets 0.6–1.2 m. After calibrating the lead, the apex is within ±5 cm.
- No 3→2 transition happens during a push.
- Record the injection per kg available as a function of v_z0 within 0.4 s. It feeds stage 3's
  push-capacity gate.

### Stage 2 results (2026-10-29)

Implemented as `PushController` (`JumpLib/push_controller.py`, 16 unit tests) at PWM 30000.
`JumpingHeightController` is no longer used.
- **v̂** is the differentiated odom pose. At the first tick after liftoff it is off by +0.005 m/s
  (median, worst −0.57). The rotated twist is off by −0.15 there (worst −1.0). From the second
  tick on, both are right.
- **The push is armed** at the first climbing tick after a stance of at least 10 ms above 5 g,
  with v̂ > 1 m/s. A low first tick arms on the next one.
- **The cut is evaluated from the arming tick on.** v̂ only reads low on the first tick, so that
  tick can cost at most one extra tick of push (≤ 3 cm).

Problem 2 surfaced first: the first 30000 run lost all motors at its third landing, every time.
The cause was two SITL artifacts, both fixed (see the problem 2 entry under Deferred).

Headless runs, flat world, on the rebuilt firmware: 16:55 (in-place sweep 0.8 / 0.6 / 1.0 /
1.2 / 0.8 / 2.0, `t_lead` 0.03) and 17:06 (in place 0.8 / 0.6 / 1.0 / 0.8, then goal (4,0) and
goal (1.0,1.5), `t_lead` 0.026). Re-run with `python calibrate_hops.py`.

| Check | Result |
|---|---|
| Apex within ±5 cm in place, after calibrating the lead | Pass. With `t_lead` 0.026: −1.8 to +3.0 cm, median −0.1 cm (n = 92; targets 0.6, 0.7, 0.8, 1.0). The 0.03 sweep ran −1.1 cm (−3.2 to +0.8, n = 74), the same at every target from 0.6 to 2.0. |
| Lead | Each cut implies 21 ms (17–31) of thrust after it, so `t_lead` = 21 + 5 = 26 ms. The model predicted 16–19 ms of spin-down plus odom age. |
| No 3 → 2 during a push | Pass: 0 in both runs; JSTO logged 0 3 → 2 transitions. |
| Injection within 0.4 s against v_z0 | 3.1 J/kg at 1.5 m/s, 4.5 at 2.0, 6.0 at 2.5, 7.4 at 3.0, 8.9 at 3.5, 10.4 at 4.0 (apex 0.44, 0.68, 0.95, 1.25, 1.57, 1.92 m). This is the push model fitted to 75 pushes: 7.55 m/s² for the push time −1 ms, RMS 0.10 J/kg. The one 0.41 s capped push measured 8.75 J/kg at 3.40 m/s; the fit gives 8.82. |
| Push at liftoff (2c) | After a 28° liftoff, a 0.31 s push added 0.52 m/s along the motion (two hops). Two near-level liftoffs added ≤ 0.03. Integrating the logged attitude predicts 0.59–0.63. |

For stage 3:
- The capacity curve is ~15% below the plan's estimate (≥ 3.6 J/kg at 1.5 m/s, ~8.4 at 3 m/s). It
  is in terms of the liftoff speed that stage 0's η uses, so the planner gets v_z0 the same way.
  The steady thrust is the motor model's 7.98 (IMU 7.98).
- g′ per flight is bimodal: 9.26 when the idle motors reach PWM 1000 and 9.81 when they do not.
  The median is 9.55. This gives ±1.4 cm of apex scatter, within the target.

Seen in the steered runs, for stage 4:
- **The first steered hop of each path landed ~1 m past WP1.**
  - LJC still sizes the hop from the current apex and falls back to 45° ("unreachable" from a
    standing start). The stance then launched at 28° of body tilt.
  - The push, now running 0.31 s to reach the 0.7 m target, added 0.52 m/s.

  The tracker caught up (one `retry` toward the goal, then `goal`), and both goals were reached.
  Before stage 2, the weak push had hidden this.
- **Landings as flown reached 21.7° and 23.4°.** These were commanded at 20.0° (the first just
  under the guard; the second after the guard cut it from 23.7°): 109–117% of command. This is
  the D4 question again.

## Stage 3: plan the real push, account for momentum, publish the takeoff (problem 4, planner)

All behind one planner flag. With the flag off, `ballistic_parity_test` output stays byte-identical
(confirmed in the prototype). The parts below must ship together: on its own, the vertical-push
model at min_clearance 0.5 plans a 156° reversal with a ~40° landing.

**Model the push the robot makes.**
- **Vertical push.** With an instantaneous vertical push, the arc, apex, landing angle and injected
  energy come out exactly as today. Only the takeoff friction check moves, from the flight angle α
  to the leg angle α_leg: one closed-form bound on tan α. Add α_leg, v_leg and v_h to `Hop` and the
  CSVs.
- **Margin.** Plan α_leg ≥ 42°, so the controller's 40° guard has room for speed binning
  (0.25 m/s, about ±1.5–2°) and the η spread.
- **Push-capacity gate.** A hop's injection may not exceed what a 30000 push delivers from that
  hop's leg vertical speed within t_p_high (0.4 s), using stage 2's measured curve. Set
  e_inject_max to ~4 J/kg as the overall cap.
- **Energy carry-over across the stance.** Keep carrying the full landing energy × η (item 4), but
  make η a function of the redirection, η(θ). Keep it constant until stage 4 measures turns and
  straight running. Since the A* state carries the arrival direction, it is one lookup per edge.
  Which angle predicts the loss is itself open: the stance rotates the velocity least for a brake
  (~40°) and most for straight continuation (~75°).
- **Real-robot parameters.** mass 0.3609 kg; η 0.77 (stage 0); min_clearance 0.3 (D3);
  w_energy 1.86 (0.84 × 0.8/0.3609 keeps today's trade-off). The planner's g is 9.81, while the
  robot falls at 9.55 level and 9.0–9.3 steered. Stage 4 corrects the flight time for that.
- **Seed the first hop from the real warm-up landing,** ~4.0 m/s. Today's seed
  √(2g·h_initial/η) = 4.7 m/s assumes the leg alone sustains the 0.8 m warm-up apex; in fact a
  ~0.2 s push sustains it.
- **Relax `validate_params`' first-hop check** to "some first hop exists". Otherwise a realistic
  budget or the real seed kills the node at startup.

**Account for momentum.**
- **Arrival direction.** Each A* state carries the arrival direction (heading and descent angle)
  of its best path. The key stays (cell, speed bin).
- **Tilt cap.** Compute every landing's tilt with the same stance model as hopcopter (make k1 and
  k2 shared parameters), using the leg direction as the takeoff. Reject any tilt above τ_max = 20°.
  The start counts as a vertical arrival, and the goal must allow a stop.
- **Banking.** Besides the least-injection angle, try 4 steeper angles, each landing one speed bin
  faster (closed form). Two is not enough at 5 J/kg. This removes the sideways, energy-burning
  hops. It departs from the reference planner's least-injection rule, so document that.
- **Re-check the returned path** with exact headings and descent angles, including the stop. A
  cheaper path found later can overwrite a state's stored direction after its children were
  generated. If the check fails, report NO PATH.

**Publish the takeoff (D5).** Add a `HopPlan` message in a new `hopcopter_msgs` package, published
just before the marker pair with the same stamp. Each hop carries:
- landing xy and ground z, heading, α_leg, v_h, target apex, injection per kg;
- the expected arrival velocity at its landing point, and the landing attitude and tilt the
  planner computed for the next takeoff.

Keep the id=400 Marker for RViz.

**Expected plans** (offline: 4 J/kg, seed 4.0 m/s, η 0.77, min_clearance 0.3, τ_max 20°;
~3–4.5 s per plan at -O2):

| Goal | Path | Hops (m) | Injection (J/kg) | Push at 30000 (s) | Landing tilts |
|---|---|---|---|---|---|
| low_wall (4,0) | (0.05,0.05)→(1.65,0.05)→(3.55,0.05)→(4.05,0.05) | 1.60 / 1.90 / 0.50 | 3.4 / 3.8 / 2.0 | 0.16 / 0.15 / 0.06 | ≤ 14.3°, stop 3.9° |
| low_wall (4,1.5) | (0.05,0.05)→(1.65,0.75)→(3.55,1.35)→(4.05,1.55) | 1.75 / 1.99 / 0.54 | 3.4 / 4.0 / 2.0 | 0.17 / 0.16 / 0.06 | ≤ 16.6°, stop 4.3° |
| flat (4,0) | (0.05,0.05)→(1.35,0.05)→(2.65,0.05)→(4.05,0.05) | 1.30 / 1.30 / 1.40 | 2.4 / 1.6 / 1.6 | 0.10 / 0.06 / 0.06 | ≤ 15.8° |

For comparison, legacy at node defaults plans the 10:53 detour, with tilts of 15.4°, 29.7° and
22.0°.

**Verify offline:**
- `ballistic_parity_test` stays byte-identical with the flag off.
- Every landing in every plan is ≤ τ_max, including the stop.
- Every hop passes the controller's stage 4 solve and the push-capacity gate, using chained,
  unbinned speeds across η 0.71–0.85.
- Re-run the 13-case matrix and stairs. Search effort should stay near legacy (stairs 25.5k
  expansions against 25.8k).

### Stage 3 results (2026-10-29)

Implemented behind `search.momentum_aware`, which is on by default, with the robot's parameters
as the node's defaults (`ballistic::robot_params()`). `PlannerParams` keeps the reference
defaults for `ballistic_parity_test`. Implementation notes, including where it differs from
the text above:
- **Vertical push**: `feasible_alpha_interval_vertical_push`, with the 42° leg-angle floor
  inside the closed-form bound.
- **Push-capacity gate**: stage 2's fit (7.55 m/s², −1 ms, g′ 9.55, 0.4 s). It is checked at the
  bottom of the arrival's speed bin, not at its centre. At the centre, the reference-config
  stairs plan put two pushes at 99% of capacity, and they failed with real speeds at η ≤ 0.76.
  With the robot's parameters the gate never binds: the 4 J/kg cap binds first.
- **Stance model**: `ballistic::stance` is fcn1. Only k2 is a parameter (`stance.k2`); k1 sets
  the liftoff attitude, which no planner check uses.
- **Banking**: each of the 4 banking angles is checked on its own. The prototype skipped them
  whenever the least-injection angle failed the tilt check. All 15 plans and the run times came
  out the same either way.
- **η(θ)**: η stays one constant. Each stance's redirection angle θ_v is computed
  (`path_stances()`), but nothing depends on it yet.
- **Exact re-check**: it never failed in the 15 cases.
- **Seed**: √(2g·h_initial) = 3.96 m/s, binned to 4.0.
- **`validate_params`**: it expands the start once on flat ground with the search's own edge
  test.
- **`HopPlan`**: in the new `hopcopter_msgs` package, with one `PlannedHop` per hop. It also
  carries the start's stance (the warm-up landing that launches hop 1) and the model constants.
  A live test with the map publisher and a fake pose showed it with the marker pair's stamp.
- **Speed**: `clearance_floor_alpha` now walks the arc without allocating, about 5% faster in
  both modes.

Offline check: `ros2 run ballistic_motion_planner ballistic_plan_check` (15 cases, ~2 min).

| Check | Result |
|---|---|
| `ballistic_parity_test` byte-identical with the flag off | Pass |
| Every landing ≤ τ_max, the stop included | Pass: at most 17.0° over 14 plans, stops ≤ 16.7°. With the same parameters, the reference search exceeds 20° in 5 of its 12 plans (up to 25.2°), mostly at the stop. |
| The three expected plans | Pass: identical paths, injections and tilts (low_wall (4,0) ≤ 14.3°, stop 3.9°; low_wall (4,1.5) ≤ 16.6°, stop 4.3°; flat (4,0) ≤ 15.8°). |
| Chained, unbinned speeds at η 0.71–0.85: 40° guard and push capacity | Pass for all 14 plans. The robot's plans keep α_leg ≥ 46.4° and pushes ≤ 70% of capacity; the reference-config stairs plan reaches 98%. |
| 13-case matrix | 12 plan. `stairs` (0.6 m risers up to 3.0 m) has no path within 4 J/kg; the reference search finds none either. |
| Search effort | Stairs, reference configuration: 26,120 expansions against 25,790 (15.1 s against 10.1 s). Robot parameters, against the reference search on the same problem: 1.1–1.8× the expansions on the long goals; up to 6.7× on short ones, where the reference flies one long hop that cannot stop within 20°. |
| Time per plan | 3.9 s (flat (4,0)) to 7.2 s (tall_stairs); low_wall (4,0) 5.1 s. The estimate above was 3–4.5 s. The reference search takes 1–4 s on the same problems. |

For stage 4:
- **hopcopter does not read `HopPlan` yet.** Until it does, it flies the new waypoints and
  target apexes with LJC's own takeoff angle. The plans are now within the push budget and
  straight (flat (4,0): 1.30 / 1.30 / 1.40 m, where stage 1's run flew a dogleg).
- **Plans now contain short hops on purpose**: low_wall's final 0.5 m hop, and tall_stairs' two
  0.10 m hops in place that rebuild speed before a riser. A hop shorter than the landing error
  can be skipped by the tracker's catch-up, which drops its energy.
- **The planner still uses g = 9.81** for arcs and flight times. The robot falls at 9.55 level
  and 9.0–9.3 steered; stage 4 corrects the flight time.

## Stage 4: fly the published takeoff (problem 4, controller)

At the apex before landing on p[k], replace `LJC.jumping_planning()` with feed-forward plus
correction.

**Feed-forward.** Take hop k+1 from `HopPlan`: heading, α_leg, v_h, target apex H, expected
arrival, and planned landing attitude.

**Correction, from what the robot is actually doing:**
- **Heading** runs from the predicted foot landing P to p[k+1].
- **Horizontal speed.** Re-solve v_h for the actual distance X' = |p[k+1] − P| with the planner's
  flight time, `T = √(2H/g'_up) + √(2(H−Z)/g'_down)`. The climb coasts level at 9.55 after the
  push. The fall is steered, at 9.0–9.3 (stage 0), so fit g'_down against the commanded tilt. Then
  subtract the calibrated biases, which all push landings long:
  - the push-at-liftoff Δv_h (stage 2c);
  - the finite push, about +3–8% at 30000;
  - the stance displacement: the liftoff CoM sits 0.05–0.12 m ahead of touchdown, computed from
    fcn1's own attitudes.

  Uncorrected, these put landings 0.3–0.9 m long. LJC's `velocity_gain` 0.85, which today shortens
  every aim by 15%, goes away with `jumping_planning`; these corrections replace it.
- **Leg angle.** α_leg = acos(v_h / v_leg), with v_leg = √η·|measured arrival|.
- **Landing attitude.** fcn1 computes it from the measured arrival and
  (cos α_leg cos ψ, cos α_leg sin ψ, sin α_leg). This is the controller's own cancellation.
- **Log** the planned against the commanded landing attitude, and the planned against the
  predicted landing point, every hop. That is the running record of how far off-plan the robot is.

**Guards:**
- **Leg alone overshoots H.** Use the steep no-push root (apex ≥ H). The shallow root would fly
  under the planned clearance.
- **Leg cannot supply v_h** (α_leg < 40°). Pump in place to the apex the hop needs,
  H_pump = v_h² / (2g·η·cos²40°), capped by the push budget. Pumping to H can never rescue a hop
  longer than ~2.7·H. Allow at most 2 retries, then hold and log.
- **Clearance on re-aim.** When P is off the waypoint, re-check the arc from P to p[k+1] against the
  cached elevation map, and raise H or retry if it fails.
- **Tilt.** The stage 1 tilt guard applies when the commanded tilt exceeds 20°.
- **Push target** is H, via the stage 2 cutoff.

**Verify, in order:**
1. **Flat-ground range regression.** Fly straight hops of 1.0–2.5 m. Fit the residual landing
   bias, and report landing error against hop length (D6).
2. **Redirection tests on flat ground.** Fly straight continuation at speed, then doglegs of 30°,
   45° and 60°. Compare the measured takeoff direction and landing tilt with fcn1's prediction, and
   re-fit k1/k2 if they are off by more than 3°. Measure the energy kept against the redirection
   angle; if it varies, fit stage 3's η(θ).
3. **low_wall (4,0) at min_clearance 0.3.** Pass when the apex is within ±0.1 m of the target,
   landings are within 0.35 m of their waypoints, and no landing exceeds 20°.

### Stage 4 results (2026-10-29)

Implemented as `HopModel` (`JumpLib/takeoff_solver.py`, 30 unit tests) and hopcopter's apex
logic; the per-hop record is in `apex_decisions_*.csv` and `calibrate_hops.py`'s stage 4 report.
Implementation notes, including where it differs from the text above:
- **Feed-forward and correction.** The controller re-solves every takeoff with its own forward
  model, from the predicted foot touchdown P and arrival v_in. It does not add corrections to
  HopPlan's numbers. HopPlan's apex (also the marker's z) is the target; its heading, α_leg, v_h,
  arrival and landing attitude are logged next to the commanded ones. On long hops the commanded
  α_leg runs 4–8° steeper than the plan's, because the plan's point-mass arc has no stance shift,
  no kick and an instant push. The commanded landing tilts run up to 4° below the plan's, because
  of the refit k2.
- **The corrections are terms of the forward model**, not biases subtracted afterwards:
  - the stance shift, from the stance model's liftoff attitude;
  - the push kick, 6.69 m/s² × sin(liftoff tilt) over stage 2's leveling profile;
  - the finite push's climb;
  - g′ of the fall from the next landing tilt;
  - the next landing's foot offset, looked ahead with the same model. HopPlan's axis assumes the
    planner's g and arrival.
- **Touchdown prediction**: the odom twist rotated to world frame, the vertical speed at the apex
  tick, and the ground under P. The foot touchdown is predicted to < 1 mm median, 2.6 cm max (stage
  1: 2.1 cm median).
- **Guards** as specified, except the clearance re-check. It flies at the least apex that passes
  the planner's test, plus 1 cm, instead of raising H in steps. Planned wall hops sit exactly on
  that apex, so landing a few cm behind the waypoint already fails the test: the raises were
  3–6 cm on low_wall. The push target now includes the takeoff ground.
- **Firmware attitude PID wind-up** (found in step 1). After a continuing landing the push is
  short (40–65 ms) and ends mid-leveling. The idle motors cannot finish, so the body coasts 3–4° past
  level while the rate and attitude integrators (KI 500 and 3) wind up. At the apex they drove
  the fall 1–1.5° past the landing attitude, and the stance turned that into a takeoff 2.5–4° too
  steep: continuing hops landed 0.1–0.3 m short. Fix: zero thrust for one tick at each apex, which
  makes the firmware reset its attitude PIDs. The median touchdown attitude error is now 0.02–0.04°.
- **Step 2 refits.** fcn1's takeoff angle was off by 1.5–2.5°. That is under the 3° threshold
  above, but it was systematic and moved landings 0.05–0.1 m, so the constants were refit:
  - k1 1.688 → 1.50: the stance shift was overpredicted 35%, the kick 2–3×.
  - k2 2.469 → 2.53, rising to 2.70 for arrivals descending more steeply than 75–85°. Measured:
    2.65–2.72 after first hops from in place, 2.46–2.62 on moving arrivals by run (straight
    continuation high, doglegs low).
  - η varies with the redirection, so η(θ) = 0.77 up to θ_v 30°, rising to 0.81 at 60°. Measured:
    0.765 at 15–35° (48 stances), 0.787 at 45–55° (24), 0.816 at 55–65° (12), each ±0.006.
  - The planner keeps η 0.77 and k2 2.469. Both err on the safe side: less energy and more tilt
    than the robot needs.

Final runs, with the final constants (headless, 19:29–19:33). `calibrate_hops.py` on each
`attitude_*.csv` reproduces these.

| Check | Result |
|---|---|
| 1. Range regression: straight hops of 1.0–2.7 m, flat, from rest and at speed (15 hops) | Landing error against hop length (D6), median miss (max): ≤ 1.5 m 0.03 m (0.08); 1.5–2.0 m 0.08 m (0.20); 2.0–2.7 m 0.18 m (0.19). Residual along-track bias +0.02 m up to 1.5 m and +0.08 m at 1.5–2 m, both within the scatter; no bias term added. First hops from hopping in place land within ±0.02 m. |
| 2. Redirection: straight continuation, doglegs of 30°, 45° and 60° left and right, 1.3–1.5 m hops (17 hops) | Median miss 0.053 m; along −0.018 m (sd 0.070), across sd 0.094. One miss of 0.36 m: a 45° turn at speed whose touchdown attitude was 1.6° off. |
| 2. Stance model against fcn1 | Refit as above (fcn1 off by 1.5–2.5°). |
| 2. η against the redirection angle | Varies by 0.05 over 15–65°: η(θ) as above. |
| 3. low_wall (4,0) at min_clearance 0.3, and back to (0,0) | Pass. Apex within −0.022 to +0.029 m of target. Landings within 0.054 m of their waypoints on the way out and 0.103 m back. Landing tilt ≤ 12.6°. The three earlier runs with interim constants also passed (worst 0.16–0.22 m). |
| Apex against target, every push cut at the target | ±3 cm in every run. Short continuing pushes (40–65 ms) overshoot by 2–5 cm: stage 2's one-tick cutoff quantization. |
| Pump guard, live: a 2.8 m first hop from rest | Pumped in place to 1.07, 1.24 and 1.37 m (each apex within 1 cm), then held as specified. The robot drifted 0.09 m back while pumping, which made the hop 2.89 m, just beyond reach. |
| Apex computation | ~4 ms, inside the 10 ms tick. |

For later:
- **Attitude authority at speed is now the largest error.** Three of the four misses above
  0.15 m in the final runs came with a 1.6–2.1° touchdown attitude error. Each was a fast hop
  (v_h ≳ 1.8 m/s) whose landing attitude needed a forward correction during the fall. The fourth
  (0.20 m) was the stance turning 1.9° further than the model. Corrections backward complete;
  forward ones stall. The
  motor plugins model rotor drag and rolling moments (`model.sdf.jinja`; motor 4's
  `rollingMomentCoefficient` is 1e-6, 10× the other three). At idle (PWM 1000, at the plugin's
  cut-off) the motors correct about as much as those moments push. Options:
  - a higher thrust during the fall, for example PWM 3000: g′ ~9.0, ~0.4 J/kg per hop, and stage
    0's g′ and stage 2's coast need re-measuring;
  - a feed-forward on the attitude command.
- **D4** can stay at 20°. The overshoot behind the 98–117% achieved/commanded tilts was the PID
  wind-up; with the reset, landings tilt as commanded (median error 0.03°). The worst landing as
  flown was 18.8°.
- **D6** (hop-length cap): errors grow past 2 m because of the attitude limit above. A 2.0 m cap
  would keep landings within ~0.2 m in these runs.
- **The pump height** (v_h² / (2g′η cos²40°) + 5 cm) undershoots after the first pump: a faster
  arrival shortens the climb, so the hop needs more v_h. With two retries it reaches ~2.7 m from
  rest. The planner never plans such hops at 4 J/kg.
- The stance and kick constants were fit in the sim only.

## Deferred

- **Problem 2: fixed 2026-10-29** (found in stage 2). A `supervisor.info` log showed armed →
  flying → tumbled + crashed at a stance, which locks the motors. The firmware's `acc.z` showed
  why:
  - `sensors_sitl.c` computes an accelerometer scale from the 1000 samples after the gyro bias
    settles, about 1.1–2.1 s after Play. The robot is hopping then, and the scale came out 0.538.
    Idle coasts then read 0.104 g instead of 0.056, just over `isTumbledCheck`'s 0.1 g free-fall
    limit, so the tumble clock runs through the whole flight.
  - The plugin's unchecked int16 cast wraps stance readings above 16 g to negative values: the
    stance reads −14 g, "upside down", while that clock has already run past 100 ms.

  Whether a run hits this depends on how the start lines up. At 21000 the scale happened to stay
  above 0.56. The steered WP1 falls, with more thrust, fit the same mechanism. Fixed: the scale
  stays at 1, and IMU counts saturate. With the supervisor enabled, in-place and steered hops up
  to 33° stayed armed and flying. In the source, `stabilizer.c` has had the supervisor commented
  out in SITL since 2025-10-15. The rebuilt `cf2` is the first binary built with that edit.
- **Replanning.** The planner takes its start from `/cf_1/pose`. A duplicate goal at 09:43 planned
  from (0.05, 0.05) while the robot was at (0.78, 1.64). Mid-run goals, for example from
  nlp_goal_node, need a start from the predicted landing, with the arrival velocity.
- **Free stop at the goal.** In the expected plans the final 0.4–0.5 m settling hop injects
  ~2 J/kg only to avoid the cost's momentum-loss charge, which is pointless where the robot stops.
  Without it the final apex would be ~0.87 m instead of 1.08 m. Prototype exempting that hop
  before adopting it.
- **Hop-length cap** (D6): stage 4's range regression is in; see its results for the data.

Every stage gets a CHANGELOG entry under [Unreleased] giving the reason for each new value. Update
CLAUDE.md and README wherever a stage changes what they describe: the waypoint queue, JHC, LJC's
takeoff angle, and the Marker-only contract.
