# Plan: Reach the Commanded Landing Tilt Faster

Status: proposed, not implemented. Written 2026-09-23.

## Problem

`hopcopter` computes a landing attitude once per hop, at the apex, inside the
`jumping_state_old == 3 and jumping_state == 1` block in `hopcopter.py`. The robot must
reach that attitude before touchdown or the inverted-pendulum stance redirects its velocity
by the wrong angle and the hop lands short.

Measured on `data/data_ballistic_planner/hop_tracking/hopcopter_2026-09-23_09-47-07.csv`,
goal `x = 1.0`, first aimed apex:

| quantity | value |
|---|---|
| commanded landing tilt | 7.59 deg |
| tilt at touchdown | 3.57 deg |
| fall time available | 0.378 s |
| hop requested by `LinearJumpingController` | 0.85 m |
| hop delivered | 0.505 m |

The robot reaches 47% of the commanded tilt and delivers 59% of the requested hop. Those are
the same shortfall. With the tilt achieved, the hop would have landed at 0.85 m, inside the
0.4 m `current_goal_tolerance`, and the goal would have retired one hop earlier.

## Why it is slow

Not torque saturation. `powerDistributionCap` in `power_distribution_sitl.c` only reduces
motor commands when one exceeds 65535, so at base thrust 1000 the mixer can still drive two
motors to 17,383 PWM.

| quantity | value |
|---|---|
| angular acceleration available at thrust 1000 | 2340 deg/s^2 |
| angular acceleration used | 50 deg/s^2 |
| headroom | about 47x |

It is loop bandwidth. The firmware runs a cascade at 500 Hz: an outer PI on angle error
(`pid_attitude`, kp 6.0, ki 3.0, kd 0.0) producing a rate setpoint, then an inner PID on gyro
error (`pid_rate`, kp 250, ki 500, kd 2.5) producing mixer counts. The outer time constant is
1/kp = 0.167 s against a 0.378 s fall, so even perfect inner tracking arrives at about 90% of
the command, most of it near touchdown. Measured performance is another factor of two below
that.

These are stock Bitcraze gains for a bare 27 g Crazyflie. This airframe is 0.361 kg with
74.25 mm arms and a 0.4 m leg, pitch inertia about 1.2e-3 kg m^2. The cascade was never tuned
for it.

## Constraints

Two options are ruled out by the project owner and are not part of this plan:

- Do not raise the falling thrust above 1000 in `hopcopter.py` line ~809.
- Do not tilt during the **powered** portion of the climb.

## Steps

### 1. Instrument before changing anything

The controller regulates the firmware Kalman estimate while the CSVs log Gazebo ground truth.
In free fall the accelerometer carries almost no gravity reference, so the two can disagree.
Separate the two failure modes before tuning.

Add to the crazyswarm2 log config for `cf_1`:

- `stabilizer.pitch`, `stabilizer.roll` (firmware attitude estimate)
- `controller.cmd_pitch`, `controller.cmd_roll` (mixer counts out of the rate PID)
- `pid_attitude.pitch_outP` (rate setpoint contribution)

Log these alongside the Gazebo attitude across one fall and compare.

- If the firmware estimate reaches the commanded angle while ground truth does not, the
  problem is the estimator and more gain will not help. Stop and reconsider.
- If both track together and both fall short, it is bandwidth. Continue to step 2.

### 2. Raise the attitude gains

Cheapest change with the largest effect. No firmware rebuild and no `hopcopter.py` edit.

`ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml` already pins these under
`firmware_params` at the stock values, and every one is a runtime parameter
(`PARAM_PERSISTENT` in `attitude_pid_controller.c`).

Starting point:

| parameter | current | try |
|---|---|---|
| `pid_attitude.pitch_kp` | 6.0 | 20 to 25 |
| `pid_attitude.roll_kp` | 6.0 | 20 to 25 |
| `pid_rate.pitch_kp` | 250.0 | raise if the inner loop cannot follow |
| `pid_rate.roll_kp` | 250.0 | raise if the inner loop cannot follow |

At `pitch_kp` of 25 the outer time constant drops to 0.04 s, which settles well inside the
fall. A 7.6 deg error then commands 190 deg/s, which briefly saturates the rate PID at int16.
That is acceptable and means full torque.

Sweep the gain and read the achieved landing tilt out of
`data/data_ballistic_planner/hop_tracking/*.csv`. Target is the achieved tilt matching the
commanded tilt computed from the same apex.

**Watch for**: because `PWM2OMEGA` in `CrtpUtils.h` returns zero below 1000 PWM, the low-side
motors are already at zero thrust and cannot go lower. All torque comes from pushing two
motors up, so torque and upward force are coupled. At 840 deg/s^2 that is 6.8% of weight
pushing up; at the 2340 deg/s^2 ceiling it is 19%. Aggressive gains will slow and lengthen the
fall, which `falling_time` in `hopcopter.py` does not model. Expect a small bias in the
`LandingStateEstimator` prediction and watch it as the gain goes up.

**Also note**: `MIN_THRUST` in `crtp_commander_rpyt.c` is 1000 and the test is
`rawThrust < MIN_THRUST`. The falling thrust of 1000 passes by one count. At 999 the setpoint
thrust becomes 0, which makes `controller_pid.c` call `attitudeControllerResetAllPID()` and
zero roll, pitch and yaw every tick. Do not lower the falling thrust.

### 3. Start the slew when the powered burst ends

Respects the no-tilt-during-powered-climb constraint and roughly doubles the time available.

The powered burst lasts `powered_climbing_end_timer`, returned by `JumpingHeightController.step`
and bounded by `t_p_low` 0.04 s and `t_p_high` 0.7 s. It was likely under 0.15 s in the run
above. After `powered_climbing_end_flag` clears, `jumping_state` stays 3 while the robot
coasts upward at thrust 1000, roughly another 0.2 to 0.3 s. Thrust there is identical to the
falling phase, so nothing about the hop energy changes.

Today that window commands roll and pitch of zero (`hopcopter.py` line ~818) and the slew only
begins at the apex.

Change: run the `LinearJumpingController` computation every tick from the end of the burst
onward instead of once at the 3-to-1 transition, and command `LJC.roll` / `LJC.pitch` during
the unpowered part of state 3 as well as state 1.

The apex height is not yet known during the coast, so predict it from the current state rather
than reading `Z_f`:

```
h_apex = (pos_z - leg_length) + vel_z**2 / (2 * G)
```

Use that in place of `jumping_height_record` while coasting, then let the existing apex
computation refine it at the 3-to-1 transition.

Combined slew window goes from about 0.38 s to about 0.6 s.

### 4. Fall back to command overdrive if the gain sweep hits stability trouble

Pure `hopcopter.py` change, no firmware touch. The response is a first order lag with a
measurable time constant, so invert it:

```
commanded = desired / (1 - exp(-fall_time / tau))
```

Identify `tau` once from a run. At the measured 47% delivery over a 0.378 s fall, `tau` is
about 0.6 s and the factor is about 2.1, so 16 deg commanded to arrive at 7.6 deg. It adapts
correctly as `fall_time` changes, which matters because shorter hops need more overdrive.

Clip the result so the commanded landing tilt never leaves the friction cone. This cancels the
lag rather than removing it, so treat it as a stopgap.

### 5. Rate mode, if steps 2 and 3 are not enough

`stabModeRoll` and `stabModePitch` in `crtp_commander_rpyt.c` are runtime parameters and accept
`RATE` (0). In rate mode the outer loop is bypassed and `hopcopter` commands the angular rate
directly:

```
rate_cmd = (desired_angle - current_angle) / remaining_fall_time
```

recomputed at 100 Hz from `gzgt_robot_euler`, which `hopcopter` already has. This is a deadline
problem, and commanding rate against a deadline is more direct than asking a regulator to
settle.

Larger rework: `hopcopter` then owns the angle loop in every phase, including stance and climb
where it currently sends zero. Set the mode once at startup rather than switching it mid-flight.

## Verification

Per gain setting, run the `x = 1.0` goal from a standstill and check, from
`data/data_ballistic_planner/hop_tracking/`:

1. Achieved landing tilt versus the tilt commanded at that apex. Target is parity.
2. Hops needed to bring `land_x` inside 0.4 m of 1.0. Currently 3 after the plan arrives.
   Target is 1.
3. `previous_apex_h` stays near 0.8 and does not collapse, and the hop period stays near
   0.9 s. A collapsing apex or a halving period means a false touchdown, not a real hop.
4. `land_z` stays near zero. A `land_z` well above ground means the `acc_z` threshold in
   `JumpingStateTrackerOnboard` fired mid-air, which corrupts every downstream estimate.

## Related, deliberately out of scope here

Tracked separately, not part of this plan:

- `norm_d_max` in `LinearJumpingController.jumping_planning` models only the energy limit
  (`2 * jumping_altitude + v_h^2 / g`) and ignores the attitude limit entirely. At the tilt
  currently achieved the attitude limit is about 0.42 m per hop against an energy limit of
  1.40 m, so attitude binds by more than a factor of three. Once attitude tracking improves
  the two constraints converge; the guard should become `min(energy, attitude)`.
- `norm_d_max` also ignores the energy the powered climb is about to add, which is why the
  controller saturates to its 45 degree max-range branch and then overflies the target.
- The touchdown detector in `JumpingStateTrackerOnboard` thresholds raw body-frame IMU z and
  degrades when the robot pitches hard.
- `u_x = norm_v * d_x / norm_d` in `jumping_model.py` divides by `norm_d` with no guard.
- Waypoints retire on a geometric tolerance evaluated every tick, so they can retire mid-flight
  as the robot passes overhead without landing near them.
