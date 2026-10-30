# What to do after stage 4

Written 2026-10-29, right after stage 4 was committed (`eb847e75`). Read this top to bottom when
you come back. Part 1 and Part 2 are checks you run; Part 3 is decisions only you can make.

## Where things stand

- All five stages of [follow_planner_plan.md](follow_planner_plan.md) (0–4) are implemented and
  committed. The robot now flies the planner's plan: one waypoint per hop, the planned apex, and
  a takeoff solved at every apex from where it is about to land
  (`ros2_ws/src/hopping_robot/hopping_robot/JumpLib/takeoff_solver.py`).
- Stage 4 was tested by Claude in a headless sim (no Gazebo window). **You have not watched it
  fly yet.** Part 2 is you doing that in your normal 7-terminal setup.
- Known weak spot: fast hops (horizontal speed above ~1.8 m/s, usually hops longer than ~2 m) can
  land 0.2–0.36 m off. Everything else lands within ~0.1 m. See Part 3, decision 1.

---

## Part 1 — Build and unit tests (5 minutes)

```bash
cd ~/CrazySim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

cd ~/CrazySim/ros2_ws/src/hopping_robot
/usr/bin/python3 -B -m pytest -p no:cacheprovider \
  test/test_waypoint_tracker.py test/test_push_controller.py test/test_takeoff_solver.py
```

**Pass:** the last line says `63 passed`.

If `colcon build` fails on `hopcopter_msgs` with "No module named 'em'", a Python virtualenv is
first on your PATH. Deactivate it (`deactivate`) and build again.

Optional (the planner did not change in stage 4, so these should still pass as before):

```bash
ros2 run ballistic_motion_planner ballistic_parity_test   # output unchanged
ros2 run ballistic_motion_planner ballistic_plan_check    # ~2 min, ends with "0 case(s) failed"
```

---

## Part 2 — Fly it yourself (about 1 hour)

### Setup for every test

Use the 7 terminals from `README.md` ("Running the Simulation"), with two changes per world:

| Test world | Terminal 1 (`sitl_singleagent.sh ... -w <world>`) | Terminal 5 (elevation map) |
|---|---|---|
| low_wall | `-w crazysim_low_wall` (as in the README) | `ros2 run elevation_map_publisher elevation_map_publisher_node` |
| flat | `-w crazysim_default` | `ros2 run elevation_map_publisher elevation_map_publisher_node --ros-args -p scenario:=flat` |

**Terminal 7 (hopcopter):** its terminal is flooded with `At RPYT_commands` prints. To see only
the useful lines, start it like this instead:

```bash
ros2 run hopping_robot hopcopter 2>&1 | grep --line-buffered -v "At RPYT_commands\|At robot_pose\|sample time = 0"
```

**Every test starts the same way:**
1. Start all 7 terminals, then press **Play** in Gazebo.
2. Wait until hopcopter prints `Warm-up done (2 touchdowns); steering toward goal from this apex`.
   The robot hops in place at the start.
3. Send the goal (or path) from an 8th terminal (sourced like the others).
4. When the test is done, stop hopcopter with **Ctrl+C** first (that closes its CSV files), then
   the rest.

### What the hopcopter terminal tells you

| Line | Meaning |
|---|---|
| `HopPlan for this path: 3 hops` | It received the planner's takeoff plan. Should appear right after every planner goal. |
| `New path: (x, y) -> ...` | The waypoints it will fly. |
| `Apex: landing at (x, y), track -> aim at (x, y), waypoint k, apex h` | Normal: at the top of a hop it predicts where it will land and aims the next hop at waypoint k. |
| `... goal -> aim at ...` | It reached the last waypoint and hops in place there. |
| `... retry -> ...` (warning) | It landed more than 0.8 m from the waypoint and aims at it again. One now and then is fine. |
| `... hold ...` (warning) | It gave up on the path after 3 misses and hops in place. **A hold in Tests 1–3 is a failure.** |
| `Hop to (...): push, apex raised 0.05 for clearance` (warning) | It landed off the waypoint and flies higher to clear the wall. Normal on the wall hop. |
| `Hop to (...): pump, hopping in place to 1.07` (warning) | The next hop is too long for its energy, so it hops higher in place first. Expected only in Test 4. |

### After each test: run the report

From the repo root, with the repo's `.venv` Python (the one that has pandas):

```bash
cd ~/CrazySim
python calibrate_hops.py
```

It reads the newest run automatically. Scroll to the section
`== Takeoff solve (stage 4): each steered hop against the model and the plan`. The lines to read:

| Report line | What it means | Pass |
|---|---|---|
| `landing against the target, by hop length (D6)`, the `miss median ... max ...` numbers | How far each landing was from the waypoint it aimed at | max ≤ 0.35 m |
| `landing tilt as flown [deg]: max ...` | Worst landing tilt | ≤ 20° |
| `apex - target, pushes cut at the target [m]` (in the earlier `== Push cutoff` section) | Apex height against the planned apex | min/max within ±0.1 m |
| `touchdown attitude against the command [deg]` | How well the body reached the commanded landing attitude | median under ~0.1°; single hops up to ~2° are the known issue |
| `landing prediction at the apex [m]` | Where it predicted the landing vs. where it landed | median under ~0.01 m |

In the per-hop table above those lines, `along` and `across` are each landing's error along the
hop and sideways (+ along = long), and `att_err` is that hop's attitude error. A big miss with a
big `att_err` (over ~1.5°) is the known fast-hop issue, not a new bug.

Write your results in the table at the bottom of this file as you go.

### Test 1 — low_wall (4,0) and back (the plan's acceptance test)

World: **low_wall**. After warm-up:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

The planner takes ~5 s. Expect the path `(0.05, 0.05) -> (1.65, 0.05) -> (3.55, 0.05) ->
(4.05, 0.05)` (the start may read −0.05): three straight hops, the second one over the wall. Let
it settle at the goal for ~10 s, then send it back:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**Pass:** no `hold`; it clears the wall both ways without touching it; it ends hopping in place at
each goal. In the report: miss max ≤ 0.35 m, landing tilt ≤ 20°, apex within ±0.1 m.
Claude's runs: misses ≤ 0.10 m (0.16–0.22 m in earlier runs), apex ±0.03 m, tilt ≤ 12.6°.

### Test 2 — flat goals

World: **flat**. After warm-up, send goal (4, 0); after it settles (~15 s), send goal (1.0, 1.5):

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
# wait ~15 s, then:
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 1.0, y: 1.5, z: 0.0}, orientation: {w: 1.0}}}"
```

**Pass:** same as Test 1. Expect about three hops of ~1.3 m to (4, 0), all straight (before stage
3 the first path was a dogleg).

### Test 3 — hand-made paths: long hops and turns

This repeats Claude's range and turn tests, so you can see the known issue yourself. World:
**flat**. These commands publish a path directly to hopcopter, as the planner would (there is no
HopPlan for them, which is fine). Each point is `{x, y, z}` with z = the target apex of the hop
that lands there.

**The first point of each path must be where the robot is hopping right now** (it ignores a path
that starts more than 0.8 m away). Send the next command only after the previous path is
finished and the robot hops in place at its last point.

```bash
# 3a. straight hops of 1.0, 1.5, 2.0 and 2.5 m along +x, starting from (0, 0)
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 0.0, y: 0.0, z: 0.8}, {x: 1.0, y: 0.0, z: 0.75}, {x: 2.5, y: 0.0, z: 0.75}, {x: 4.5, y: 0.0, z: 0.8}, {x: 7.0, y: 0.0, z: 0.85}]}"

# 3b. back along -x: 1.2, 2.2, 2.0 and 1.2 m
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 7.0, y: 0.0, z: 0.8}, {x: 5.8, y: 0.0, z: 0.75}, {x: 3.6, y: 0.0, z: 0.8}, {x: 1.6, y: 0.0, z: 0.8}, {x: 0.4, y: 0.0, z: 0.75}]}"

# 3c. a 30-degree left turn at the first waypoint
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 0.4, y: 0.0, z: 0.8}, {x: 1.8, y: 0.0, z: 0.75}, {x: 3.01, y: 0.7, z: 0.75}]}"

# 3d. a 45-degree left turn
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 3.01, y: 0.7, z: 0.8}, {x: 4.41, y: 0.7, z: 0.75}, {x: 5.4, y: 1.69, z: 0.75}]}"

# 3e. a 60-degree left turn, heading back toward -x
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 5.4, y: 1.69, z: 0.8}, {x: 4.0, y: 1.69, z: 0.75}, {x: 3.3, y: 0.48, z: 0.75}]}"
```

If the robot did not end exactly at a path's last point (say it settled 0.2 m away), that is fine:
the next path's first point only has to be within 0.8 m.

**Pass:** no `hold`, landing tilt ≤ 20°. **Expect** most landings within ~0.1 m, but the fast
hops (the 2.0–2.5 m ones in 3a/3b, and the turns right after a fast hop) may miss by 0.2–0.36 m
with `att_err` of 1.5–2°. Claude's runs: median miss 0.03 m for hops up to 1.5 m, 0.08 m at
1.5–2 m, 0.18 m at 2–2.7 m; turns 0.05 m median, worst 0.36 m.

### Test 4 (optional) — the "too long to fly" guard

World: **flat**, fresh start at (0, 0). Ask for one 2.8 m hop from standing:

```bash
ros2 topic pub --once /ballistic_trajectory visualization_msgs/msg/Marker "{header: {frame_id: 'world'}, ns: 'ballistic', id: 400, type: 7, action: 0, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {g: 1.0, a: 1.0}, points: [{x: 0.0, y: 0.0, z: 0.8}, {x: 2.8, y: 0.0, z: 0.8}]}"
```

**Expect:** `pump, hopping in place to ~1.07`, then two `retry` lines with higher pumps, then
either the hop or `hold ... missed waypoint 1 3 times`. Both endings are correct behaviour; what
matters is that it never tries a hop it cannot fly and never falls over. (The planner never plans
a hop like this; this only checks the safety net.)

### If something goes wrong

| Symptom | Likely cause / what to do |
|---|---|
| No `HopPlan for this path` after a planner goal | hopcopter was built before `hopcopter_msgs`; rebuild the whole workspace (`colcon build`) and re-source. It still flies without it; only the `plan_*` log columns stay empty. |
| `hold ... far from the new path` right after a hand-made path | The path's first point is more than 0.8 m from the robot. Republish starting where it hops. |
| The robot tips over or stops hopping | Note the time and which hop. Run `python calibrate_hops.py` and keep the CSVs (`data/data_ballistic_planner/position_attitude_tracking/attitude_<time>.csv` and `hop_tracking/apex_decisions_<time>.csv`); they are what's needed to debug it. |
| Landings consistently long or short by more than ~0.15 m on slow hops | The model constants may not match your setup. Keep the CSVs; the report's `stance model, measured` lines show the refit values. |
| `calibrate_hops.py` says it needs pandas | You ran it with `/usr/bin/python3`; use the repo `.venv` Python. |

---

## Part 3 — Decisions only you can make

### 1. Fast hops (the biggest remaining error)

On hops faster than ~1.8 m/s, the body sometimes reaches its landing attitude 1.5–2° off. Through
the stance that becomes a takeoff 4–5° off, so the hop misses by 0.2–0.36 m. Cause: during the
fall the motors idle (PWM 1000, right at the simulator's motor cut-off), so they can barely
correct the attitude. The motor model's rotor drag at speed pushes the body back more than they
can correct forward. Pick one:

- **A. Accept it.** The next hop re-aims from wherever it landed, and low_wall passes easily. No
  work.
- **B. Cap hop length at ~2 m** (decision 2). That avoids most fast hops.
- **C. More thrust during the fall**, e.g. PWM ~3000 instead of 1000 in falling state 1. It should
  give the attitude controller real authority, but costs ~0.4 J/kg per hop (about 10% of the
  push budget). The fall's g′ also changes, so the calibration has to be re-measured (stage 0's g′,
  then the range test again). This is a real change to the controller and needs its own testing
  session.

Also worth a look before C: in
`crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/models/crazyflie/model.sdf.jinja`,
motor 4's `rollingMomentCoefficient` is `0.000001` while the other three are `0.0000001` (line
~483). It looks like a typo. If you fix it, the sim's behaviour changes, so re-run Tests 1–3
afterwards.

Claude's suggestion: A for now, plus B if you want landings within ~0.2 m everywhere.

### 2. D6 — hop-length cap

The data is in: landings miss by a median 0.03 m up to 1.5 m, 0.08 m at 1.5–2 m, and 0.18 m at
2–2.7 m. A 2.0 m cap would keep misses within ~0.2 m. There is **no planner parameter for this
yet**; it is a small planner change (limit each hop's radius) plus re-running
`ballistic_plan_check` and updating its expected plans. Decide yes/no and the length.

### 3. Planner constants (optional, low priority)

Stage 4 re-measured how the robot's stance behaves, and the controller uses the new values. The
planner still uses the old ones (η 0.77 and k2 2.469). Both err on the safe side: the planner
assumes slightly less energy and slightly more landing tilt than the robot needs. Updating the
planner would let it plan a little more aggressively. It needs a planner code change and new
expected plans in `ballistic_plan_check`. Only worth it if plans feel too conservative.

### 4. D4 — landing tilt limit

Keep 20°. The earlier worry (landings tilting up to 17% more than commanded) came from a firmware
controller wind-up that stage 4 fixed. Landings now tilt as commanded. Nothing to do; just don't
lower it.

### 5. Before real hardware

hopcopter now sends **zero thrust for one 10 ms tick at the top of every hop**, on purpose: it
makes the Crazyflie firmware reset its attitude controller (see `CLAUDE.md`, "Things that will
bite you"). In the sim the firmware's safety supervisor is off. On a real Crazyflie, check that
the supervisor does not treat these brief zero-thrust ticks as a landing before flying.

---

## Part 4 — What comes after

Once Tests 1–3 pass and you've made the decisions above, the plan's remaining items (in
[follow_planner_plan.md](follow_planner_plan.md), "Deferred") are:

1. **Replanning mid-run.** Today a new goal is planned from `/cf_1/pose`, which lags ~90 ms and
   ignores the robot's speed. Goals sent while it is moving (for example from `nlp_goal_node`)
   need a start at the predicted landing, with the arrival speed.
2. **Free stop at the goal.** The last short hop injects ~2 J/kg only to avoid a cost penalty.
3. **LiDAR map building** instead of the hand-made elevation map (the longer-term goal).

### Picking this up with Claude later

Start a new session and say, for example:
- "Read `docs/next_steps_after_stage4.md`. Here are my test results: ..." (paste the report
  lines, or name the CSV files).
- "Implement the 2 m hop cap from `docs/next_steps_after_stage4.md`, decision 2."
- "Try option C from `docs/next_steps_after_stage4.md`, decision 1, and re-measure."

---

## Your results (fill in)

| Test | Date | Passed? | Miss max [m] | Tilt max [°] | Apex error [m] | Notes |
|---|---|---|---|---|---|---|
| Part 1 (63 tests) | | | | | | |
| 1. low_wall (4,0) and back | | | | | | |
| 2. flat goals | | | | | | |
| 3. hand-made paths | | | | | | |
| 4. pump guard (optional) | | | | | | |

| Decision | Choice |
|---|---|
| 1. Fast hops (A / B / C) | |
| 2. Hop-length cap (yes/no, length) | |
| 3. Update planner constants (yes/no) | |
| 5. Hardware supervisor checked | |
