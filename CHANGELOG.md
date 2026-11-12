# Changelog

## [Unreleased]

### Added
- **Offline checks of the LiDAR map's edges and of plans on it.** `mapper_check` reports phantoms
  and sunken cells (against all the terrain under each cell) at planning time and at the end,
  the error on flat terrain, and the share of points taken for faces; `--shift <m>` moves the
  lattice against the world, `--no-face-filter` / `--no-edge-attribution` turn the new rules off,
  and `--map-out <dir>` writes the map at planning time. `ballistic_plan_check --lidar-maps <dir>`
  plans on those maps as the node would (its map contract and targets, frontier included), with
  the margins before Fix 2, the node's, and a 2 cm steep-noise allowance, and checks every
  landing against the real terrain (fails on one more than 5 cm off, with the node's margins).
- **The elevation map is built from the LiDAR** (docs/lidar_mapping_plan.md, stages 0–5; new
  package `hopcopter_perception`). Gazebo's LiDAR and its IMU go through FAST-LIO to a new mapper,
  which publishes `/elevation_map` for the unchanged planner and hopcopter.
  `elevation_map_publisher` now publishes the scenario's known map on `/elevation_map_known` and
  is the ground truth the LiDAR map is scored against. One launch file
  (`ros2 launch hopcopter_perception lidar_mapping.launch.py scenario:=<world>`) replaces the
  README's Terminals 3–5. Gazebo starts with `-H`, which holds the robot until FAST-LIO is up.
  - **The sensors (D1).** The model's LiDAR is a Livox Mid-360 mounted upside down, 0.4 m above
    the base. It runs at 10 Hz with 512 × 40 rays over −52…+7°, 0.1–40 m and 2 cm range noise. The
    old sensor fired 2048 × 64 rays at 60 Hz, pitched 15° down (2.4.0): ~40× a Mid-360. The blind
    zone at touchdown is now 0.6 m all round, where it was 0.8 m ahead and 3.2 m behind.
    - The LiDAR's own IMU (an ICM-40609-D) is a second `imu` sensor at the LiDAR's pose: 1 kHz,
      with noise from the datasheet's densities scaled so its 200 Hz average has the datasheet's
      noise, plus a small constant bias. `LIDAR_IMU_RATE` and `LIDAR_IMU_NOISE` (read by
      `jinja_gen.py`, which now passes the environment to the template) change them.
    - The base_link IMU is unchanged. RTF with the whole pipeline running: 0.95–1.00.
  - **Hold and release (D3).** `sitl_singleagent.sh -H` spawns a hidden static model whose
    DetachableJoint holds the robot where it spawns. FAST-LIO initializes while the robot hangs
    still, and `lio_anchor` releases it on `/cf_0/release` (bridged). No FAST-LIO patch was needed.
  - **`lidar_sim_adapter`.** It publishes the finite points outside the 0.3 m blind radius on
    `/lidar/points` (the mapper's) and every 3rd of them on `/lio/points` (FAST-LIO's). It averages
    the 1 kHz IMU to 200 Hz on `/lio/imu`, and optionally clips it at the real sensor's range.
    - **Why it averages.** Gazebo samples an IMU instantaneously, a real one through an
      anti-aliasing filter. At 200 Hz the mast-top IMU aliased the stance's rotation: on
      2026-11-07 one stance's x velocity integrated 6.4 m/s wrong (−7.81 against −1.44 m/s), and
      FAST-LIO ran 1.2 m away. With the 5 ms means, the error is 0.04 m/s RMS (max 0.30) over
      122 stances.
    - **Why every 3rd point.** It stands in for FAST-LIO's `point_filter_num` (the shipped Mid-360
      config keeps every 3rd), which this fork's default handler ignores. With all 20 k points a
      scan took up to 51 ms. FAST-LIO's 10-deep IMU queue (50 ms at 200 Hz, on the same thread)
      can then drop a touchdown's samples: in one flat run the height jumped 1.1 m mid-hop. With
      every 3rd point a scan takes ≤ 37 ms, at the same accuracy.
  - **FAST-LIO (D2, D8).** `config/fast_lio_sim.yaml` runs the `~/fastlio_ws` overlay (commit
    18418bc) with:
    - the default handler (`lidar_type` 5), which keeps each point's time 0;
    - `filter_size_surf`/`filter_size_map` 0.1. At 0.2, the height drifted 4 cm over the first
      hops and stayed there.
    - `acc_cov` 3.0. At FAST-LIO's 0.1, the horizontal speed after a fast touchdown lagged for
      ~0.5 s (8–10 cm). At 0.01 it diverged.
    - Replay of a low_wall bag: RMSE 4.0 → 1.5 cm. In the final runs FAST-LIO stayed within
      1.3–3.1 cm RMSE in every world but the maze, where it drifted to 9.5 cm along the 14 m
      corridor. Attitude stayed under 1° except one 0.4 s, 2° transient on lilypads. Clipping the
      IMU at ±32 or ±16 g changed nothing, so Point-LIO was not needed.
  - **`lio_anchor`.** It publishes the static `world → camera_init` from one pose during the hold:
    T(world ← imu, Gazebo) · T(camera_init ← imu, FAST-LIO)⁻¹. It releases the robot and logs
    FAST-LIO against Gazebo to `data/data_lio/` (`scripts/lio_report.py`). `gz_truth_tf` publishes
    Gazebo's pose as TF for the mapper on Gazebo's pose (`pose_source:=gazebo`, stage 3a) and for
    the anchor.
  - **`lidar_elevation_mapper` (D7).** It keeps a Fankhauser-style height per cell: a higher hit
    replaces, a lower one is ignored, the rest fuse. It also keeps an upper bound from the rays
    that pass over each cell, and writes foot contacts as near-certain heights, with a 0.6 m start
    disk at the first contact. It publishes `elevation`, `upper_bound`, `variance` and `hits` at
    2 Hz, and scores itself against the known map (`data/data_elevation_map/`, `map_report.py`,
    `plot_elevation_map.py`). The rules that came out of testing:
    - A height counts once it has 2 hits and three rays have passed above it. A face whose top no
      ray saw (a 3 m pole seen from 1.5 m) stays unbounded: an obstacle.
    - A ray bounds only the cells whose centre it passes within 3 cm of: one that clips a corner
      can miss a pad covering the rest.
    - The bound is the third-lowest ray, and three rays must pass clearly below a height to remove
      it. gpu_lidar returns mixed pixels at silhouettes, at the background's range (one 0.75 m past
      a wall's face, one 27 cm under the floor).
    - A ray that passes through a well-supported height (≥ 10 hits) is dropped from there on.
    - No variance inflation on lower hits (`multi_height_noise` 0). At 1e-4 per hit, a wall's
      thousands of face hits sank its top to the ground.
    - Each cell's sigma has a 1 cm floor. Without it the fused variance collapsed, and a sloped
      cell's spread counted as "a higher surface".
    - Offline check on synthetic scans of all nine worlds (`ros2 run hopcopter_perception
      mapper_check`): median ≤ 4 mm and p95 ≤ 13 mm away from edges, no bound below the terrain
      that a neighbour does not cover. In the sim on FAST-LIO: median 0.4–0.7 cm and p95 1.5–1.8 cm
      everywhere but the maze.
  - **hopcopter publishes** each touchdown's foot on `/hopcopter/foot_contact` and each steering
    apex's decision on `/hopcopter/apex_decision` (new `hopcopter_msgs/ApexDecision`).
- **A `slanted_pads` world** (`elevation_map_publisher` scenario `slanted_pads`,
  `worlds/crazysim_slanted_pads.sdf`; no reference file).
  - **Layout.** The map is 19 × 6 m from (−2, −3). The start platform is the one from lilypads
    (x −1.5 to 1.0, z 2.0, robot at (0, 0), spawn `-z 3.4`). The goal platform is at x 15.1–16.6,
    with the goal at (15.85, 0).
  - **Pads.** Seven 1 × 1 m pads, sloped 12–20°, with every top within 2.0 ± 0.2 m:
    - six in two columns (y 0.3…1.3 and −1.3…−0.3) that face each other, each low on its inner
      edge;
    - a last pad C at y 0, low toward −x.
  - **The stagger forces the zigzag.** Consecutive pads are 1.9 m apart in x, so two pads in one
    column (and start→R1, L3→C, R3→goal) are 2.8 m apart edge to edge, past the ~2.4 m longest
    hop. The momentum-aware plan goes start → L1 → R1 → L2 → R2 → L3 → R3 → C → goal: 10
    waypoints, 3.6 s, hops of 0.76–2.33 m, landing tilts 3.6–19.3° against the 20° cap.
  - **The slopes are not edges.** At most 0.36 m rise per m, against steep_grade's 1.73, so only
    the pad rims are unstandable.
  - **Gazebo build.** Each pad is a 0.05 m box tilted so its top face is the map's plane, on five
    vertical support strips. The map equals that plane at every cell center (checked against the
    planner's snapshot grid to 1e-7 m).
  - **Untested in flight.** hopcopter's stance model was measured on flat ground, so landings on
    these slopes may turn the robot differently than it predicts.
- **hopcopter marches the fall to its touchdown** (`JumpLib/touchdown.py`, `march_touchdown`;
  tested in `test/test_touchdown.py`). The apex prediction used to take the ground under the apex,
  fall to it, look up the ground there and fall once more. Over a gap that first guess is the gap
  floor, so the fall came out ~2 m too long and the pad in between was never looked at. On the
  2026-11-05 lilypads run this put two touchdowns 0.64 and 0.76 m long on ground 0. The push
  target (target apex + ground) then sat ~2 m low and cut at once, the solve saw a 2 m climb
  (`blocked`, hop in place), and the robot fell short of the pad at (7.8, −0.5). Now the fall is
  sampled every 0.2 m of travel (at most 0.1 s apart, 6–7 ground lookups per fall), stopped at the
  first sample where the foot is at or below the ground, and the touchdown is solved in closed
  form for that surface. A foot that meets a surface from its side is bisected to the rim and
  flagged `edge` (warned). The lookup is now under the foot (the CoM minus the leg along the landing
  attitude) rather than the CoM. Replaying that run's apexes: the two errors drop to 0.01 m, and the
  others are unchanged. New `apex_decisions_*.csv` columns `td_kind`, `td_ground`, `td_steps`.
- **A `lilypads` world** (`elevation_map_publisher` scenario `lilypads`,
  `worlds/crazysim_lilypads.sdf`; no reference file). Two 2.0 m platforms 10 m apart, start x
  −1.5 to 1.0 (the robot at (0, 0)) and goal x 11.0 to 12.5 (goal (11.75, 0)), and 20 cylindrical
  pads between them, 1.8–2.2 m tall, r 0.35–0.75 m. The map is 16 × 6 m from (−2, −3); a cell is a
  pad when its center lies inside the pad's circle, and the world's top surface equals the map at
  every cell center. The gaps are ground at 0, not OBSTACLE: the planner rejects any arc over an
  OBSTACLE cell, so obstacle gaps would forbid hopping across them. The robot cannot climb 2 m out
  of a gap, so plans stay on the pads. The goal plans in 3.4 s with 8 waypoints. Each landing is
  on a pad, but 0.15–0.23 m inside its rim: only a pad's rim cells are unstandable, so nothing yet
  keeps landings away from the edges.
- **`sitl_singleagent.sh -z`**: the spawn height, 1.34128 m by default. Lilypads spawns at 3.4 m,
  above its start platform.
- **hopcopter solves each hop's takeoff at the apex** (stage 4 of `docs/follow_planner_plan.md`;
  `JumpLib/takeoff_solver.py`, `HopModel`). This replaces `LinearJumpingController.jumping_planning`,
  which sized each hop from the apex energy alone and aimed 15% short (`velocity_gain` 0.85). From
  the predicted foot touchdown P and arrival velocity v_in, the solve finds the leg launch (heading,
  α_leg) whose landing hits the next waypoint at its target apex. The forward model of the hop:
  - **The leg** launches at √η·|v_in|, in the direction the stance model reaches from the
    landing attitude. η depends on how far the stance turns the arrival (verify step 2): 0.77 up
    to θ_v 30°, rising linearly to 0.81 at 60°. Stage 4 measured 0.765 at 15–35° (48 stances),
    0.787 at 45–55° (24) and 0.816 at 55–65° (12), each to ±0.006. The planner keeps η 0.77, so
    it plans continuing hops with slightly less energy than they carry.
  - **The stance** moves the CoM: the liftoff CoM sits 0.02–0.10 m ahead of the touchdown CoM,
    along the liftoff attitude.
  - **The push** is level and cut off at the apex. Its climb comes from stage 2's push model: a
    finite push climbs slower than a ballistic arc, about 8% more flight time. While the body is
    still tilted after liftoff, it also kicks the robot forward:
    6.69 m/s² × sin(liftoff tilt), over stage 2's leveling profile. That is 0.15–0.25 m/s on a
    first steered hop.
  - **The fall** runs at the g′ of the next landing tilt, 9.55 − 4.1·(1 − cos tilt) (stage 0),
    down to the touchdown CoM height over the ground there. The foot lands off the CoM along the
    next landing attitude, which the controller looks ahead with the same model.

  HopPlan supplies the apex. Its planned heading, α_leg, v_h, arrival and landing attitude are
  logged next to the commanded ones. The guards:
  - **The leg alone overshoots the apex:** the steep no-push root, which keeps the arc above the
    planned apex.
  - **The leg cannot reach the next waypoint above 40° of leg angle:** hop in place first, to the
    apex from which it can (v_h² / (2g′η cos²40°), plus 5 cm, at most 1.5 m). The tracker counts
    that as a retry and holds after two.
  - **Clearance:** every hop re-runs the planner's test (`clearance_floor_alpha`) from where the
    robot actually lands, on the cached map. A hop that fails flies at the least apex that passes
    plus 1 cm, or hops in place if that is more than 0.3 m higher. Planned wall hops sit exactly on
    that apex, so a landing a few cm behind the waypoint already needs a raise: 3–5 cm on
    low_wall.
  - **Tilt:** the braking tilt guard, now `HopModel.limit_tilt`.

  Results (the plan doc's stage 4 results):
  - low_wall (4,0) and back: apex within ±0.03 m, landings within 0.10 m of their waypoints,
    landing tilt ≤ 12.6°.
  - Flat: hops up to 1.5 m land 0.03 m from target (median); doglegs of 30–60° 0.05 m.
  - Fast hops (v_h ≳ 1.8 m/s) still miss by up to 0.2–0.36 m. Their landing attitude needs a
    forward correction that the idle motors cannot make against the motor model's rotor drag.
- **Zero thrust for one tick at each apex.** The firmware resets its attitude PIDs whenever the
  thrust setpoint is 0. A short push after a continuing landing ends while the body is still
  leveling. The idle motors cannot finish the job, so the body coasts ~3–4° past level, and the
  rate and attitude integrators wind up against it. At the apex they drove the fall 1–1.5° past
  its landing attitude, and the stance turns that into a takeoff 2.5–4° too steep: landings
  0.1–0.3 m short. With the reset, the median touchdown attitude error is 0.02–0.04°. Over the
  reset and the refits below together, the dogleg runs' median miss fell from 0.084 m to
  0.04–0.05 m.
- **Stage 4 report in `calibrate_hops.py`**. It shows each steered hop against the solve and
  the plan:
  - the apex landing prediction;
  - the touchdown attitude against the command;
  - the takeoff angle, η, push kick, stance shift and flight time against the model;
  - the next landing against its target and against the model.

  It also reports landing error against hop length (D6), and refits k1, k2 (by arrival) and η (by
  redirection angle).
- **`test_takeoff_solver.py`** (30 tests). Every solved hop lands on its target, in the model and
  in an independent step integration of the same physics (within 1 cm). It also covers the
  stance model against fcn1, the climb, the steep root, the pump, the in-place hop, the tilt guard
  against stage 1's, and the clearance apex over low_wall.
- **Momentum-aware planner mode** (stage 3 of `docs/follow_planner_plan.md`;
  `search.momentum_aware`, on by default). The reference A* plans hops the robot cannot fly: it
  pushes along the takeoff angle, lets the leg alone sustain the start's apex, never checks how
  far a landing must tilt to redirect the robot into its next hop, and flies every hop at its
  least-injection angle. With the flag:
  - **Vertical push.** The robot pushes level after liftoff, so the leg alone supplies the
    horizontal speed. The arc, apex and injection are unchanged; the takeoff friction cone now
    applies to the leg angle α_leg = acos(v_h / v_leg), one closed-form bound on tan α. Hops
    keep α_leg ≥ 42° (`stance.min_leg_angle_deg`), so the controller's 40° guard has room for
    speed binning (±1.5–2°).
  - **Push capacity.** A hop may not inject more than one 30000 push delivers within 0.4 s from
    the leg's vertical speed: stage 2's fit, `push.accel` 7.55 m/s², `push.g_coast` 9.55,
    `push.time_offset` −1 ms, `push.max_time` 0.4 s. It is checked at the bottom of the
    arrival's speed bin; checked at the bin's centre, a hop at 99% of capacity failed with real
    speeds.
  - **Landing tilt.** Every landing must redirect the robot into its next hop within 20°
    (`stance.max_tilt_deg`, D4) under hopcopter's stance model (fcn1 with `stance.k2` 2.469; k1
    sets only the liftoff attitude, which the planner does not use). The start counts as a
    vertical arrival. The goal must allow a stop: a descent angle of at least 56.4°. Each A*
    state keeps its cheapest path's arrival direction; the key stays (cell, speed bin). The
    returned path is re-checked with its exact arrival directions, and a failure is reported as
    no path.
  - **Banking.** Besides the least-injection angle, each hop also tries 4 steeper angles
    (`search.bank_bins`), each landing one speed bin faster. This departs from the reference
    planner's rule: spending a little more on one hop can carry the energy a later hop needs,
    which the reference burned with sideways hops (the 93° detour).
  - **Seed.** The start's arrival speed is the landing from the warm-up apex, √(2g·h_initial) =
    3.96 m/s, binned to 4.0. It was √(2g·h_initial/η) = 4.5 m/s, which assumed the leg alone
    sustains that apex; in fact the push does (40–75 ms at 30000).
  - η stays one constant (0.77) until stage 4 measures whether it depends on the redirection.

  Off, the search is the reference port, unchanged: `ballistic_parity_test` output is
  byte-identical.
- **`hopcopter_msgs` package, `HopPlan` on `/ballistic_hop_plan`** (D5). Per hop: the landing
  (xy and ground z), heading, α_leg, v_leg, v_h, target apex, injection per kg, the expected
  arrival velocity, and the landing axis and tilt that redirect the robot into the next hop (or
  stop it at the goal). Per plan: the arrival speed and stance assumed at the start, and the
  model constants (g, η, k2, `momentum_aware`). It is published just before the marker pair,
  and all three share one stamp. The id=400 marker is unchanged. hopcopter matches it to the
  marker by stamp and logs every commanded hop against it (stage 4).
- **`ballistic_plan_check`** (offline, no ROS graph):
  `ros2 run ballistic_motion_planner ballistic_plan_check`. It plans 15 cases with the node's
  defaults: the plan doc's three expected plans, the rest of the 13-case matrix, and stairs on
  the reference configuration. For each it checks:
  - every landing's tilt, the stop included;
  - the expected plans;
  - with chained, unbinned speeds for η 0.71–0.85, the controller's 40° leg-angle guard and the
    push capacity.

  It also reports search effort against the reference search, and exits 1 if a check fails.
- **Planner output**: `ballistic_hops_*.csv` gains `heading_deg`, `alpha_leg_deg`, `v_leg`,
  `v_h`, `e_inject_per_kg`, `launch_tilt_deg` and `land_tilt_deg` after its existing columns.
  The logged hop table gains the leg angle, J/kg and tilt, plus the stop's tilt. Snapshots carry
  the same per hop, and the PNG's side views show the leg angle and landing tilt.
- **Stage 2 reports in `calibrate_hops.py`**: the lead each push cutoff implies (and the
  `t_lead` it calls for), apex error per target, 3 → 2 transitions during a push, injection
  available within the 0.4 s cap against liftoff vertical speed (fitted to every push), and the
  horizontal speed a push adds after a tilted liftoff.
- **`test_push_controller.py`** (16 tests): arming, the cutoff, the cap, and a closed-loop climb
  with the Gazebo motor model, whose apex lands within ±1.2 cm of target with the lead and
  4–8 cm high without it.
- **Stage 0 instrumentation (`docs/follow_planner_plan.md`)**: the per-tick
  `position_attitude_tracking/attitude_*.csv` gains, after its existing columns: the odom stamp
  (sim time) and pose, the velocity differentiated from consecutive odom poses on their stamps
  (`diff_v*`) next to the odom twist (`odom_v*`), the IMU stamp and z specific force, the thrust
  command and a push flag, the four motor PWMs with the firmware tick, and `/cf_1/pose` position
  and attitude with its stamp (diagnosis item 4). The differentiated velocity is there because
  the twist is a 50 ms window mean whose frame was unverified. Existing columns are unchanged, so
  `visualize_rpy.py` still works.
- **Per-apex decision log**: `hop_tracking/apex_decisions_*.csv`, one row per 3 → 1 transition:
  path id (marker stamp), target index and waypoint, apex state, fall height and time, LSE's
  predicted landing, LJC's takeoff direction and an `unreachable` flag (its 45° fallback), the
  commanded landing attitude and tilt, fcn1's redirection angle, JHC's energy estimate and the
  push time. `LinearJumpingController` now keeps `norm_d` / `norm_d_max` for the flag.
- **`desired_rel_z` is a ROS parameter** (default 0.8 m), so stage 2's in-place apex sweep can
  run in one session: `ros2 param set /hopcopter desired_rel_z 1.0`. With no active waypoint it
  also updates `desired_z`, which drives in-place hops after warm-up.
- **`calibrate_hops.py`** (repo root): reads the newest per-tick CSV and reports the real-time
  factor, g′, η per stance (with landing speed, tilt and redirection angle), the odom twist
  frame, push thrust against the motor model, push-cutoff latency, and `/cf_1/pose` error.
- **Planner-commanded hop height (`target_apex`)**: the `/ballistic_trajectory` marker's point z
  is now each waypoint's `target_apex` = `apex_drop + Z` of the hop that lands on it (planned
  apex height above that hop's takeoff ground), and `output.hover_offset` (0.8 m) for the start
  point. It replaces `terrain + 0.8`, which hopcopter ignored. `hopcopter` sets `desired_z` from
  it in `_load_next_waypoint` and passes it to `JHC.step()` at each apex, so the powered-climb
  duration now follows the plan instead of the constant `desired_rel_z`. The 0.8 m
  `desired_rel_z` remains the fallback before a path arrives, during warm-up hops, and after the
  last waypoint. The per-tick `desired_z = ground + 0.8` lookup was removed; it only fed the
  unused flight-mode PID. The planner search is unchanged, so flat hops can command 0.70 m
  (`min_apex`).
- **`target_apex` CSV column**: added after `previous_apex_h` in `planned_hops_*.csv` and in
  hopcopter's `hop_tracking/hopcopter_*.csv` (there it is the target that sized that arc's
  push; blank for startup arcs), and after `apex_drop` in `ballistic_hops_*.csv`.
  `ballistic_path_*.csv` z now matches the published marker z.
- **Five more test worlds**, mirroring `hopcopter-ballistic-planning/maps/`: `elevation_map_publisher`
  scenarios `stairs`, `stairs_with_curb`, `slope_30deg`, `cliff_gap` and `maze`, each with a
  Gazebo twin `worlds/crazysim_<scenario>.sdf`. Each map is placed so its reference `START` is
  world (0, 0), where the robot spawns (origin = −START), and sets its own default size (the
  cliff gap and maze are 15 × 5 m). The README's "Other worlds" table lists the commands and goals.
  Each map's published cells equal its world's top surface at every cell center, and each
  reference goal plans with the real nodes. With the robot's planner (`robot_params()`), three
  reference maps have no plan the controller can fly, so they were edited until they had one:
  - **stairs:** no path with 0.6 m risers on 0.3 m treads (nor 0.5 or 0.4 m risers on 0.3–0.5 m
    treads, nor 0.6 m on 0.6 m). Now four 0.4 m risers on 0.6 m treads from x 2.0 → 1.5
    (reference frame), so the top is 1.6 m instead of 3.0 m. 10 waypoints, 4.8 s.
  - **stairs_with_curb:** no path at any curb rise from 0.2 to 0.6 m on 0.4 m treads. A curb
    stands above the tread on both sides of it, and the inflated field (0.33 m) puts it under the
    end of every arc landing on a tread. Now 0.15 m curbs on 1.0 m treads from x 1.2. The planner
    lands on the curbs themselves, which at 0.15 m are not steep edges. Curbs of 0.2 m or more
    still fail, except 0.2 m curbs with 0.3 m (not 0.4 m) between treads, which needs a 12-hop
    plan that pumps speed in place. 10 waypoints, 3.5 s.
  - **cliff_gap:** the 1.5 m gap plans, but the 1.9 m hop across it falls below the controller's
    40° leg guard at η 0.71 (39.5°). Now a 1.0 m gap (x 6.5–7.5); a 1.2 m gap gives the same
    failing plan. 14 waypoints, 21 s. Its descending 0.6 m steps are unchanged; a goal on the far
    ground plans around them through the side margin.
  - **slope_30deg** (10 waypoints, 7.2 s) and **maze** (10 waypoints, 20 s, around the wall) are
    unchanged, except that a maze pole is the 2 × 2 cells inside its 0.2 m footprint
    (`set_obstacle_region` rounds out to 3–4 cells). Maze poles are 3 m tall in Gazebo. (The maze
    was later redesigned; see "Changed".)

### Changed
- **On a LiDAR map a step counts as an edge from 0.153 m per cell** (`standable.lidar_steep_noise`
  0 → 0.02, in `ballistic_planner_node`, `ballistic_plan_check` and hopcopter's clearance re-check,
  `lidar_steep_noise`). The 60° rule makes an edge of a jump over 0.173 m per cell, and the stairs'
  0.18 m risers sat 7 mm over it. The LiDAR map measures them right (median 0.180 m on the
  2026-11-11 stairs run) but each riser's height spreads ±10 mm, so 4 in 10 of the near risers fell
  under the line, read as a slope, and their cells became landable: the planner landed on the
  first 10 cm of each tread, at the riser's edge. With 0.02 the near risers that were mere
  near-misses (22 of 136 in that run) are edges again; offline (a replay of the run's trajectory) the
  near risers caught go from 84% (robot at the bottom) and 64% (after climbing) to 95% and 99%. The slope stays a slope: 0.058 m per cell, 0.107 m at most
  on that run's LiDAR map, no false edges at 0.02 on the stairs or the slope. On
  `ballistic_plan_check --lidar-maps` the stairs plan now lands mid-tread (1.15, 1.45, 1.75,
  2.05) instead of at each riser (1.05, 1.35, 1.65, 1.95). Not fixed: risers the mapper reads 4–19
  cm short because stray face points take over a cell (31 of the 136, and every far riser seen
  from below); the stairs' LiDAR-map case still fails on one such cell, as it did at 0. The
  "2 cm noise" comparison in `ballistic_plan_check` is gone (now the node's defaults).
- **The mapper no longer lets a vertical face set a floor's height** (docs/lidar_map_quality_plan.md,
  Fix 2). A face (a riser, a pole's side, a wall) usually sits on a cell border, and the 2 cm range
  noise scattered its points into the floor cell in front, which "a higher hit replaces" handed
  the face's height: every face the LiDAR saw was a cell (10 cm) fatter toward it. On 2026-11-10
  that put every stair edge a cell early, and a 0.94 m phantom beside a maze pole blocked the
  robot's hop. Three rules (`ElevationMapper`, `face.*` and `edge.*` parameters):
  - **Face points.** A point that forms, with another point of the same scan, a line steeper
    than 60° (the planner's own limit) and 5–50 cm tall, within 6 cm in xy, is on a face. Where a
    cell has ground hits, face points are ignored there. A cell only faces reached (a pole's
    inside, a tread above the sensor) works as before, but is published unseen (its bound, or an
    obstacle), never a landing. On flat ground 0.5–1.6% of the points are taken for faces.
    A 10 cm search radius flagged tread points near each edge and doubled the phantoms; flagging
    only the lower point of a pair tripled them.
  - **Edge attribution.** Noise also moves a top surface's points past its edge along the ray (a
    step's edge seen from above lands on the tread below). A hit that would raise a cell with
    at least 5 hits is credited instead to a neighbour within 3 σ of the ray's horizontal noise,
    along the ray, whose estimate is already as high. Without the 5-hit floor a cell that started
    low could never be raised; with it, sparsely seen cells stay exposed.
  - Such a credited hit no longer resets a run of lower hits that is correcting the cell.

  `mapper_check` (phantoms: cells more than 5 cm above all the terrain under them, at planning
  time / at the end; old mapper → new): stairs 249/308 → 75/49, low_wall 87/88 → 40/2, maze
  8/71 → 0/0, slope_30deg 23/76 → 14/39, stairs_with_curb 207/284 → 89/29. With the lattice
  shifted 3 cm (faces inside cells, as in the real world) stairs, stairs_with_curb, maze, slope
  and cliff_gap have none. What remains is mostly a rim seen only from above (cliff_gap, lilypads,
  slanted_pads, the stairs' far side): no ray sees past it, so the cell beyond reads the rim's
  height. The worlds put every face on a cell border, the worst case. The mapper takes 11 ms a
  scan in `mapper_check` (5.5 before).
- **The planner's LiDAR-map margins** (`ballistic_planner_node`): `standable.lidar_steep_noise`
  0.08 → 0, so an edge is again a jump steeper than 60° (it made 9.3 cm per cell an edge, and
  noise put false edges on the 30° slope). `standable.lidar_edge_margin` (0.1) now applies only to
  edges dropping at least `standable.lidar_edge_min_drop` (0.3 m): a rim seen from above still
  reads a cell wide, and the robot fell at the maze's wall and cliff_gap's rim, but stair risers
  (0.18 m) keep their treads' middle cells landable, as on the known map. Planned on the maps
  `mapper_check` builds at planning time, stairs now climbs the stairs (7 hops, frontier on
  tread 6) instead of hopping around them on the floor (3 hops), which is what the 2026-11-10 run
  did. The 0.18 m risers are 7 mm over the 60° threshold (0.173 m per cell): one mapped 1 cm low
  is no edge, and three of the stairs' landings then land on a tread's first cell, 5 cm past a
  riser; `lidar_steep_noise` 0.02 puts them all on middle cells. Open: on the aligned lattice the
  frontier cell on tread 6 reads 10 cm high (seen at 10° from the start). `ballistic_parity_test`
  is byte-identical; the 15 cases and the partial-map cases pass.
- **The planner reads a LiDAR map's contract** (docs/lidar_mapping_plan.md, "The map contract").
  A map with an `upper_bound` layer is a LiDAR map:
  - an unseen cell with a bound is terrain at the bound for the clearance test and never a
    landing (`Map2D5::set_landable`, which `standable_mask` ANDs in);
  - an unseen, unbounded cell is OBSTACLE, as NaN always was;
  - a hand-made map, without the layer, keeps every finite cell landable.
  `ballistic_parity_test` is byte-identical, and the original 15 `ballistic_plan_check` cases are
  identical; 5 new partial-map cases pass.
  - **Landing margins on LiDAR maps.** A landing keeps 0.3 m (`standable.unseen_margin`) from
    unseen cells, the body plus a typical miss. A 0.2 m wall whose shadow was bounded at the
    wall's height read as a platform, and the robot came down on its front edge. Landings also keep
    0.1 m further from steep edges (`standable.lidar_edge_margin`), since the mapper puts an edge
    up to a cell outward. And a step counts as an edge 8 cm sooner (`standable.lidar_steep_noise`:
    0.093 m per cell, ~43°), because the mapper spreads a step over two cells. The stairs' 0.18 m
    risers read as 0.09 + 0.09, and a plan landed 5 cm past one. The costs:
    - stairs (0.3 m treads) has no landable tread on the LiDAR map;
    - the edge margin still leaves 0.05–0.15 m to a real edge. One maze run landed on the 0.3 m
      wall and one cliff_gap run 0.15 m from the 1.8 m rim, and the robot fell at both. A margin
      of 0.2 m prevents that but breaks lilypads' pit-unseen plan, so it is left open
      (docs/lidar_mapping_plan.md, Status).
  - **The start guard.** The planner refuses to plan from an unmeasured start. It used to plan
    every hop from z = −1.0.
  - **Frontier plans.** A goal not yet on measured, standable ground gets a plan to the reachable
    cell nearest it (`frontier_candidates`: a 1.5 m-step flood over standable cells, ≥ 0.35 m from
    unknown cells, nearest first, two tries).
  - **Hold-and-replan.** On every map update the remaining hops are re-checked: landing still
    standable, ground moved < 5 cm, every arc clearing by at least 0.15 m with its planned takeoff.
    A replan is also triggered when a frontier plan can do better (the goal is now measured, or the
    robot reached the frontier) or the robot holds. The planner then sends a DELETE alone, waits
    for the robot to hop in place, and plans from there, at most every 5 s (backing off up to 30 s
    after failures). Logged to `data/data_ballistic_planner/replans_*.csv`.
  - **`clearance.active_tolerance` 0.05** (reference 0). Terrain constrains an arc only where it
    is more than this above both endpoints. A LiDAR map's flat ground is a few mm rough, and at 0 a
    3 mm bump next to a takeoff asked the full bottom clearance there: a 1.95 m apex for a 1.2 m
    flat hop.
  - **`clearance.start_exempt` 0.25 m** (reference 0). From a takeoff inside the clearance
    inflation, the arc's first 0.25 m read the terrain itself. In the maze the robot landed 0.21 m
    from a pole and no hop could leave. Takeoffs outside the inflation are unaffected.
  - Snapshots carry the measured mask, and `visualize_planned_path.py` shades unseen cells and
    marks frontier goals.
- **hopcopter on a LiDAR map.**
  - Its clearance re-check uses the planner's composed field (`composed_terrain`), with the same
    tolerance and start exemption.
  - The touchdown march falls through unseen cells to the floor. A takeoff ground on an unseen
    cell is the last foot contact's, and `current_ground_z` is that contact (it was 0.0 forever).
  - `elevation_map_callback` takes 0.6–0.9 ms per map on the control thread (logged every 20
    maps).
- `elevation_map_publisher`'s scenario maps moved to a header (`include/elevation_map_publisher/
  scenarios.hpp`, shared with `ballistic_plan_check` and `mapper_check`), and it takes a `topic`
  parameter.
- **Bottom and side clearance are separate** (`ballistic_motion_planner` `robot_params()`,
  `hopcopter`'s clearance re-check). One `min_clearance` (0.3 m) used to be both the gap the arc
  keeps above the terrain and, with `robot_radius` (0.03 m), the sideways inflation of steep
  edges (0.33 m). On the 0.3 m stair treads that inflation covered each whole tread, so every hop
  had to clear the next step's height plus 0.3 m right up to its landing, and the plans had 0–3 cm
  to spare. On 2026-11-06 a usual 9 cm short landing then needed a 0.334 m raise at the first riser,
  over `apex_raise_max`, and the robot stopped at the stairs. Now `bottom_clearance` 0.2 m is the
  vertical gap, and edges are inflated by `robot_radius` 0.15 m (the body to the propeller tips)
  + `side_clearance` 0.03 m = 0.18 m; obstacle (NaN) cells are dilated by the same 0.18 m for
  standability. ROS parameters `robot.min_clearance` → `robot.bottom_clearance` and
  `robot.side_clearance`; snapshots carry both (the visualizer still reads old ones).
  `PlannerParams`' defaults, `ballistic_parity_test` (byte-identical) and the reference case of
  `ballistic_plan_check` keep one gate for both. The stairs (4, −2) now plan 10 waypoints, one
  hop per step, with 0.03–0.16 m of apex to spare (0.07–0.16 on the risers) and a raise of at most
  0.26 m from 10 cm short.
  low_wall (4,0) and (4,1.5) re-plan, and `ballistic_plan_check` expects the new paths.
- **hopcopter's `apex_raise_max` 0.3 → 0.5 m**, more room for the clearance re-check to raise a
  hop after a short landing (the 2026-11-06 stairs run needed 0.334 m).
- **The stairs are human stairs** (`elevation_map_publisher` `stairs`, `worlds/crazysim_stairs.sdf`).
  The four 0.4 m risers on 0.6 m treads (top 1.6 m) were each the robot's full 0.37 m standing
  height, out of scale with the stairs a robot this size would meet. Now eight 0.18 m risers on
  0.3 m treads (a standard 7 in / 12 in step, about half the robot's height), from world x 1.0;
  the top is 1.44 m from x 3.1 to the map edge, still y −3.5 to −0.5. 0.2 m risers on the same
  treads have no plan. The goal (4.0, −2.0) plans in 3.9 s with 16 waypoints: one step per two
  hops (a 0.3 m hop up a riser, then a 0.2 m hop sideways along the tread), and a last 1.0 m hop
  up three steps. It keeps the 40° leg guard (≥ 72°) for η 0.71–0.85, with pushes ≤ 53% of
  capacity and landing tilts ≤ 8.9°.
- **The maze makes the robot hop among its poles and over its wall instead of going around**
  (`maps/maze.py`, `elevation_map_publisher` `maze`, `worlds/crazysim_maze.sdf`). The old plan
  (10 waypoints) went along the open strip below the wall at world y ≈ −2, where it hardly met a
  pole. The 17 poles are unchanged; in the reference frame:
  - **A corridor:** infinitely tall side walls, 0.2 m thick, at y 0.8–1.0 and 4.0–4.2 along the
    whole map, so the robot stays in y 1.0–4.0 with the poles. In Gazebo they are 2 m tall and
    see-through. Three poles ((4.0, 4.2), (4.5, 0.8), (8.7, 4.2)) now touch a side wall.
  - **The wall spans the full map width** (it used to stop 1 m short of each side), still 0.3 m
    thick at x 7.35–7.65.
  The goal (14.0, 0.0) plans in 6.6 s with 8 waypoints through the poles, and one 2.31 m hop over
  the wall (apex 1.05 m, 3.77 J/kg of the 4 J/kg cap). The wall's middle cell is not a steep edge,
  so the planner may land on top of it; it did on a gate layout tried along the way, not on this one.
- **hopcopter's clearance re-check spreads only terrain edges, as the planner does**
  (`takeoff_solver.dilate`, new `steep_mask`). It used to take the highest cell within 0.33 m of
  every cell, which equals the planner's field on flat-topped obstacles but lifts a slope by its
  rise over that distance. On `slope_30deg` the first hop (planned apex 0.84 m) then needed 5.8 m,
  so every hop onto the slope came out `blocked` and the robot held at the start. Now only cells
  steeper than 60° to a neighbour (and obstacles) spread, never below a cell's own ground; that
  hop needs 0.55 m. low_wall, flat, stairs and maze give the same field as before; on
  `stairs_with_curb` the 0.15 m curbs (not steep against their treads) no longer spread over the
  treads, as in the planner.
- **hopcopter's stance model is refit** (stage 4, verify step 2; `HopModel`):
  - k1 1.688 → 1.50.
  - k2 2.469 → 2.53, rising to 2.70 for arrivals descending more steeply than 75–85° (a linear
    ramp). Moving arrivals measured 2.46–2.55 by descent angle, higher on straight continuation
    and lower on doglegs; 2.53 landed with the least bias.

  The fit is over 48 steered stances with the zero-thrust reset (and 58 before it). With fcn1's
  constants:
  - First steered hops launched ~2° flatter than commanded (k2 2.70) and landed 0.03–0.11 m long.
  - On continuing hops, k1 overpredicted the stance shift by 35% and the push kick 2–3×.

  The planner keeps 2.469 for its 20° tilt cap, which errs on the safe side: the refit k2 needs
  less tilt for the same turn.
- **Touchdown prediction at the apex**:
  - the odom twist rotated into the world frame (it is body frame);
  - the vertical speed already reached when the apex is detected;
  - the ground under the touchdown from the map;
  - the touchdown CoM height L·cos(tilt);
  - the g′ of the landing tilt.

  The foot touchdown is now predicted to ~1 mm median (2 cm max) in the sim, against 2.1 cm in
  stage 1.
- **The push aims at the apex above the takeoff ground** (the map's elevation under the takeoff
  foot), not above flat ground.
- **Apex-decision CSV columns** (stage 4):
  - `unreachable` becomes `solve` (`push`, `leg`, `capped`, `pump`, `blocked`), and
    `planned_tilt_deg` becomes `tilt_needed_deg`.
  - New: the predicted arrival `v_in_*`; the solved `alpha_leg_deg`, `v_leg`, `v_h_leg` and
    `apex_raise`; the commanded landing axis `cmd_axis_*`.
  - New `pred_*` columns, the model's expectations: push, climb and fall times, kick distance and
    speed, stance shift, liftoff tilt, next foot touchdown.
  - New `plan_*` columns from HopPlan: this landing's waypoint, arrival, landing axis and tilt, and
    the next hop's heading, α_leg, v_h and apex.
  - `takeoff_heading_deg` and `takeoff_elev_deg` are now the commanded leg launch, and
    `theta_v_deg` the solve's.
- **The planner node's defaults are the real robot** (stage 3; this and the next two entries).
  `search.momentum_aware` true, and:
  - `physics.mass` 0.8 → 0.3609 kg (hopcopter's `hop_mass`);
  - `physics.eta` 0.7 → 0.77 (stage 0, in place);
  - `physics.e_inject_max` 7.85 J (9.8 J/kg) → 4 J/kg × mass = 1.44 J. One push at 30000
    delivers 3–7 J/kg (stage 2), and the plans need ~4;
  - `robot.min_clearance` 0.5 → 0.3 m (D3): low_wall plans become three straight hops within
    the budget;
  - `search.w_energy` 0.84 → 1.862 m/J, i.e. 0.84 × 0.8/0.3609, which keeps the reference
    trade-off per kg.

  The defaults live in `ballistic::robot_params()`; `PlannerParams` keeps the reference
  defaults, which `ballistic_parity_test` uses. low_wall (4,0) now plans (0.05,0.05) →
  (1.65,0.05) → (3.55,0.05) → (4.05,0.05) at 3.4 / 3.8 / 2.0 J/kg with landing tilts ≤ 14.3°,
  as the plan doc predicted. The reference search planned (4,0) with a 29.7° WP1 landing at the
  old defaults. Plans take 4–7 s, against 1–4 s for the reference search on the same
  parameters.
- **`validate_params`' first-hop check is now "some first hop exists"**: it expands the start
  once, on flat ground, with the search's own edge test. It used to check only the 45° hop at
  the start's full radius, which threw at node startup for any budget under 5.9 J/kg, and with
  the new seed, although shorter hops existed. `search.bank_bins` must be 0–8.
- **`clearance_floor_alpha` walks the arc in one pass** instead of building `arc_samples`'
  vectors: the same samples and arithmetic, without three allocations per candidate edge (about
  5% faster). `ballistic_parity_test` output is byte-identical.
- **The push is cut off at the target apex, closed-loop** (stage 2 of
  `docs/follow_planner_plan.md`; this and the next five entries). JHC guessed a push duration at
  the apex from a one-hop-old energy estimate. It compared a vertical target with a total-energy
  estimate, assumed 0.7 g of thrust where the real value was 0.48 g, and its first estimate hit
  an epoch bug. `PushController` (`JumpLib/push_controller.py`) instead cuts the push on the
  tick where
  `(z − z_lo) + v²/(2g′) + v·(a_T·cos tilt/g′)·t_lead ≥ H`:
  - z_lo is the liftoff CoM height, `leg_length` 0.366;
  - v is the vertical speed from the differentiated odom pose, which is world frame;
  - g′ is 9.55;
  - t_lead is 0.026 s: the 21 ms of thrust that keeps arriving after the cut (spin-down,
    latency, odom age), measured over 74 pushes, plus half a 10 ms tick.

  In place at 0.6–2.0 m, every apex is within −1.8 to +3.0 cm of target (median −0.1 cm,
  n = 92). The odom twist is body frame and reads up to 1 m/s low on the first tick after
  liftoff, so it is not used.
- **Push thrust 21000 → 30000** (D1): 0.81 g, below hover (34265), so a push still ends by
  the apex on its own. 21000 (0.48 g) could not reach the planned apexes. Measured within the
  0.4 s cap: about 3.1 J/kg at a liftoff vertical speed of 1.5 m/s and 7.4 J/kg at 3 m/s.
- **Push length**: capped at 0.4 s (was `t_p_high` 1.5 s), with no minimum (was `t_p_low`
  0.04 s, which alone adds ~11 cm of apex at 30000). JSTO's initial `powered_climbing_end_timer`
  is 0.2 → 0 s; the timer no longer drives the push.
- **A push is armed once per real stance**: the IMU must read above 5 g for 10 ms, and the
  robot must climb faster than 1 m/s. The push is never armed in flight (state 1). JSTO re-armed
  on any acc_z crossing, and spurious 3 → 2 → 3 transitions were logged at 09:44 and 10:30. A
  stance tick in mid-climb now only pauses the push.
- **Apex-decision CSV**: `jhc_nominal_h` and `push_time` are replaced by the record of the
  climb that just ended: `climb_target`, `climb_apex` (both above the liftoff CoM height),
  `push_end` (`target`, `leg`, `cap`, `apex`, `stance` or `unarmed`), `push_s`, and the state at
  the tick that ended the push: `cut_h`, `cut_vz`, `cut_cos_tilt`.
- **SITL IMU fixes (problem 2, the motor cut-out)**. The firmware log showed the supervisor
  going armed → flying → tumbled + crashed at a stance, which locks the motors for good. Two sim
  artifacts caused it, and both are fixed:
  - `sensors_sitl.c` computed an accelerometer scale from the 1000 samples after the gyro bias
    settles, about 1.1 s after Play. A dropped robot is hopping then, so the scale came out
    around 0.54. Idle coasts then read 0.104 g, above the tumble check's 0.1 g free-fall limit,
    so its clock ran through every flight. The scale now stays at 1: Gazebo's accelerometer has
    no scale error.
  - `crazysim_plugin.cpp` cast IMU values to int16 without a range check, so 20–30 g stance
    readings wrapped to negative values: "upside down". Counts now saturate at ±16 g and
    ±2000 °/s like a real sensor.

  With the supervisor enabled, a run with in-place and steered hops up to 33° of tilt stayed
  armed and flying throughout. The scale fix also corrects the firmware's own state estimate,
  which feeds `/cf_1/pose`. `cf2` and the plugin were rebuilt from current source, which also
  compiles `stabilizer.c`'s 2025-10-15 edit: the supervisor calls are commented out in SITL. The
  previous `cf2` binary predated that edit.
- **Waypoints advance at the apex, not by distance** (stage 1 of `docs/follow_planner_plan.md`;
  this and the next six entries). The planner plans exactly one hop per
  waypoint, and LJC/JHC commit to the next landing and push at the apex. The old 0.4 m distance
  pop fired around touchdown, so every steered landing aimed at the waypoint the robot was
  already landing on (the WP1 brake). Diagnosis showed no single tolerance works.

  `WaypointTracker` (`JumpLib/waypoint_tracker.py`) now decides at each steering apex, at most
  once per real touchdown, from the predicted foot landing point P:
  - within `reach` 0.8 m (R_far) of the waypoint the flight was aimed at: aim at the next one,
    with its target apex;
  - missed it: retry, and after 2 retries hold in place;
  - on the last waypoint: hop in place at `desired_rel_z`.

  A new path syncs to the waypoint nearest P; if none is within R_far, the robot holds. Waypoints
  the robot overshot are caught up; a close one is skipped only if P is past it.
- **Foot landing point.** P is LSE's CoM prediction minus the leg along the planned landing
  attitude, because waypoints are foot positions. In the stage 1 test it was within 0.02 m
  (median) of the actual landing, against 0.10 m for the CoM prediction.
- **A touchdown sooner than 0.15 s after the apex does not count** toward the next decision
  (false trigger). 3 → 2 counts as a touchdown whose apex was missed.
- **DELETE no longer clears the path.** The planner sends DELETE right before every ADD, so the
  robot keeps flying the current path. It drops the path only if no ADD follows within 0.5 s.
- **Touchdown height and fall-time gravity.** `leg_length` was the filtered z at the first
  touchdown, 0.420 m. The touchdown CoM height is 0.366 m (stage 0 measured 0.367). Fall times
  now use g′ 9.55 (stage 0, level falls) instead of 9.8. Together these made LSE predict
  steered landings ~0.1 m short and made the predicted landing too shallow (braking commands
  ~1.6° too steep).
- **Braking tilt guard** (D4, τ_max 20°): `LinearJumpingController.limit_landing_tilt` rotates a
  takeoff whose landing would need more than 20° of tilt toward continuing the arrival (the fall
  mirrored about the vertical), so a brake is spread over two hops. Today's controller tracks
  95–117% of the commanded tilt, so a capped landing can still reach ~23°.
- **Apex-decision CSV columns**: `decided`, `mode`, `land_idx` (the waypoint this flight was
  aimed at), `target_idx`, `retries`, `foot_land_x/y`, `planned_tilt_deg`, `tilt_capped`.
  These replace `wp_remaining` and the nearest-match `target_idx`.
- **`ballistic_motion_planner` builds Release by default**: `CMakeLists.txt` sets
  `CMAKE_BUILD_TYPE=Release` when none is given; it was empty, i.e. unoptimized. The A* ran
  about 3× slower that way (one low_wall goal: 10.6 s vs 3.1 s). `ballistic_parity_test` output
  is byte-identical (0.60 s → 0.17 s). An explicit `-DCMAKE_BUILD_TYPE` still wins.
- **`low_wall` moved +1.3 m in x**: the wall now sits at world x ∈ [2.5, 2.7] (was [1.2, 1.4]),
  giving the robot more room before the wall. Changed in both `elevation_map_publisher`
  (`wall.x_min` / `wall.x_max` defaults) and the Gazebo world `crazysim_low_wall.sdf` (pose x
  1.3 → 2.6). Height, thickness and y span are unchanged. The wall no longer matches the Python
  reference `maps/low_wall.py`; `parity_main.cpp` still uses the reference geometry.
- **`ballistic_motion_planner` min clearance**: raised `robot.min_clearance`
  (`min_clearance_gate`) from 0.2 m to 0.5 m in both the ROS parameter default and the
  `PlannerParams` struct default. It is both the vertical gap the foot's arc must keep above
  terrain and part of the horizontal inflation (`robot_radius + gate` = 0.53 m) used for obstacle
  widening and the standability mask. `parity_main.cpp` keeps 0.05 m to stay in lock-step with
  the Python reference. On `low_wall` (start (0,0) → goal (3.5,0)) the wall-crossing hop grows
  from X 2.1 m / apex 0.88 m / 3.0 J to X 2.5 m / apex 1.28 m / 6.1 J.
- **`hopcopter` JHC max powered-climb time**: `t_p_high` raised from 0.7 s to 1.5 s to leave
  room for longer pushes toward taller `target_apex` values. Thrust is applied only while
  `jumping_state == 3`, so the push still ends at the detected apex.
- **`ballistic_motion_planner` min apex height**: raised `physics.min_apex` from 0.3 m to
  0.7 m, so every planned hop's arc must peak at least 0.7 m above the higher of its two
  endpoints. Updated in both the ROS parameter default (`ballistic_planner_node.cpp`) and the
  `PlannerParams` struct default (`hopping_astar.hpp`). The `parity_main.cpp` harness keeps
  0.3 m on purpose — it mirrors the Python reference `config.py` for the parity check and must
  stay in lock-step with it. Still below the first-hop validation ceiling
  (`min_apex < R1/(4·mu) ≈ 0.833 m`), so `validate_params` continues to pass.
- **`ballistic_motion_planner` initial apex**: lowered `physics.h_initial` from 1.0 m to
  0.8 m to match the 0.8 m hover height the robot starts hopping at. It seeds the energy chain:
  the start cell's virtual landing speed drops from √(2g·1.0/η) ≈ 5.29 m/s to
  √(2g·0.8/η) ≈ 4.73 m/s, so the first hop's reach shrinks from ≈ 3.97 m to ≈ 3.61 m. Updated
  in both the ROS parameter default (`ballistic_planner_node.cpp`) and the `PlannerParams`
  struct default (`hopping_astar.hpp`); `parity_main.cpp` keeps 1.0 m to stay in lock-step with
  the Python reference. `validate_params` still passes (first-hop tan floor 0.833 < ceiling 1.0).

### Fixed
- **A perception node left over from an earlier launch duplicated the sensor streams**
  (docs/lidar_map_quality_plan.md, Fix 1). On 2026-11-10 `gz_truth_tf` and `lidar_sim_adapter`
  from an 18:48 launch kept running, so FAST-LIO got every scan and IMU sample twice, out of
  order, cleared its buffers ("loop back, clear buffer") and diverged (0.15–3 m jumps at
  touchdowns, km in long runs); the maps were drawn on that pose. Signatures: 14–18 scans/s at
  the mapper, half the `data_lio` stamps duplicated. `lio_anchor` now refuses to release the
  robot while `/lio/points`, `/lio/imu` or `/lidar/points` (`single_publisher_topics`) has more
  than one publisher, and the mapper warns above 11 scans/s (`max_scan_rate`) or on a scan not
  newer than the last. On 2026-11-11, clean: FAST-LIO RMSE 1.4–1.7 cm, no duplicates,
  9.4–9.8 scans/s, map median 0.6–0.9 cm.
- **`clearance_apex` returned NaN, not inf, when an infinitely tall cell was under the arc**
  (`JumpLib/takeoff_solver.py`). `need > apex` was then false, and the hop flew. That was the
  maze's poles on a known map, and every unbounded unseen cell on a LiDAR map.
- **The touchdown march no longer misses a pad landed on near its far rim**
  (`JumpLib/touchdown.py`; `test_far_rim_of_a_pad`, `test_far_rim_onto_lower_ground`,
  `test_falls_off_a_rim`). The march only stopped at a sample where the foot was at or below the
  ground. A foot that reached a pad's top between two samples and passed its far rim before the
  next was above the gap at that sample, so the pad was skipped. On the 2026-11-06 lilypads run
  the foot landed 2 cm inside the far rim of the pad at (7.8, −0.5): 8 cm above the pad at the
  x 8.10 sample, over the gap at x 8.30. The march went on to the gap floor (touchdown x 9.0,
  2.75 m fall, 7.2 m/s arrival) while the robot landed at x 8.13. The takeoff was solved from the
  gap floor, the push target (0.8 m above ground 0) sat below the pad, so the hop off it was
  leg-only, landed on the near rim of the pad at (10.2, −0.6), and the robot flipped there. Now,
  whenever the ground drops from one sample to the next, the closed form is also solved for the
  previous sample's surface, and if the foot is still over it at that time, that is the touchdown.
  The sample spacing `ds` is 0.05 m (was 0.2 m), half a map cell, so no cell is stepped over; at
  ~20 lookups per fall the cost is negligible. Replaying that apex: CoM touchdown x 8.137 against
  8.133 measured. The old march misses it at 0.05 m too; the rim check is the fix.

### Removed
- The `stairs_with_curb` scenario: its map in `scenarios.hpp`, its Gazebo world
  `crazysim_stairs_with_curb.sdf`, its README section, and its entries in `mapper_check` and
  `ballistic_plan_check --lidar-maps`. No longer needed. The reference planner's
  `maps/stairs_with_curb.py` stays in `hopcopter-ballistic-planning/`.
- `hopcopter`: `JumpingHeightController` and its calls (`estimate_height`, `update_apex_state`,
  `step`). The class stays in `JumpLib/jumping_tools.py`, unused.
- `hopcopter`: the distance pop (`current_goal_tolerance`), `_load_next_waypoint`, the
  `waypoint_list` queue, `apply_follower_gating` and the `/trajectory_start_position`
  subscription. The gating never used the start position; it ran on the first plan in only 2 of
  9 runs, because that topic arrives after the marker, and it dropped points on turning paths.
  Also removed: `pending_new_traj`, which was written but never read.

## [3.0.0] - 2026-09-07 - Ballistic A* Is the Only Planner

### Removed
- **`rrt_star_planner` package**: the OMPL RRT* 2D global planner (`rrt_star` node,
  `/ompl_rrt_star_trajectory`, `/rrt_star_grid`) is deleted. The ballistic A* planner
  (`ballistic_motion_planner`) is now the sole global planner.
- **`local_planning` package**: the PCL landing-point scorer (`local_planner` node,
  `/local_planner/adjusted_waypoint`, `/local_planner/markers`) is deleted.
- **`hopcopter` local-planner wiring**: removed the `/ompl_rrt_star_trajectory`
  subscription, the `/jumping_state`, `/trajectory_queue_state`, and `/visited_waypoint`
  publishers, the `/local_planner/adjusted_waypoint` subscription, the
  `adjusted_waypoint_callback` and `_publish_queue_state` methods, the in-loop
  "Local Planner Integration" block, and the now-unused `Pose`/`Point`/`PointStamped`/
  `PoseArray`/`Int8` imports.

### Kept
- `hopcopter` still subscribes to `/ballistic_trajectory` (Marker id=400) and
  `/trajectory_start_position`, and `apply_follower_gating` still trims waypoints behind the
  robot — both are shared with the ballistic planner (comments corrected).

### Docs
- `README.md`, `CLAUDE.md`, and `nlp_goal_interface/docs/NLP_INTEGRATION.md` rewritten to a
  single 7-terminal ballistic pipeline (elevation map → ballistic planner → hopcopter).

---

## [2.4.0] - 2026-03-24 - LiDAR Tilt for Landing Zone Coverage

### Changed
- **LiDAR sensor tilt**: Increased downward pitch from 2° to 15° (0.0349 → 0.2618 rad)
  - Enables local_planner to see landing zones 0.5-1.0m ahead during jump planning
  - Previous 2° tilt resulted in 0 points in waypoint regions at jump start
  - New FOV: approximately +13.6° (up) to -43.6° (down) with ±28.6° vertical scan
- **RRT* max range filter**: Added `map.max_ground_range` (default 1.0m) to ignore
  distant ground returns that appear as obstacles due to tilted LiDAR geometry

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map.max_ground_range` | 1.0 | Max distance (m) for obstacle detection - points beyond ignored |

---

## [2.3.0] - 2026-03-24 - Platform Ground-Filter Fix

### Changed
- **RRT* planner ground filtering**: Added absolute z-floor threshold (`map.min_obstacle_z`)
  to prevent low traversable surfaces (e.g. the 0.1m landing platform) from being
  marked as obstacles during altitude changes
  - Ground threshold is now `max(robot_relative_threshold, min_obstacle_z)`
  - Robot-relative filter alone was altitude-dependent and misclassified the platform
    during takeoff/landing when the drone dipped below ~0.3m

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map.min_obstacle_z` | 0.15 | Absolute world-frame z floor (m). LiDAR points below this are always treated as ground. |

---

## [2.2.0] - 2026-03-22 - Voxel Grid Downsampling

### Changed
- **Point cloud downsampling**: Added voxel grid filtering before edge detection
  - `downsamplePoints()` reduces filtered region from ~10-20K to ~200-1000 points
  - Custom hash-based implementation (no PCL dependency for this step)
  - Applied after spatial filtering, before edge pre-computation

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `downsample_voxel_size` | 0.05 | Voxel grid cell size for downsampling (m) |

### Performance
- Before downsampling: 88ms - 577ms (variance from O(n²) edge detection)
- After downsampling: **3-7ms consistent** (well within 300-400ms apex window)
- Typical point counts: 8K-22K regional → 200-1000 downsampled

---

## [2.1.0] - 2026-03-22 - Performance Optimizations

### Changed
- **Point cloud pre-filtering**: Added `filterPointsInRegion()` to reduce cloud from ~131K to ~5-20K points
  - Filters to circular region around waypoint before any scoring
  - Radius = `candidate_grid_radius + max(obstacle_detection_radius, edge_detection_radius)`

- **Edge detection optimization**: Pre-compute edges ONCE, then measure distances
  - Added `findEdgePoints()` - scans region once to identify all edge points (O(n²) done once)
  - `scoreEdgeAt()` now just finds nearest pre-computed edge point (O(edges) per candidate)
  - Reduced complexity from O(candidates × n²) to O(n²) + O(candidates × edges)
  - **Result**: Planning time reduced from ~10 seconds to ~10-50ms

### Performance
- Before optimization: 577ms - 9838ms (unusable, missed apex timing)
- After optimization: ~10-50ms (well within 300-400ms apex window)

---

## [2.0.0] - 2026-03-22 - Landing Point Scoring System

### Changed
- **local_planner.cpp**: Complete rewrite of waypoint adjustment algorithm
  - Replaced potential field repulsion with multi-criteria scoring system
  - Waypoints now selected from discrete candidate grid rather than continuous displacement

### Added
- **Candidate grid generation**: Discretizes circular region around next_waypoint_
  - Configurable radius (`candidate_grid_radius`, default 0.3m)
  - Configurable step size (`candidate_grid_step`, default 0.1m)
  - Cartesian grid clipped to circular boundary

- **Slope scoring (Criteria 1)**: PCA-based surface normal estimation
  - Gathers LiDAR points within `slope_analysis_radius` (default 0.1m)
  - Computes surface normal via covariance matrix eigenvalue decomposition
  - Score decreases as slope angle increases toward `max_slope_angle` (default 0.5 rad)

- **Obstacle scoring (Criteria 2)**: Distance-based penalty
  - Finds minimum distance to any obstacle point above ground threshold
  - Score = min_dist / obstacle_detection_radius
  - Full score (1.0) if no obstacles within detection radius

- **Hop distance scoring (Criteria 3)**: Penalizes distant candidates
  - Measures 2D distance from candidate to current_goal_ (takeoff point)
  - Closer candidates receive higher scores

- **Edge proximity scoring (Criteria 4)**: Height discontinuity detection
  - Searches for points with neighbors that drop > `edge_height_threshold` (default 0.08m)
  - Score decreases as candidate approaches detected edges
  - Designed for stair climbing safety

- **Weighted scoring**: Configurable weights for each criterion
  - `weight_slope`, `weight_obstacle`, `weight_distance`, `weight_edge` (all default 1.0)
  - Total score normalized by sum of weights

- **Visualization**: Score-colored candidate grid in RViz
  - Candidates shown as spheres (red=low score, green=high score)
  - Selected point highlighted as larger green sphere
  - Takeoff point shown as yellow sphere
  - Search radius shown as cyan cylinder

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `candidate_grid_radius` | 0.3 | Radius of candidate search circle (m) |
| `candidate_grid_step` | 0.1 | Spacing between grid points (m) |
| `obstacle_detection_radius` | 0.5 | Radius for obstacle proximity check (m) |
| `slope_analysis_radius` | 0.1 | Radius for surface normal estimation (m) |
| `edge_detection_radius` | 0.15 | Radius to search for height drops (m) |
| `edge_height_threshold` | 0.08 | Z drop to classify as edge (m) |
| `max_slope_angle` | 0.5 | Maximum slope angle for scoring (rad) |
| `weight_slope` | 1.0 | Weight for slope criterion |
| `weight_obstacle` | 1.0 | Weight for obstacle proximity criterion |
| `weight_distance` | 1.0 | Weight for hop distance criterion |
| `weight_edge` | 1.0 | Weight for edge proximity criterion |
| `robot_leg_height` | 0.25 | For ground point filtering (m) |

### Removed
- `force_gain` parameter (potential field scaling)
- `max_waypoint_displacement` parameter (now implicit via grid radius)
- `min_obstacle_distance` parameter (clamping for force calculation)
- Potential field force calculation and visualization

---

## [1.0.0] - Previous - Potential Field Approach

- Initial implementation using potential field repulsion
- Single closest obstacle determined repulsive force
- Waypoint displaced away from obstacle up to max_waypoint_displacement
