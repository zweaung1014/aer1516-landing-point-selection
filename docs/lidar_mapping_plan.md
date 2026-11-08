# Plan: build the elevation map from the LiDAR

Written 2026-11-06 on branch `lidar-simulation-hopcopter-bmp`; **implemented 2026-11-07** on
`lidar-mapping-bmp` (see "Status" below). This is item 3 of Part 4 in
[next_steps_after_stage4.md](next_steps_after_stage4.md). The decisions (D1–D8, at the end) were
taken as recommended.

## Status (2026-11-07)

All stages are implemented. Every number below comes from headless runs with FAST-LIO, the
mapper, the planner and hopcopter together on the LiDAR map. The details, and why each parameter
has its value, are in CHANGELOG.md's [Unreleased] entry.

| Stage | Result |
|---|---|
| 0 | `hopcopter_perception` builds against the overlays; `fastlio_mapping` runs with `config/fast_lio_sim.yaml`. |
| 1 | Mid-360 upside down (10 Hz, 512 × 40 rays, 19–20 k finite points per scan), its IMU, one bridge (`config/bridge_sim.yaml`), `lidar_sim_adapter`, `/clock`. RTF 0.95–1.00 with everything running. Gazebo's scan stamps match its pose to the millisecond. |
| 2 | Hold and release works as planned: a hidden static DetachableJoint parent (`sitl_singleagent.sh -H`); no FAST-LIO patch. FAST-LIO, low_wall: RMSE 1.5–2.0 cm, max ≤ 4.1 cm, attitude ≤ 0.35°, no touchdown jumps. Other worlds: RMSE 1.3–3.1 cm; one 0.4 s attitude transient of 2° on lilypads. The same with the IMU clipped at ±32 g or ±16 g, so Point-LIO was not needed. Maze: drifted 9.5 cm along its 14 m corridor (RMSE 6.7 cm). |
| 3a/3b | `lidar_elevation_mapper`. Offline (`mapper_check`, nine worlds): median ≤ 4 mm, p95 ≤ 13 mm away from edges. Sim on FAST-LIO: median 0.4–0.7 cm, p95 1.5–1.8 cm. Edges stay within one cell. 3b matches 3a to < 1 cm on the low_wall bag. |
| 4 | The planner and hopcopter read the bound layer: start guard, frontier plans, hold-and-replan. `ballistic_parity_test` is byte-identical; the 15 original `ballistic_plan_check` cases are unchanged and 5 new partial-map cases pass. With the final settings, goals were reached on low_wall (Test 1, there and back; 4 of 4 runs), flat (Test 2; 2 of 2), lilypads and slope_30deg (the latter with the look-out hop). On maze and cliff_gap it was 1 of 2 runs each; the other runs fell, see below. Stairs is not reached. |
| 5 | `lidar_mapping.launch.py`; README, CLAUDE.md, CHANGELOG. |

What testing changed from the design above:
- **The LiDAR IMU runs at 1 kHz and is averaged to 200 Hz** in the adapter. Gazebo samples an IMU
  instantaneously, and at 200 Hz the mast-top IMU aliased a stance's rotation: one stance's
  velocity came out 6.4 m/s wrong, and FAST-LIO ran 1.2 m away. A real IMU's anti-aliasing filter
  averages instead.
- **FAST-LIO gets every 3rd point**, as the shipped Mid-360 config's `point_filter_num` 3 does
  (this fork's default handler ignores that parameter). Its IMU queue is 10 deep on the
  scan-processing thread, and 51 ms scans dropped a touchdown's samples.
- **FAST-LIO's voxels are 0.1 m and `acc_cov` 3.0.** At 0.2 m the height drifted 4 cm; at 0.1 the
  horizontal speed lagged after fast touchdowns.
- **The mapper's rules needed hardening against gpu_lidar's mixed pixels** at silhouettes:
  - rays carve only near cell centres;
  - the bound is the third-lowest ray;
  - three rays must pass below a height to remove it, and rays through well-supported heights
    are dropped;
  - a height counts after 2 hits and three rays above it;
  - no variance inflation, and a 1 cm floor on each cell's sigma.
- **The planner on LiDAR maps:**
  - terrain constrains an arc only 5 cm above both ends (LiDAR ground is mm-rough);
  - landings keep 0.3 m from unseen cells and one cell further from steep edges, and steps count
    as edges 8 cm sooner;
  - from a takeoff inside the clearance inflation, the first 0.25 m read the terrain itself.

  Without these, the plans landed on a wall's top and on stair risers, and in the maze the robot
  was trapped beside a pole.

Open:
- **Stairs (0.3 m treads, 0.18 m risers).** The mapper spreads each riser over two cells, and
  with the margins above no tread is landable. The robot goes to the side of the staircase and
  waits; it no longer falls. Options: a mapper that localizes step edges (e.g. the max of the
  face hits in front of a face), or `standable.lidar_edge_margin:=0` with
  `standable.lidar_steep_noise` tuned per world.
- **Landings near edges (a decision for you).** With `standable.lidar_edge_margin` 0.1, a
  landing can sit 0.05–0.15 m from a real edge, and two runs fell:
  - maze: the plan landed on the 0.3 m-wide wall (the known map leaves its middle cell
    standable too);
  - cliff_gap: the plan landed 0.15 m from the far platform's 1.8 m face (as the known-map plan
    does). A replan fired mid-flight, and the robot came down on the rim and fell back into the gap.

  A margin of 0.2 prevents both, but leaves lilypads' smallest pads (r 0.35 m) only their centre
  cell, and the pit-unseen `ballistic_plan_check` case then has no path. Better fixes:
  - a margin that grows with the drop (or a "no drop within 0.25 m" footprint test);
  - not replanning during a hop onto a platform;
  - a mapper that localizes faces instead of spreading them a cell outward.
- **Maze odometry drift** (9.5 cm over 14 m). Mapping and flying still worked.
- **Replanning on the move**, the look-out hop by hand, and the controller on LIO remain "Later".
- **Not done: anchoring without mocap** (camera_init levelled by gravity, the start ground at 0).
  The sim anchors on Gazebo's pose.

## Goal

Today `elevation_map_publisher` publishes a hand-made map of each world, and the planner and
hopcopter trust it. The goal is the pipeline a real robot would run:
1. the LiDAR scans the world;
2. FAST-LIO estimates the LiDAR's pose;
3. a mapper turns the registered scans into the same 2.5D `/elevation_map`;
4. the planner plans on that map;
5. hopcopter flies the plan exactly as it does now.

**What does not change:**
- the planner's search and robot model;
- the takeoff solver and the push controller;
- the `/ballistic_trajectory` + `/ballistic_hop_plan` contract;
- `/elevation_map` as the topic that both the planner and hopcopter read.

`elevation_map_publisher` stays. It becomes the ground truth that the LiDAR map is scored
against, and the known-map mode stays available for controller work.

## Target pipeline

```
Gazebo gpu_lidar + LiDAR IMU ─▶ ros_gz_bridge ─▶ lidar_sim_adapter ─▶ FAST-LIO ─▶ /Odometry, TF camera_init→body
                                                                          │ /cloud_registered (dense)
                 lio_anchor: TF world→camera_init, release ───────────────┤
                 hopcopter: /hopcopter/foot_contact ──────────────────────┤
                                                                          ▼
                                                            lidar_elevation_mapper
                                                                          │ /elevation_map (+ upper_bound layer)
                                                                          ▼
                                    /goal_pose ─▶ ballistic_planner (+ replanning) ─▶ hopcopter
elevation_map_publisher ─▶ /elevation_map_known   (scoring only)
```

New nodes go in a new package, `hopcopter_perception`: the adapter, the anchor, the mapper,
the FAST-LIO config, the bridge YAML and a launch file.

## What the investigation found

These findings shape the plan. The file references show where to look.

1. **FAST-LIO cannot start while the robot falls, and the sim drops the robot at Play.**
   FAST-LIO estimates gravity from the first ~10 IMU samples after the first scan
   (`MAX_INI_COUNT` in `~/fastlio_ws/src/FAST_LIO_ROS2/src/IMU_Processing.hpp`). It scales
   every later accelerometer sample by 9.81/|mean| of those samples. The robot spawns at z 1.34
   (3.4 on lilypads) and is in free fall from the moment Play is pressed. Those first samples
   therefore read near zero, and the scale factor blows up. FAST-LIO needs about 0.1 s of
   stillness before the drop.
2. **Gazebo scans are instantaneous, and some points are not finite.**
   - gpu_lidar renders each scan at a single instant, so every point's time offset is 0. That
     is correct, and there is nothing to undistort.
   - Some of FAST-LIO's input handlers (`src/preprocess.cpp`) invent per-point times from each
     point's azimuth. The Velodyne handler does this whenever the last point's time is 0; the
     MID360 handler always does. Either one would "undistort" the scan by up to a full rotation
     of motion.
   - The default handler keeps time 0. It is the handler for any `lidar_type` other than 1–4.
     In this fork 4 means MID360.
   - The default handler does not drop non-finite points (rays that hit nothing).
3. **The simulated LiDAR is about 40× a real one, and the controller is tuned at real time.**
   - The template's sensor fires 2048 × 64 rays at 60 Hz: 7.9 M points/s (`model.sdf.jinja`
     ~234). A Livox Mid-360 gives 200 k points/s at 10 Hz. The LiDAR link's 265 g mass is a
     Mid-360's.
   - hopcopter's 100 Hz loop runs on wall time, and it was calibrated at RTF 0.98 with the
     LiDAR not bridged.
   - If rendering, the bridge or FAST-LIO slow the sim down, the push cutoff's timing
     constants stop matching.
4. **The current mount leaves big blind zones, and the start cell is never seen.**
   - The sensor sits 0.4 m above the base, pitched 15° down, with ±28.6° of vertical FOV. That
     puts it 0.77 m above the ground at touchdown and 1.57 m at a 0.8 m apex.
   - It first sees the ground 0.8 m ahead (1.6 m at the apex), 1.4 m to the sides (2.9 m) and
     3.2 m behind (6.4 m).
   - Yaw is held at 0, so the robot always faces +x and looks backward with the short side.
   - The 15° tilt was tuned for the removed local planner (CHANGELOG 2.4.0). Nothing in the
     ballistic pipeline needs it.
5. **On a LiDAR map, "unknown" must stop meaning "infinitely tall".** The planner turns NaN into
   an infinite column (`ballistic_planner_node.cpp:269`), and so does hopcopter's clearance field
   (`takeoff_solver.py:356`). That is right for the hand-made maps and wrong for cells the LiDAR
   has not seen. It breaks these worlds:
   - **low_wall:** at the start's apex, the ground behind the wall is hidden from x 2.7 to 3.6 m.
     That region holds the plan's 3.55 m landing. As infinite columns, the hidden cells block
     the wall hop.
   - **lilypads:** much of the 2 m pit is hidden behind the pads, which would block most
     pad-to-pad hops.
   - **slope_30deg** (2.02 m platform) and **cliff_gap** (1.8 m platforms): the goal is higher
     than the sensor ever gets from the ground, so the LiDAR does not see it until the robot has
     climbed.
6. **An unknown start cell causes two silent failures.**
   - The planner reads the start cell's height without any check (`hopping_astar.cpp:306` and
     `:587`). An unknown start plans every hop from z = −1.0, the OBSTACLE sentinel.
   - For unknown cells, hopcopter falls back to `current_ground_z`. That value is 0.0 and is
     never updated (`hopcopter.py:230`). On lilypads' 2 m platform the push would aim 2 m too
     low.
7. **The planner plans once per goal, takes 4–20 s, and assumes a vertical arrival at the
   start.** A map that grows while the robot moves needs replanning. A replan started while the
   robot is moving would plan from where it was seconds earlier.
8. **FAST-LIO's frame is not the world frame.** It publishes in `camera_init`, which is the IMU's
   pose at initialization: the origin is at the sensor, and the frame is not gravity-aligned
   unless the IMU was level. The planner, the goals and the controller all use Gazebo's world
   frame.
9. **Touchdowns are hard on an IMU.** Stance lasts 30–48 ms with 23–30 g peaks (stage 0). The
   base_link IMU is perfect and has no range limit, but a real LiDAR's IMU may saturate at
   touchdown. FAST-LIO has no handling for saturation. Point-LIO (from the same lab) does, and it
   can start from a given gravity vector instead of a static initialization.
10. **Most of the software is already on this machine.**
    - `~/fastlio_ws` has FAST_LIO_ROS2 (Ericsii's port, commit `18418bc`), already built.
    - `~/ws_livox` has livox_ros_driver2, and Livox-SDK2 is installed. FAST-LIO needs the
      driver's message package to build.
    - `~/slope_ws` has elevation_mapping_cupy, not built. It needs CUDA and cupy, which are not
      installed.
    - grid_map (with grid_map_pcl), pcl_ros, tf2_sensor_msgs, and gz-sim 8.10's DetachableJoint
      system are installed.
    - The GPU is an RTX 500 Ada with 4 GB.

## Design

### Sensors in Gazebo

- **LiDAR** (`model.sdf.jinja`): simulate the sensor you will fly (D1). For a Mid-360 that means
  10 Hz, about 20 k rays per scan (for example 512 × 40 over 59°), 0.1–40 m range and 2 cm
  Gaussian range noise. gpu_lidar cannot produce the Mid-360's non-repetitive scan pattern. It
  fires rings at fixed angles, and the hopping sweeps those rings across the ground.
- **LiDAR IMU:** a second `imu` sensor at the LiDAR's pose, publishing `/cf_0/lidar_imu` at
  200 Hz with the real IMU's noise. FAST-LIO uses this IMU. The base_link IMU (1 kHz, used by the
  firmware and by hopcopter) stays as it is, so the controller's calibration still holds.
- **One bridge process,** configured from a YAML file. It carries `/clock`, the LiDAR points, the
  LiDAR IMU and the release topic, plus today's `/cf_0/imu` and `/cf_0/odom`.
- **`lidar_sim_adapter`** (sim only):
  - drops non-finite points and points inside the blind radius;
  - optionally clips the IMU at the real sensor's range, so saturation can be switched on and
    off;
  - publishes `/lio/points` and `/lio/imu`.

### Start: hold, initialize, release

1. A DetachableJoint holds base_link where it spawns. FAST-LIO initializes while the robot hangs
   still, with motors idle as they are now.
2. When FAST-LIO publishes its first odometry, `lio_anchor` waits about 1 s for a few more scans
   and then anchors the frames.
3. `lio_anchor` publishes the release, and the robot drops exactly as it does today.
   hopcopter's start-up (`ready_to_drop`, the warm-up hops) does not change.

This is the sim's version of holding the robot by hand and letting go. A bonus: the first scans
come from the hanging pose, 1.74 m up, the best vantage point of the run.

**To check first:** which way round the joint has to go. One option is the plugin in the robot
with the ground plane as the child; the other is a hidden fixed holder model with the robot as
the child. If neither works, the fallback is a small FAST-LIO patch that reads gravity from a
parameter instead of averaging the first samples.

### Frames

`lio_anchor` publishes one static transform, taken during the hold:

world → camera_init = T(world ← imu, from Gazebo) · T(camera_init ← imu, from FAST-LIO)⁻¹

After that, mapping uses only FAST-LIO. In a mocap lab you would do exactly this, using the mocap
pose at the start. Without mocap, world would be camera_init levelled by FAST-LIO's gravity
estimate, with z set so that the start ground is at 0.

While the robot flies, `lio_anchor` also logs FAST-LIO's pose against Gazebo's for every scan,
to `data/data_lio/`.

### FAST-LIO configuration

`hopcopter_perception/config/fast_lio_sim.yaml` sets:
- the default (time-0) input handler, with `lidar_type` 5;
- `blind` about 0.3 m and `point_filter_num` 1–2;
- `filter_size_surf` and `filter_size_map` about 0.2–0.3 m (this is a 20 × 12 m room, not a
  street);
- the extrinsics from the template (identity rotation if the IMU sits at the LiDAR's pose), with
  `extrinsic_est_en: false`;
- `dense_publish_en: true`, because the mapper wants every point;
- `det_range` about 30 m;
- IMU covariances from the simulated noise;
- `use_sim_time: true`.

Every world has the 1.2 m yellow boundary walls. They are visual-only, but gpu_lidar renders
visuals, so the LiDAR sees them. Together with the ground they constrain all six degrees of
freedom, so even `flat` is not degenerate for LIO.

### The mapper: `lidar_elevation_mapper`

A C++ node (D7). It subscribes to a point cloud in any frame, and it looks up the cloud's
transform to world, and the sensor origin, in TF at the cloud's stamp. The same node can then
run on two inputs:
- FAST-LIO's dense `/cloud_registered` (stage 3b, and the real robot);
- the raw scan with Gazebo's pose (stage 3a, and a mocap lab).

By default the map uses the scenario map's extent and 0.1 m resolution, so every cell lines up
with the known map and with the planner's lattice. The scenario sets only the map's size and the
scoring reference.

The mapper keeps three things per cell:
- **Height.** A Kalman estimate with a variance, in the style of Fankhauser et al. (ANYmal's
  `elevation_mapping`):
  - a hit well above the estimate replaces it (a higher surface);
  - a hit well below it is ignored (a wall's face, under its top);
  - anything in between fuses with the estimate.

  The variance grows with range and with LIO's attitude uncertainty. Hits beyond about 8 m do
  not update heights.
- **Upper bound.** A ray that passes over a cell without hitting it proves that the terrain
  there is lower than the ray. Each scan's rays are traced through the grid, and each cell keeps
  the lowest ray height that passed over it.
  - This is what makes unseen cells usable. The ground hidden behind the low wall is bounded by
    the rays that grazed the wall's top. It is at most 0.4 m high, so the robot may fly over it.
  - The same test removes heights that a ray has passed below (outliers, or a LIO glitch).
- **Foot contacts.**
  - hopcopter publishes the foot position at every touchdown, and the mapper writes it as a
    near-certain height.
  - At the first touchdown, the mapper also fills a disk around the start at the foot's height
    and flags those cells as assumed. Any LiDAR hit replaces them. The disk's radius is the blind
    radius ahead: 0.8 m with the current mount.
  - This is the usual assumption that the robot was put down on level ground. It covers the
    blind zone the robot takes off over. Without it, the rays that bound that zone form a cone
    0.77 m high next to the robot, and the first hop fails its clearance check.

The mapper publishes `/elevation_map` at 2 Hz (transient_local, as now) with four layers:
`elevation` (NaN = never measured), `upper_bound`, `variance` and `hits`.

**Scoring.** With the scenario's known map running on `/elevation_map_known`, the mapper logs for
every update, to `data/data_elevation_map/`:
- coverage;
- height error on observed cells;
- false obstacles (observed more than 10 cm above the truth);
- missed obstacles.

It also writes a snapshot that a plotting script can show next to the known map and the error.

### The map contract

| Cell | `elevation` | `upper_bound` | Planner | hopcopter |
|---|---|---|---|---|
| Measured | height | any | as now | as now |
| Unseen, bounded | NaN | finite | terrain at the bound for clearance; never a landing | clearance at the bound; the touchdown march falls through it |
| Unseen and unbounded, or a hand-made obstacle | NaN | NaN or no layer | OBSTACLE, as now | as now |

The hand-made maps have no `upper_bound` layer, so they keep today's meaning, and
`ballistic_parity_test` and `ballistic_plan_check` are unaffected.

### Planner changes

- **Map conversion.** `convertMap` builds the grid as the table says, plus a "measured" mask that
  `standable_mask` ANDs in.
- **Start guard.** The planner refuses to plan if the start cell is unmeasured, instead of
  silently planning from z = −1.
- **Goal not seen yet.**
  1. A cheap 2D reachability flood finds the measured, standable cells the robot can reach.
  2. The planner plans to the one nearest the goal and marks the plan as a frontier plan.
  3. It replans when the goal cell becomes measured or the robot reaches the frontier.
- **Replanning.** On every map update, the planner re-checks the remaining hops of the current
  plan:
  - the landing cell is still standable, and its height has moved by less than 5 cm;
  - each arc still clears the terrain with its planned takeoff.

  hopcopter publishes which waypoint it is on, so the planner knows which hops remain. When the
  check fails, or a frontier plan can now do better:
  1. publish DELETE alone (hopcopter already drops a path 0.5 s after a DELETE with no ADD, and
     hops in place);
  2. wait for in-place hops;
  3. plan from there and publish the new path.

  At most one replan every ~5 s. Planning from a hold keeps the search's vertical-arrival start
  valid. Replanning on the move needs the start's arrival velocity and is a later improvement.
- **Snapshots** carry the measured mask and the bound, so `visualize_planned_path.py` can shade
  what the planner had not seen.
- **New offline `ballistic_plan_check` cases:** low_wall with the shadow unseen but bounded,
  lilypads with the pit unseen, and an unmeasured start.

### hopcopter changes

- The clearance re-check uses the same composed field as the planner.
- In the touchdown march, an unseen cell has no ground (the planner never lands there). For the
  takeoff ground, an unseen cell falls back to the last foot contact instead of 0.
- hopcopter publishes `/hopcopter/foot_contact` at each touchdown, and the tracker's decision at
  each apex.
- Measure the time `elevation_map_callback` takes on the 100 Hz thread. At 2 Hz it should cost
  1–3 ms per map.

## Stages

| Stage | What | Verified by |
|---|---|---|
| 0 | Decisions D1–D8; FAST-LIO builds against this workspace; a baseline run | build, one known-map run |
| 1 | Sensors, bridge, adapter, `/clock` | RTF, point clouds, unchanged controller numbers |
| 2 | FAST-LIO, hold and release, anchor | LIO against Gazebo truth while hopping |
| 3a | Mapper on Gazebo's pose | map against the known map, from recorded bags |
| 3b | Mapper on FAST-LIO | the same, compared with 3a |
| 4 | Planner and hopcopter on the LiDAR map | offline plan check, then every world |
| 5 | One launch file; README, CLAUDE.md, CHANGELOG | a run from fresh terminals using the README |

Order: 0 → 1 → (2 and 3a in parallel) → 3b → 4 → 5. The planner half of stage 4 is offline work
and can start any time after stage 0.

### Stage 0: decisions and prerequisites

- Make D1–D8.
- Build `hopcopter_perception` against the FAST-LIO overlay (D8) and confirm `fastlio_mapping`
  starts with the new config.
- Record one known-map low_wall run, so later runs have a baseline to compare against.

### Stage 1: sensors in Gazebo

- Change the LiDAR, add the LiDAR IMU, and write the bridge YAML and `lidar_sim_adapter`.
- Check the point fields with `ros2 topic echo --once --no-arr /cf_0/lidar/points`. Expect x, y,
  z, intensity and ring, and confirm that rays that hit nothing come through as non-finite.
- **Pass:**
  - RTF ≥ 0.95 with the LiDAR bridged;
  - 10 Hz scans with about 20 k finite points, and correct frames in RViz;
  - hopping in place on flat for 1 min, `calibrate_hops.py` matches the stage-4 numbers (apex
    within ±0.03 m, landing prediction median under 0.01 m).
- Record bags in low_wall and lilypads, each with in-place hops and the Test 3 paths. Stages 2
  and 3 replay them instead of re-running seven terminals. At 20 k points × 10 Hz that is about
  6 MB/s.

### Stage 2: FAST-LIO on a hopping robot

- Implement hold and release (check the joint first, as noted in the design), the anchor, and
  the FAST-LIO config.
- **Pass,** in each world, hopping in place for 2 min and then flying the Test 3 paths:
  - position RMSE ≤ 5 cm and max ≤ 10 cm;
  - attitude error ≤ 1°;
  - no jump in the error at touchdowns, and no divergence.
- Then switch on the realism one setting at a time: the IMU at 200 Hz, IMU noise, and saturation
  at the real IMU's range.
- **Gate:** if FAST-LIO diverges at touchdowns with the real IMU, switch to Point-LIO before
  stage 3b.

### Stage 3: the mapper

- 3a runs from the bags with Gazebo's pose; 3b runs on FAST-LIO.
- **Pass,** in every world after warm-up and one path:
  - height error on observed cells: median ≤ 2 cm, 95th percentile ≤ 5 cm;
  - no false obstacles on flat ground;
  - walls and step edges within one cell of the truth;
  - along the known-map plan, every landing cell measured, and every cell under every arc
    measured or bounded;
  - 3b within about 1 cm of 3a.

### Stage 4: planning and flying on the LiDAR map

1. **Offline first.**
   - `ballistic_parity_test` output stays byte-identical.
   - The 15 existing `ballistic_plan_check` cases are unchanged.
   - The new partial-map cases plan as expected: the robot flies over shadows and never lands
     in them.
2. **Then the sim,** in this order: flat, low_wall (Test 1 of `next_steps_after_stage4.md`,
   there and back), lilypads, stairs, maze, slope_30deg, cliff_gap.
3. **Pass:**
   - the Test 1/2 criteria: no hold, miss ≤ 0.35 m, tilt ≤ 20°, apex within ±0.1 m;
   - every replan logged with its reason;
   - slope_30deg and cliff_gap reach the goal through frontier plans. If cliff_gap's frontier
     dead-ends on the ground beside the platform, see D6.

### Stage 5: bring-up and docs

`ros2 launch hopcopter_perception lidar_mapping.launch.py scenario:=<world>` starts the bridge,
the adapter, FAST-LIO, the anchor and the mapper. It replaces today's terminals 3–5, so a run
takes five terminals:
1. Gazebo;
2. the cflib server;
3. the perception launch;
4. the planner;
5. hopcopter.

A run then goes like this:
1. Press Play.
2. The robot hangs for 1–2 s until FAST-LIO is ready, then drops.
3. It makes its warm-up hops while the map fills in.
4. Send the goal.

Update the README, CLAUDE.md (architecture, map contract, the sim-time nodes, hold and release)
and the CHANGELOG.

## Decisions

All eight were taken as recommended (2026-11-07).

| | Decision | Recommendation |
|---|---|---|
| D1 | Which LiDAR and IMU to simulate, and the mount | The one you will fly. The 265 g in the model and your `mid360.yaml` point to a Mid-360: 10 Hz, ~20 k points, its built-in IMU at 200 Hz with the datasheet's noise and range. Mount it upside down (−52° to +7°): the blind radius becomes 0.6 m all round at touchdown, against today's 0.8 m ahead and 3.2 m behind. Upright, a Mid-360 first sees the ground ~6 m away, which is useless for terrain. Keep the sensor 0.4 m above the base unless the real robot differs. |
| D2 | LIO | FAST-LIO2, as you asked; it is built and on ROS 2. Move to Point-LIO only if stage 2's gate fails with the real IMU (check its ROS 2 support first). |
| D3 | How the robot starts | Hold and release. It needs no FAST-LIO change and mirrors releasing it by hand. Fallback: the gravity-from-parameter patch. |
| D4 | The controller's state source in this phase | Gazebo's truth, as now. That is the same as flying in a mocap lab, and it keeps FAST-LIO's job to the map, so map errors and controller errors stay separable. Putting the controller on LIO is a separate project (see "Later"). |
| D5 | Unseen terrain | Never land on it; fly over it only above its bound; a start disk with the blind radius. |
| D6 | Goals the LiDAR cannot see yet, and the growing map | Frontier plans plus hold-and-replan, at most one replan every ~5 s. For goals on platforms above the sensor (slope_30deg, cliff_gap), do a look-out hop before sending the goal: `ros2 param set /hopcopter desired_rel_z 1.4` lifts the sensor to ~2.2 m at the apex. Automate it later if it works. |
| D7 | The mapper | A purpose-built C++ node. It gives the exact contract and the bound and foot-contact layers, with no CUDA. elevation_mapping_cupy would need CUDA and cupy on a 4 GB GPU that Gazebo also renders on, and still need those layers added. |
| D8 | Where FAST-LIO lives | Keep `~/fastlio_ws` and `~/ws_livox` as overlays (source `/opt/ros` → `ws_livox` → `fastlio_ws` → `ros2_ws`) and record the commit in the README. The config and launch file live in this repo. Vendor FAST-LIO later if you want the repo self-contained. |

## Risks

1. **FAST-LIO through touchdowns** is the most likely blocker, especially with a saturating
   200 Hz IMU. Stage 2's gate catches it before any mapping work depends on it, and Point-LIO is
   the fallback.
2. **The hold may not hold.** The FAST-LIO init patch is a few lines.
3. **RTF.** If it drops below 0.95, reduce the LiDAR further or run without the GUI, and re-run
   `calibrate_hops.py`.
4. **Map churn.** Small height changes could trigger constant replans, and each replan costs
   ~5 s of hopping in place. The 5 cm tolerance and the rate limit are there to prevent this; the
   replan log shows whether they are enough.
5. **Frontier dead ends** (cliff_gap): use the look-out hop, or send intermediate goals.
6. **The sim is kinder than reality.** Its scans have no motion distortion, its rings are
   regular, and its timing and extrinsics are perfect. A real Mid-360 on a hopping body will be
   harder, which is why stage 2 adds the realism settings one at a time.

## Later (not in this plan)

- **The controller on LIO instead of Gazebo's truth.** It needs an IMU-rate state, but FAST-LIO's
  odometry comes at 10 Hz. One option is to feed the LIO pose to the firmware's estimator as an
  external pose, the way a real Crazyflie takes mocap. It also needs a fresh stage-0 calibration.
- **Replanning on the move**, with the start's arrival velocity.
- **Yawing toward the direction of travel**, so the LiDAR's best side faces where the robot goes.
