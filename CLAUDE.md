# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Perception-informed autonomous navigation for **Hopcopter**, a Crazyflie-based monopedal
hopping robot, simulated in Gazebo Harmonic and driven by a ROS 2 Humble pipeline. A goal
pose (typed as a ROS command or spoken in natural language) is turned into a global path of
physically feasible parabolic hops by a hopping-aware **A\* ballistic planner** over a 2.5D
elevation map, and the `hopcopter` node flies the robot along it by commanding
roll/pitch/yaw/thrust through a hop cycle. **The robot HOPS** — it is airborne between
waypoints and the controller works in phases, so almost every timing question ("when is a
waypoint chosen?") resolves to "at a particular jumping state."

### How planning works

The shipped pipeline plans on a **2.5D elevation map** — a grid of elevations where each move
is a physically feasible parabolic hop. `ballistic_motion_planner/` (C++ ROS 2 node) is the
live planner; it was ported from the standalone Python implementation in
`hopcopter-ballistic-planning/`, which **remains in this repo as reference context only**: it
is not built, launched, or imported by the simulation, and nothing in `ros2_ws/` depends on
it. Read its own `CLAUDE.md` (thorough, 579 lines) and `docs/planner.md` before changing the
algorithm. Do not "fix", refactor, or wire up the standalone copy unless explicitly asked.

The elevation map is published by `elevation_map_publisher/` as a **known** map (LiDAR-based
map building comes later). An earlier 2D pipeline (an OMPL RRT\* global planner plus a PCL
landing-point local planner) has been **removed** — do not reintroduce it.

## Repository layout

Everything is one flat git repo — there are **no submodules**. `crazyflie-firmware/`,
`crazyflie-lib-python/`, `crazyflie-clients-python/`, and `ros2_ws/src/crazyswarm2/` are
vendored upstream trees, checked in and locally modified (notably the Gazebo model and world
under `crazyflie-firmware/tools/crazyflie-simulation/`). The project's own code is:

| Path | What it is |
|---|---|
| `ros2_ws/src/ballistic_motion_planner/` | C++ hopping-aware A* planner (`ballistic_planner_node`) |
| `ros2_ws/src/elevation_map_publisher/` | C++ publisher of the known 2.5D `/elevation_map` (`elevation_map_publisher_node`) |
| `ros2_ws/src/hopping_robot/` | Python hop controller (`hopcopter`) + `JumpLib`/`RisLib` |
| `ros2_ws/src/nlp_goal_interface/` | Python Claude-powered goal parser (`nlp_goal_node`) |
| `ros2_ws/src/tf_transform_broadcast/` | Python TF broadcaster |
| `ros2_ws/src/tutorial_interfaces/`, `pc2tocustom/` | Livox custom point messages (legacy path, not in the live pipeline) |
| `crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/` | SITL launch scripts, world, and the robot model **jinja template** |
| `hopcopter-ballistic-planning/` | Reference-only standalone Python 2.5D ballistic planner (the port source) |

## Build and run

```bash
cd ~/CrazySim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build                                          # full workspace
colcon build --packages-select ballistic_motion_planner   # one package (C++ changes; builds Release by default)
source install/setup.bash                             # required in EVERY terminal
```

Python packages (`hopping_robot`, `nlp_goal_interface`, `tf_transform_broadcast`) are
`ament_python`; changes still need a `colcon build` + re-source because the nodes run from
`install/`.

Running the simulation takes **seven terminals**, each sourced as above; `README.md` has the
exact commands in order (Gazebo SITL → cflib server → IMU/odom `ros_gz_bridge` →
`elevation_map_publisher` → `ballistic_planner_node` → `hopcopter`). Nothing starts moving
until **Play** is pressed in the Gazebo GUI. A goal then goes in either by publishing
`/goal_pose` directly or by running `nlp_goal_node`.

There is **no test suite for the simulation**. The `test/` directories in the Python packages
contain the stock ament flake8/pep257/copyright linters, plus hopping_robot's
`test_waypoint_tracker.py`. Run it with
`cd ros2_ws/src/hopping_robot && /usr/bin/python3 -B -m pytest -p no:cacheprovider test/test_waypoint_tracker.py`.
It covers waypoint decisions and the braking tilt guard. Everything else is verified by running
the sim and inspecting the CSVs it writes (below).

### Analysis scripts (run from the repo root, not the workspace)

```bash
python graph_results.py              # latest data/data_rrt_star/ path, XY plot (historical RRT* runs)
```

> `visualize_local_planner.py` and `local_planner_description` are leftovers from the removed
> RRT*/local-planner pipeline and only apply to historical `data/data_local_planner/` logs.

Both default to the newest CSV in the relevant `data/` subdirectory. Note their axis
convention: **+x is drawn vertically up, +y horizontally left.**

For the live pipeline, `python visualize_rpy.py` plots commanded vs actual attitude and
`python calibrate_hops.py` reports g′, η per stance, the odom twist frame, push thrust, cutoff
latency and `/cf_1/pose` error. Both read hopcopter's newest per-tick
`data/data_ballistic_planner/position_attitude_tracking/attitude_*.csv` and need pandas (the repo
`.venv`, not `/usr/bin/python3`). Per-apex decisions go to `hop_tracking/apex_decisions_*.csv`.

## Architecture

```
/elevation_map ──▶ ballistic_planner ──▶ hopcopter ──▶ /cf_1/cmd_vel_legacy
  (2.5D GridMap)     ▲                     (roll/pitch/yaw/thrust)
                     │
                /goal_pose  ← ros2 topic pub, or nlp_goal_node
```

**1. `ballistic_planner_node`** (`ballistic_motion_planner/src/ballistic_planner_node.cpp`)
subscribes to the 2.5D `/elevation_map` (`grid_map_msgs/GridMap`, transient_local), the robot
pose, and `/goal_pose`. On each goal it runs a hopping-aware **A\*** search over the elevation
map (each move a physically feasible parabolic hop) and publishes the path **once** — there is
no replan timer. The path travels as a `Marker` SPHERE_LIST (id=400) on `/ballistic_trajectory`
(**waypoints travel as marker points, not a `nav_msgs/Path`**), with the robot's position at
planning time on `/trajectory_start_position`. Per-waypoint z is the **`target_apex`** of the
hop that lands on that waypoint — the planned apex height above that hop's takeoff ground
(`apex_drop + Z`) — and `output.hover_offset` (0.8 m) for the start point, which no hop lands
on. It is a relative height, not a world z. The node works entirely in the world frame and does
not use TF or the LiDAR. Each plan also writes `snapshot_<stamp>.json` to
`data/data_ballistic_planner/vis_planned_path/`. The node then starts
`scripts/visualize_planned_path.py` in the background, which renders `planned_path_<stamp>.png`
with the same layout as the reference `main.py`, plus a side view of each hop (arc, inflated
terrain and clearance from the planner's own `arc_samples`). Turn this off with `output.render_png:=false`.

**2. `hopcopter`** (`hopping_robot/hopping_robot/hopcopter.py`) follows the path one waypoint
per hop. It consumes the marker (id=400) into a `WaypointTracker` (`JumpLib/waypoint_tracker.py`,
pure Python, unit-tested in `test/test_waypoint_tracker.py`). At each steering apex (3 → 1), at
most once per real touchdown, the tracker decides from the predicted **foot** landing point P
(LSE's CoM prediction minus the leg along the planned landing attitude) where the next hop aims:
- **track:** P is within `reach` (0.8 m) of the waypoint this flight was aimed at, so aim at the
  next one with its `target_apex`;
- **retry:** P missed it, so aim at it again; after two retries, **hold**;
- **goal:** on the last waypoint, hop in place there;
- **idle:** no path yet, hop in place at the start.

A new path re-syncs to the waypoint nearest P, and the tracker catches up past waypoints the
robot overshot. A DELETE keeps the current path unless no ADD follows within 0.5 s. There is no
distance pop and no counter, so a missed apex costs one hop. Hops in place use `desired_rel_z`
(0.8 m), a ROS parameter that can change mid-run
(`ros2 param set /hopcopter desired_rel_z 1.0`).

At each apex `JumpingHeightController.step()` turns the chosen target apex into the next
powered-climb duration (fixed thrust, duration clamped to `t_p_low..t_p_high`). This is how hop
height follows the plan. Warm-up hops use `desired_rel_z`. Height control is approximate: JHC
estimates the last hop's energy from fall time and reacts one hop late. The takeoff angle is
still chosen by `LinearJumpingController`, not the planner. Its `limit_landing_tilt` (braking
tilt guard) rotates a takeoff that needs more than 20° of landing tilt toward continuing the
arrival. Control runs on a 100 Hz timer (`RPYT_commands`) around `JumpingStateTrackerOnboard`
(`JumpLib/jumping_tools.py`), whose `jumping_state` is the clock for the whole system:

- **1 — falling/ballistic:** unpowered, attitude tracked from the planned arc (`LinearJumpingController`).
- **2 — stance:** foot down, inverted-pendulum swing, unpowered.
- **3 — climbing:** powered thrust; a new hop begins here.

After the apex (3 → 1) it predicts the landing point (`LandingStateEstimator`, using
`leg_length` 0.366 m as the touchdown CoM height and g′ 9.55), decides the waypoint, plans the
ballistic arc, and derives the roll/pitch that steer it. Each decision is a row in
`hop_tracking/apex_decisions_*.csv`.

**3. `elevation_map_publisher_node`** (`elevation_map_publisher/`) publishes the **known** 2.5D
map on `/elevation_map` (`grid_map_msgs/GridMap`, transient_local so late subscribers still get
it). The scenario is selectable by parameter (`low_wall` or `flat`). `ballistic_planner_node`
cannot plan without it. `hopcopter` still subscribes and caches it (`_ground_elevation_at`), but
that lookup no longer drives hop height. LiDAR-based map building is future work.

**4. `nlp_goal_node`** (`nlp_goal_interface/`) is a terminal REPL that calls the Anthropic
API with two tools (`extract_coordinates`, `approach_object`) and publishes the result on
`/goal_pose`. It needs `ANTHROPIC_API_KEY` in the environment; the model comes from
`NLP_GOAL_MODEL`. Its `SCENE_OBJECTS` table (chair, prius) is **hand-mirrored from
`crazysim_default.sdf`** and silently goes stale if the world moves.

**5. `tf_transform_broadcast`** publishes `world → crazyflie_0/base_link` from `/cf_1/pose`
plus the static sensor frames. It is **not** required by the ballistic pipeline (the planner
and controller work in the world frame) but is kept for RViz and legacy consumers.

## Things that will bite you

- **Two namespaces for one robot.** `cf_0` is the **Gazebo** side (`/cf_0/lidar/points`,
  `/cf_0/imu`, `/cf_0/odom`, bridged by `ros_gz_bridge`); `cf_1` is the **crazyswarm2/cflib**
  side (`/cf_1/pose`, `/cf_1/cmd_vel_legacy`, `/cf_1/imu`), named by
  `ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml`. `hopcopter` subscribes to both
  and the mismatch is intentional — do not "normalize" it.
- **Absolute paths are hardcoded** to `/home/zweminhtetaung/CrazySim/data/...` in the CSV
  loggers (e.g. `hopcopter.py`, `ballistic_planner_node.cpp`). CSV logging breaks silently
  for any other checkout location.
- **Edit the model template, not the model.** The robot's SDF is generated at launch by
  `jinja_gen.py` from `models/crazyflie/model.sdf.jinja` into `/tmp/crazyflie_0.sdf`. Changes
  to `model.sdf` are discarded. The LiDAR's 15° downward tilt (`0.2618` rad, line ~235) is a
  deliberate tuning left over from the removed local planner — see `CHANGELOG.md` 2.4.0 before
  touching it.
- **`CHANGELOG.md` records why parameters have their values.** Consult it before retuning the
  ballistic planner or the hop controller; several defaults look arbitrary and are not.
- `data/` and `ros2_ws/*.txt` fill with timestamped run logs, and `data/**/*.csv` is
  gitignored. `ros2_ws/src/convert_pc2*.py`, `pc2tocustom`, and `tutorial_interfaces` are
  leftovers from a Livox-format experiment and are not part of the running pipeline.
