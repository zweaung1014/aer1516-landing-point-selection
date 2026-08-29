# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Perception-informed autonomous navigation for **Hopcopter**, a Crazyflie-based monopedal
hopping robot, simulated in Gazebo Harmonic and driven by a ROS 2 Humble pipeline. A goal
pose (typed as a ROS command or spoken in natural language) is turned into a global path by
an OMPL RRT* node, each waypoint is refined into a safe landing point by a scoring local
planner, and the `hopcopter` node flies the robot there by commanding roll/pitch/yaw/thrust
through a hop cycle. **The robot HOPS** — it is airborne between waypoints and the
controller works in phases, so almost every timing question ("when is a waypoint chosen?")
resolves to "at a particular jumping state."

### Where the work is heading

The shipped pipeline plans on a **2D occupancy grid** built by thresholding the LiDAR cloud.
The intended direction is to replace that with **2.5D ballistic planning** — a grid of
elevations where each move is a physically feasible parabolic hop. `hopcopter-ballistic-planning/`
is a working standalone Python implementation of that algorithm and **exists in this repo as
reference context only**: it is not built, launched, or imported by the simulation, and
nothing in `ros2_ws/` depends on it. Read its own `CLAUDE.md` (thorough, 579 lines) and
`docs/planner.md` before proposing an integration. Do not "fix", refactor, or wire it up
unless explicitly asked.

## Repository layout

Everything is one flat git repo — there are **no submodules**. `crazyflie-firmware/`,
`crazyflie-lib-python/`, `crazyflie-clients-python/`, and `ros2_ws/src/crazyswarm2/` are
vendored upstream trees, checked in and locally modified (notably the Gazebo model and world
under `crazyflie-firmware/tools/crazyflie-simulation/`). The project's own code is:

| Path | What it is |
|---|---|
| `ros2_ws/src/rrt_star_planner/` | C++ / OMPL global planner (`rrt_star`) |
| `ros2_ws/src/local_planning/` | C++ / PCL landing-point scorer (`local_planner`) |
| `ros2_ws/src/hopping_robot/` | Python hop controller (`hopcopter`) + `JumpLib`/`RisLib` |
| `ros2_ws/src/nlp_goal_interface/` | Python Claude-powered goal parser (`nlp_goal_node`) |
| `ros2_ws/src/tf_transform_broadcast/` | Python TF broadcaster |
| `ros2_ws/src/tutorial_interfaces/`, `pc2tocustom/` | Livox custom point messages (legacy path, not in the live pipeline) |
| `crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/` | SITL launch scripts, world, and the robot model **jinja template** |
| `hopcopter-ballistic-planning/` | Reference-only 2.5D ballistic planner (Python, standalone) |

## Build and run

```bash
cd ~/CrazySim/ros2_ws
source /opt/ros/humble/setup.bash
colcon build                                          # full workspace
colcon build --packages-select rrt_star_planner       # one package (C++ changes)
source install/setup.bash                             # required in EVERY terminal
```

Python packages (`hopping_robot`, `nlp_goal_interface`, `tf_transform_broadcast`) are
`ament_python`; changes still need a `colcon build` + re-source because the nodes run from
`install/`.

Running the simulation takes **nine terminals**, each sourced as above; `README.md` has the
exact commands in order (Gazebo SITL → cflib server → LiDAR/IMU/odom `ros_gz_bridge` →
TF → `rrt_star` → `local_planner` → `hopcopter`). Nothing starts moving until **Play** is
pressed in the Gazebo GUI. A goal then goes in either by publishing `/goal_pose` directly or
by running `nlp_goal_node`.

There is **no test suite for the simulation**. The `test/` directories in the Python packages
contain only the stock ament flake8/pep257/copyright linters. Verification is done by running
the sim and inspecting the CSVs it writes (below).

### Analysis scripts (run from the repo root, not the workspace)

```bash
python graph_results.py              # latest data/data_rrt_star/ path, XY plot
python visualize_local_planner.py -l 5   # candidate scores from one local-planner decision
```

Both default to the newest CSV in the relevant `data/` subdirectory. Note their axis
convention: **+x is drawn vertically up, +y horizontally left.**

## Architecture

```
LiDAR PointCloud2 ──▶ rrt_star ──▶ local_planner ──▶ hopcopter ──▶ /cf_1/cmd_vel_legacy
   /cf_0/lidar/points    ▲            (per-hop            (roll/pitch/yaw/thrust)
                         │             refinement)
                    /goal_pose  ← ros2 topic pub, or nlp_goal_node
```

**1. `rrt_star`** (`rrt_star_planner/src/rrt_star.cpp`) accumulates the LiDAR cloud into a
flat `GridMap` (free/occupied, `map.resolution` 0.05 m over `map.min_x…max_y`), clusters
obstacles by 8-connected flood fill, and runs OMPL RRT* over a 2D `RealVectorStateSpace`.
It publishes the path as a `Marker` SPHERE_LIST on `/ompl_rrt_star_trajectory` (**waypoints
travel as marker points, not a `nav_msgs/Path`**) plus `/rrt_star_grid` and
`/trajectory_start_position`. Ground rejection is two-sided and was tuned repeatedly — a
robot-relative `robot.leg_height` threshold *and* an absolute `map.min_obstacle_z` floor, so
low traversable surfaces are not read as walls when the robot dips, plus a
`map.max_ground_range` cutoff for far ground returns produced by the tilted LiDAR.

**2. `hopcopter`** (`hopping_robot/hopping_robot/hopcopter.py`) owns the waypoint queue. It
consumes the marker, holds `self.waypoint_list`, and pops the next target once the current
one is inside `current_goal_tolerance`, holding the last position when the list empties.
`_load_next_waypoint` **discards the waypoint's z and forces `desired_z = 0.8`** — one of the
places where the pipeline's 2D-ness is baked in, and a direct obstacle to 2.5D planning. Control runs on a 100 Hz timer (`RPYT_commands`)
around `JumpingStateTrackerOnboard` (`JumpLib/jumping_tools.py`), whose `jumping_state` is
the clock for the whole system:

- **1 — falling/ballistic:** unpowered, attitude tracked from the planned arc (`LinearJumpingController`).
- **2 — stance:** foot down, inverted-pendulum swing, unpowered.
- **3 — climbing:** powered thrust; a new hop begins here.

At the *start of state 3* `hopcopter` publishes `/jumping_state` (Int8) and, once per cycle,
`/trajectory_queue_state` (PoseArray: `poses[0]` = current goal, `poses[1]` = next waypoint).
It also emits `/visited_waypoint` for every waypoint it targets. After the apex (3 → 1) it
predicts the landing point (`LandingStateEstimator`), plans the ballistic arc, and derives
the roll/pitch that steer it.

**3. `local_planner`** (`local_planning/src/local_planner.cpp`) is triggered by that
handshake — it wakes on the state-3 edge, reads the queue state, and has only the apex
window (~300–400 ms) to answer. It lays a grid of candidates (`candidate_grid_radius` /
`candidate_grid_step`) around the next waypoint and scores each on the four criteria in
`local_planner_description`: surface slope (PCA normal), obstacle proximity, hop distance,
and edge proximity. The winner goes back as `/local_planner/adjusted_waypoint`
(a `PointStamped` that smuggles the **waypoint index in `header.stamp.sec`** so `hopcopter`
can patch that entry in place — the field is not a timestamp), with debug geometry on
`/local_planner/markers`. Note the shipped weights:
`weight_obstacle` 1.0 and `weight_distance` 0.1 are live; `weight_slope` and `weight_edge`
default to **0.0**, so those two terms are computed and logged but contribute nothing unless
turned on.

**4. `nlp_goal_node`** (`nlp_goal_interface/`) is a terminal REPL that calls the Anthropic
API with two tools (`extract_coordinates`, `approach_object`) and publishes the result on
`/goal_pose`. It needs `ANTHROPIC_API_KEY` in the environment; the model comes from
`NLP_GOAL_MODEL`. Its `SCENE_OBJECTS` table (chair, prius) is **hand-mirrored from
`crazysim_default.sdf`** and silently goes stale if the world moves.

**5. `tf_transform_broadcast`** publishes `world → crazyflie_0/base_link` from `/cf_1/pose`
plus the static sensor frames. Both planner nodes transform clouds through TF, so nothing
downstream works if this node is not running.

## Things that will bite you

- **Two namespaces for one robot.** `cf_0` is the **Gazebo** side (`/cf_0/lidar/points`,
  `/cf_0/imu`, `/cf_0/odom`, bridged by `ros_gz_bridge`); `cf_1` is the **crazyswarm2/cflib**
  side (`/cf_1/pose`, `/cf_1/cmd_vel_legacy`, `/cf_1/imu`), named by
  `ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml`. `hopcopter` subscribes to both
  and the mismatch is intentional — do not "normalize" it.
- **Absolute paths are hardcoded** to `/home/zweminhtetaung/CrazySim/data/...` in
  `rrt_star.cpp:675,706`, `local_planner.cpp:270`, and `hopcopter.py:256`. CSV logging breaks
  silently for any other checkout location.
- **Edit the model template, not the model.** The robot's SDF is generated at launch by
  `jinja_gen.py` from `models/crazyflie/model.sdf.jinja` into `/tmp/crazyflie_0.sdf`. Changes
  to `model.sdf` are discarded. The LiDAR's 15° downward tilt (`0.2618` rad, line ~235) is a
  deliberate tuning that gives the local planner sight of landing zones ahead — see
  `CHANGELOG.md` 2.4.0 before touching it.
- **`CHANGELOG.md` records why parameters have their values** (ground-filter fix, voxel
  downsampling to hold edge detection at 3–7 ms, LiDAR tilt). Consult it before retuning
  anything in the two planners; several defaults look arbitrary and are not.
- The local planner's work must fit the apex window; an algorithmic change that adds cost
  there is a real constraint, not a micro-optimization.
- `data/` and `ros2_ws/*.txt` fill with timestamped run logs, and `data/**/*.csv` is
  gitignored. `ros2_ws/src/convert_pc2*.py`, `pc2tocustom`, and `tutorial_interfaces` are
  leftovers from a Livox-format experiment and are not part of the running pipeline.
