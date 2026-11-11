# How the LiDAR map grows and how the plan follows it

With a hand-made map, the pipeline is simple: one map, one goal, one plan, and the robot flies
it. With the LiDAR map (`hopcopter_perception`), the map keeps changing while the robot hops.
This document explains what changes, who notices, and when the waypoints the robot is following
actually change. The short version:

> The map is updated continuously, but the **plan is kept** as long as the hops the robot has
> not flown yet are still valid on the newest map. The planner only throws the plan away for a
> specific reason (below), and when it does, the robot finishes its current hop, hops in place,
> and a fresh plan starts from where it is standing.

So "frequent map updates" does **not** mean "frequent replanning". On a world like `low_wall`
the first plan usually survives to the goal unchanged.

---

## 1. The three loops and their rates

| Loop | Node | Rate | What it does |
|---|---|---|---|
| Mapping | `lidar_elevation_mapper` | each scan (10 Hz) in, map out at 2 Hz | Folds every scan into one persistent grid; publishes a snapshot on `/elevation_map` |
| Plan check | `ballistic_planner` | 10 Hz timer, acts only when a new map has arrived | Re-checks the hops not yet flown; replans only on a reason |
| Waypoint choice | `hopcopter` | once per hop, at the apex (state 3 → 1) | Picks which waypoint this hop lands on, from the current path |

They run independently and talk only through topics:

```
            /lidar/points (10 Hz)         /hopcopter/foot_contact (each touchdown)
                    │                                │
                    ▼                                ▼
          ┌────────────────────── lidar_elevation_mapper ──────────────────────┐
          │ persistent grid: height, variance, hits, upper bound per cell      │
          └────────────────────────────────┬───────────────────────────────────┘
                                           │ /elevation_map (2 Hz snapshot)
                       ┌───────────────────┴────────────────────┐
                       ▼                                        ▼
               ballistic_planner                           hopcopter
        (re-check plan, replan on a reason)     (caches map: touchdown prediction,
                       │                          clearance re-check)
                       │ /ballistic_trajectory (DELETE / ADD)   ▲
                       │ /ballistic_hop_plan                    │
                       └────────────────────────────────────────┘
                       ◀── /hopcopter/apex_decision (each apex: which waypoint, track/retry/hold/goal)
```

---

## 2. How the map is built and updated

Code: `ros2_ws/src/hopcopter_perception/src/elevation_mapper.cpp` (core) and
`lidar_elevation_mapper_node.cpp` (ROS wrapper).

### 2.1 One persistent grid, not a per-scan picture

The mapper owns a fixed grid (by default the known map's size, 0.1 m cells). It is created once
and **never cleared**. Every scan is merged into it and then discarded; raw points are not kept.
The published `/elevation_map` is a snapshot of that grid, so it contains everything seen since
the start of the run, not just what the LiDAR sees right now.

### 2.2 Placing a scan while the robot hops

Each scan is looked up in TF **at that scan's timestamp**: `world → camera_init` (fixed once by
`lio_anchor` before release) and `camera_init → body` (FAST-LIO's pose at that moment). So a scan
taken mid-flight, tilted, or during stance lands in the right world cells. The map is stable
because the pose is tracked, even though consecutive scans look completely different.

Mapping starts only once `lio_anchor` has published `world → camera_init`; until then scans
cannot be placed and nothing is published (`publish()` returns early while the grid has 0 scans).

### 2.3 What one scan does to the grid

For every point, in this order:

1. **Ray carving (upper bound).** The ray from the sensor to the point is traced through the
   grid. Over every cell it crosses, the terrain must be *below* the ray, so the cell's
   `upper_bound` is lowered to the ray's height there. A stored height that a ray passes clearly
   underneath cannot exist and is removed. Rays up to 30 m carve.
2. **Hit (height).** If the point is within 8 m, it updates the height of the cell it falls in,
   with a variance from range noise and attitude noise (far, shallow hits count for less):
   - empty cell → the hit becomes its height;
   - hit agrees with the cell (within 3σ) → Kalman-fused, the estimate tightens;
   - hit clearly **higher** → it replaces the height (a new obstacle top wins);
   - hit clearly **lower** → ignored (e.g. a wall's face under its top), but 30 consistent lower
     hits in a row replace the estimate (recovers from an outlier or a pose jump).

### 2.4 When a cell counts as "measured"

A height is published in the `elevation` layer only once it is **confirmed**: at least 2 hits
and a ray has passed over its top (so it is not just the side of something taller). Until
then the layer has NaN there, but `upper_bound` still says "no higher than this".

### 2.5 Foot contacts

Every touchdown hopcopter reports on `/hopcopter/foot_contact` sets that cell's height with near
certainty; LiDAR hits never overrule it. The **first** contact also fills a 0.6 m disk of level
ground around the foot, because the upside-down LiDAR cannot see directly under the robot. This
is why the robot must hop a few times before the first plan: without it, the start cell is unmeasured
and the planner refuses to plan.

### 2.6 What the planner sees in each cell

| Cell in `/elevation_map` | Planner treats it as |
|---|---|
| `elevation` finite (measured) | real terrain; a possible landing if flat and wide enough |
| `elevation` NaN, `upper_bound` finite | terrain **at the bound** for clearance (arcs may fly over it), **never a landing** |
| both NaN | obstacle (an infinitely tall column) |

Landings also stay 0.3 m from unseen cells and a little further from steep edges. So as the map
fills in, cells move from "obstacle" to "bounded" to "measured", and the set of places the robot
can land grows.

---

## 3. How the planner reacts to a new map

Code: `ros2_ws/src/ballistic_motion_planner/src/ballistic_planner_node.cpp`
(`planAndPublish`, `recheckPlan`, `replanStep`).

### 3.1 The first plan, and frontier plans

When `/goal_pose` arrives the planner plans once from the robot's position:

- **Goal already measured and standable** → normal A* plan to the goal.
- **Goal not seen yet** → a **frontier plan**: A* to a measured, standable cell that the robot
  can reach and that is nearest the goal. The log says
  `Goal ... not on measured ground yet: frontier plan to (x, y)`.
- **Nothing reachable** (e.g. the goal was sent before warm-up) → no plan; it retries as the map
  grows (`waiting for the map`).

It remembers the published plan: its waypoints, each hop, and the ground height under each
waypoint at planning time.

### 3.2 On every new map: re-check, don't replan

Each time a new `/elevation_map` arrives, the planner re-checks **only the hops the robot has not
flown yet** (it knows where the robot is in the plan from `/hopcopter/apex_decision`). For each
remaining hop it asks:

1. Is the landing waypoint still standable?
2. Has the ground under the landing or takeoff waypoint moved more than 5 cm since planning?
3. Does the planned arc still clear the newly seen terrain by at least the bottom clearance
   (0.2 m, minus 5 cm tolerance)?

If all pass, **nothing happens**. The robot keeps flying the same waypoints. This is the common
case: new map areas usually confirm what the plan assumed, or lie off the path.

### 3.3 Reasons that do trigger a replan

| Reason (as logged) | Meaning |
|---|---|
| `waypoint N ... is no longer standable` | A landing turned out to be too narrow, steep, or next to unseen ground |
| `waypoint N's ground moved ±x m` | Its measured height changed by more than 5 cm |
| `the arc into waypoint N clears the terrain by x m now` | Newly seen terrain is in the way of a planned hop |
| `the goal is measured now` | A frontier plan is running and the real goal just became standable |
| `reached the frontier` | The robot reached the end of a frontier plan; plan the next leg |
| `the robot gave up on the path (hold)` | hopcopter missed a waypoint 3 times and is holding |
| `no plan yet` | The last attempt failed; try again on the newer map |

Replans are rate-limited: at least 5 s since the last plan (`replan.min_interval`), longer after
repeated failures (up to 30 s).

### 3.4 What a replan looks like on the wire

1. Planner publishes a **DELETE alone** on `/ballistic_trajectory` and logs
   `Replanning (<reason>): path deleted, waiting for the robot to hop in place`.
2. hopcopter keeps its current hop (it cannot change a flight in the air). If no new path follows
   within 0.5 s, it drops the path and **hops in place**, and reports `idle`/`hold` on
   `/hopcopter/apex_decision`.
3. Once the robot is hopping in place (≥ 1 s after the DELETE, or after 4 s regardless), the
   planner plans **from the spot it is hopping on**, on the newest map, and publishes the new
   HopPlan + path.
4. hopcopter adopts the new path at its next apex (section 4).

Every plan and replan is a row in `data/data_ballistic_planner/replans_<stamp>.csv`
(reason, outcome, start, target, number of waypoints), and every plan writes its own
`vis_planned_path/planned_path_<stamp>.png`. If that CSV only has the first plan, the robot
flew the original plan the whole way.

---

## 4. How hopcopter picks waypoints

Code: `ros2_ws/src/hopping_robot/hopping_robot/hopcopter.py` and `JumpLib/waypoint_tracker.py`.

hopcopter never plans; it follows whatever path it was last given. The decision happens **once
per hop, at the apex**, because that is the only time it can still choose where the next hop goes:

1. Predict where the foot will land (P), by marching the fall over the **latest cached map**.
2. Ask the tracker:
   - **track**: P is within 0.8 m of the waypoint this hop aimed at → aim at the next one;
   - **retry**: missed → aim at the same one again; after 2 retries → **hold** (hop in place;
     this is what triggers the planner's `hold` replan);
   - **goal**: on the last waypoint → hop in place there;
   - **idle**: no path → hop in place.
3. Solve the takeoff for that waypoint and command the landing attitude.

When a **new path** arrives, the tracker does not jump mid-flight. At the next apex it syncs to
the waypoint of the new path nearest P and continues from there. Because a replan starts from
the spot the robot is hopping in place on, that is normally waypoint 0 of the new path.

hopcopter also uses each new map directly, without any replan: the touchdown prediction and its
clearance re-check (it can raise a hop's apex if newly seen terrain is in the way) always use the
latest map.

---

## 5. A typical run, step by step

1. **Play.** The robot hangs while FAST-LIO initializes; `lio_anchor` fixes the world frame and
   releases it. Mapping starts.
2. **Warm-up hops.** The first touchdown fills the start disk; the LiDAR fills in the area ahead
   (mostly 1–8 m away, since the sensor looks down and outward).
3. **Goal sent.** If the goal is measured → full plan. If not → frontier plan toward it.
4. **Flying.** Every 0.5 s a new map arrives. The planner re-checks the remaining hops; usually
   they pass and nothing changes. hopcopter picks the next waypoint at each apex.
5. **Something changes** (the goal comes into view, a landing turns out bad, the robot gives up):
   DELETE → robot hops in place → new plan from there → robot follows it.
6. **Goal reached** (`goal` decision on a non-frontier plan) → the planner logs
   `Goal (...) reached` and stops checking.

---

## 6. How to see this happening

- **Planner terminal**: `Replan pending (<reason>)` (a reason exists, waiting for the minimum
  interval), `Replanning (<reason>)`, `Frontier plan`, `Goal ... reached`.
- **hopcopter terminal**: `New path: ...` for each adopted path, and
  `Path deleted and no new path followed; hopping in place`.
- **RViz**: add a GridMap display (Add → By display type → `grid_map_rviz_plugin/GridMap`,
  topic `/elevation_map`, layer `elevation`) and a Marker on `/ballistic_trajectory`. You can watch
  the map fill in and the path get replaced.
- **Files** in `data/`:
  - `data_ballistic_planner/replans_*.csv`: every plan and why;
  - `data_ballistic_planner/vis_planned_path/planned_path_*.png`: each plan on the map it was made on;
  - `data_ballistic_planner/hop_tracking/apex_decisions_*.csv`: each apex's decision;
  - `data_elevation_map/score_*.csv`: map coverage and error against the known map.

## 7. Parameters that shape this behaviour

| Parameter | Node | Default | Effect |
|---|---|---|---|
| `publish_rate` | mapper | 2.0 Hz | How often the planner/hopcopter get a new map |
| `height.max_update_range` | mapper | 8.0 m | Hits farther than this don't set heights |
| `height.min_hits` | mapper | 2 | Hits before a cell counts as measured |
| `start_disk_radius` | mapper | 0.6 m | Area assumed level at the first touchdown |
| `replan.enabled` | planner | true | `false`: plan once per goal, never replan |
| `replan.min_interval` | planner | 5.0 s | Minimum time between plans |
| `replan.height_tolerance` | planner | 0.05 m | Ground change / clearance slack before a replan |
| `replan.hold_wait` / `replan.hold_timeout` | planner | 1.0 / 4.0 s | How long to wait for the in-place hop before replanning |

The design and the reasons behind these values are in `docs/lidar_mapping_plan.md` and
`CHANGELOG.md`.
