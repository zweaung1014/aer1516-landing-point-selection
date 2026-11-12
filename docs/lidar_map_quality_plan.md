# Plan: a clean LiDAR elevation map

Written 2026-11-11, after the 2026-11-10 LiDAR runs (stairs and slope_30deg found no plan, maze
stalled beside a pole). Two problems, fixed in this order.

## Status (2026-11-11)

- **Fix 1 done and checked in the sim.** Leftovers killed; `lio_anchor` refuses to release with a
  duplicated stream; the mapper warns above 11 scans/s. Four clean runs: FAST-LIO RMSE
  1.4–1.7 cm, no duplicate stamps, 9.4–9.8 scans/s, map median 0.6–0.9 cm. RTF was 0.84–0.91
  (dips to 0.67) with the GUI; low_wall fell at 0.67 and cleared the wall when the sim was not
  slow. Open: the body's roll/pitch jiggles in flight.
- **Fix 2 done offline, not yet flown.** Face points, edge attribution, face-only cells
  published unseen (CHANGELOG, Changed). Planner: `lidar_steep_noise` 0, the 0.1 m edge margin
  only at drops ≥ 0.3 m (rims seen from above still read a cell wide; see below). On
  `mapper_check`'s planning-time maps the stairs plan climbs the stairs instead of going around.
  - Open: on the aligned lattice the frontier cell on tread 6 reads 10 cm high (seen at 10°).
  - Decide: the stairs' 0.18 m risers are 7 mm over the 60° threshold, so one mapped 1 cm low is
    no edge and a landing can come 5 cm past a riser; `standable.lidar_steep_noise` 0.02 fixes it.
  - Changed from the plan: step 4 left the edge margin at big drops (a rim seen from above is
    uncertain by a cell, and the "wall widened by face hits" case lands on the wall without it).

## Fix 1: duplicated sensor streams (leftover processes)

**What went wrong.** `gz_truth_tf` and `lidar_sim_adapter` from an 18:48 launch never exited, so
every later launch had two adapters. FAST-LIO received every scan and IMU sample twice, out of
order, and cleared its buffers ("loop back, clear buffer", `laserMapping.cpp`), so its pose
jumped at touchdowns and diverged. A wrong pose draws correct scans in the wrong place.
Signatures: 14–18 scans/s at the mapper (should be ~10), about half the `data/data_lio/` rows with
a duplicate stamp, 0.3–0.8 s gaps between FAST-LIO outputs, RTF 0.63–0.92. Whenever the pose
was right, the map's median error was 6–10 mm.

**Goal:** every run has exactly one copy of each sensor stream.

1. **Clean up.** Kill the leftovers; `pgrep -af hopcopter_perception` must print nothing before a
   launch.
2. **Guard in `lio_anchor`.** Before releasing the robot, count the publishers on `/lio/points`,
   `/lio/imu` and `/lidar/points`. If any has more than one, do not release, and log an error
   naming the topic.
3. **Warning in `lidar_elevation_mapper`.** Warn when it receives more than about 11 scans/s (the
   LiDAR runs at 10 Hz).
4. **Docs.** The `pgrep` check in the README's LiDAR run steps.

**Check:** one run each of low_wall, stairs, slope_30deg and maze with the current code:
- FAST-LIO position RMSE ≤ 3 cm;
- no duplicate stamps in `data/data_lio/`;
- ~10 scans/s at the mapper;
- RTF ≥ ~0.95 (if the Gazebo GUI keeps it lower, run headless and watch in RViz).

These runs are also the "before" picture for Fix 2.

## Fix 2: the smear (vertical faces one cell too fat)

**What goes wrong.** The mapper keeps one height per 0.1 m cell, and a clearly higher hit replaces
it ("highest wins", so a cell holding part of a box reads as the box). A vertical face (riser, pole
side, wall) usually sits on a cell border. The 2 cm range noise scatters its points a few cm to
both sides, so some land in the floor cell in front, which then takes the height of the face's
highest point. Nothing corrects it: lower floor hits are ignored, and a ray low enough to clear
the cell runs into the face. Every face the LiDAR looks at gains a cell in front of it.

From the 2026-11-10 stairs run, 1 s in, pose still exact (y = −2.05, `|` = riser):

```
x     0.85  0.95 | 1.05  1.15  1.25 | 1.35  1.45  1.55 | 1.65
true  0.00  0.00 | 0.18  0.18  0.18 | 0.36  0.36  0.36 | 0.54
map   0.01  0.13 | 0.18  0.16  0.37 | 0.37  0.34  0.55 | 0.51
```

Maze, flat floor at 0 m: 0.79–0.97 m beside the pole at (3.0, 0.0), 1.5–1.9 m along both
corridor walls, 0.4–0.5 m in front of the 0.5 m wall.

The planner's LiDAR-only rules were added (2026-11-07) to cope with this, and they cost the
stairs every landing:
- `standable.lidar_steep_noise` 0.08 makes a 9.3 cm jump an edge instead of 17.3 cm (60°); on the
  slope (5.8 cm per cell) noise then makes false edges;
- `standable.lidar_edge_margin` 0.1 keeps landings 0.15 m from edges, which removes the middle
  cell of every 0.3 m tread, even on a perfect map.

**Goal:** points on vertical faces set no cell's height; the 60° edge rule stays as on the known
map.

1. **Face detection** (`elevation_mapper.cpp`, `integrate_scan`). Per scan, in the world frame:
   - bin the points into 3 cm horizontal buckets;
   - a point is a **face point** if the same scan has another point within 3 cm sideways and at
     least 5 cm lower (at most ~0.5 m lower);
   - face points still carve their rays but are skipped for heights;
   - parameters `face_radius` 0.03, `face_min_drop` 0.05, and an on/off switch.

   A cell that only faces describe (a tall pole or wall, a tread above the sensor) gets no
   height. With no ray over it, it is an obstacle, as today; landings keep 0.3 m from it. A riser
   inside a cell still gives that cell the tread's height from the tread-top points.
2. **Logging.** Face points per scan in the mapper's log and score CSV.
3. **`mapper_check`.** It already simulates 2 cm range noise but lets errors through within a cell
   of an edge. New checks:
   - every floor cell beside a face within 5 cm of the truth;
   - every stair tread cell within 2 cm;
   - the slope as accurate as now (no slope points flagged);
   - poles and tall walls still obstacles;
   - `--shift` moves every world 3 cm off the grid (risers inside cells); all checks pass there too.

   All nine worlds, with and without the shift.
4. **Remove the coping rules** (`ballistic_planner_node.cpp`): `standable.lidar_steep_noise` → 0,
   `standable.lidar_edge_margin` → 0. `standable.unseen_margin` (0.3 m) stays.
   `ballistic_plan_check` passes; `ballistic_parity_test` stays byte-identical.
5. **Docs.** `CHANGELOG.md` (why each value), and the map-contract lines in `CLAUDE.md`.

**Check in the sim:** stairs, slope_30deg (with the look-out hop), maze, low_wall, then cliff_gap
and lilypads:
- map error beside faces within 5 cm;
- stairs and slope find plans;
- the robot reaches each goal.

The stairs will plan a few steps at a time, since the top is out of view from the bottom.

## Left for after

**Margins at big drops.** The robot fell twice landing near the maze wall and the cliff rim. Decide
whether those edges need a margin, and how big, once the clean map has flown. Also whether the
maze still stalls.
