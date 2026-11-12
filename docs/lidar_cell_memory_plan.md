# Steadier LiDAR cell heights (Step 2): plan

Status: **planned, not started** (2026-11-11). Step 1 (`standable.lidar_steep_noise` 0.02 in
the planner and hopcopter) is done; see CHANGELOG.

## Goal

Make each cell of the LiDAR elevation map average more of its points, so its height stops
wobbling: cleaner step heights on the stairs and cleaner slopes. Target: a memory of about
**25 points** per cell (today about 5).

## Why the heights wobble

`lidar_elevation_mapper` fuses each new point into its cell with a Kalman update, but the cell's
variance has a floor, `MapperParams::min_cell_sigma` = 1 cm (`elevation_mapper.hpp`; ROS
parameter `height.min_cell_sigma`). With the floor reached, each new point gets a weight

    w = f² / (f² + σ_hit²)        f = min_cell_sigma
    σ_hit² = (0.02 sin el)² + (r · 0.002 cos el)² + 0.01²   (the mapper's own model)

so the cell is a moving average of its last N ≈ (2 − w) / w points. A typical point on the near
steps has σ_hit ≈ 1.5 cm (1.2–1.8 cm), which gives:

| floor `min_cell_sigma` | memory N (typical point) | far, skimming points | close, steep points |
|---|---|---|---|
| 10 mm (today) | ~5 | ~3 | ~7 |
| 4.9 mm | ~20 | ~10 | ~28 |
| **4.3 mm** | **~25** | ~12 | ~35 |
| 3.0 mm | ~50 | ~25 | ~70 |

A memory of 5 leaves each cell ±5–10 mm off, and a riser (two cells) ±7–14 mm. With the risers
only 7 mm over the old 60° line, that made 4 in 10 near risers read as a slope. Step 1 moved the
line; Step 2 narrows the spread.

## Expected effect (offline replay of the 2026-11-11 stairs run)

Near steps (x 1.0–2.2). "Bottom" = the map as the robot hopped at the foot of the stairs;
"climbed" = after flying mapper_check's stairs plan over it.

| memory | riser height spread, bottom / climbed | flat treads (±, 90%), bottom / climbed | near risers caught at 0.02 |
|---|---|---|---|
| ~5 (today) | ±7.4 / ±7.4 mm | ±3.5 / ±10.4 mm | 95% / 99% |
| ~20 | ±4.9 / ±3.9 mm | ±2.3 / ±5.1 mm | 97% / 99% |
| **~25** | **±4.6 / ±3.5 mm** | ±2.1 / ±4.5 mm | 97% / 99% |
| ~50 | ±3.8 / ±2.6 mm | ±1.6 / ±3.1 mm | 97% / 99% |

- The real map's spread is ~1.35× the replay's (±10 mm today), so expect about ±6 mm at memory 25.
- Slope (replay of the 2026-11-11 slope run): ramp error ±13 mm → ±6 mm; no false edges at any
  setting, with or without 0.02.

## The catch: wrong cells stop healing

Far steps seen only from below get erased (the cells beside each riser drift up or down its
face; see "Related" below). Today they **heal** once the robot gets close; with a long memory
they **don't**:

| after climbing past them | x 2.5 | x 2.8 | x 3.1 |
|---|---|---|---|
| memory ~5 (today) | 59% steep, Δz 0.175 | 57%, 0.175 | 59%, 0.174 |
| memory ~25 | 67%, 0.178 | **43%, 0.095** | **43%, 0.086** |

Cause, in `ElevationMapper::hit()` (`elevation_mapper.cpp`): a point more than 3σ below a cell's
height is ignored and counted in a "below-run"; 30 agreeing below-points in a row (std < 1.5 cm,
`kBelowRun`, `kBelowStd`) replace the height. But every point that fuses resets the run (unless a
neighbour explains it), and stray riser points keep fusing into a wrong cell. With a memory of 5
the wrong height drifts back within reach of the floor points by itself; with 25 it sits still,
so the floor points stay "too far below" and the run never completes.

**So Step 2 = the longer memory plus a healing fix, shipped together.**

## Plan

1. **Make the replay a repo tool first** (`hopcopter_perception`, e.g. `mapper_replay`), so every
   number here can be reproduced (see "Tools" below).
2. **Healing fix** in `ElevationMapper::hit()`, designed and measured in the replay before
   anything else. Candidates, in the order to try:
   - a. Don't reset the below-run when a point fuses; reset it only when the run itself stops
     agreeing (its std over `kBelowStd`). Stray points in between then can't erase the evidence.
   - b. Shorten the memory on surprise: when several points in a row land outside the cell's
     window on the same side, raise the cell's variance so the next points move it.
   - c. Count the run over a window (e.g. "most of the last 30 points are below and agree")
     instead of strictly in a row.
3. **Longer memory**: `MapperParams::min_cell_sigma` 0.01 → 0.0043 (memory ~25) in
   `elevation_mapper.hpp`, and update its comment (it says the floor exists for slopes; the
   slope replay showed no problem at 4.3 mm, and the comment should say so with the numbers).
   The node (`height.min_cell_sigma`) and `mapper_check` both take this default.
4. Docs: CHANGELOG (why 4.3 mm, the numbers above), CLAUDE.md if behaviour visible to the planner
   changes.

## Pass criteria

- Replay, far steps (x 2.5–3.1) after climbing: heal at least as well as today (≥ 55% steep,
  median Δz ≥ 0.17 m).
- Replay, near steps: ≥ 95% caught at 0.02, riser spread ≤ ±5 mm.
- Slope replay: no false edges, ramp error no worse than today.
- `ros2 run hopcopter_perception mapper_check`: all worlds pass.
- `ballistic_plan_check --lidar-maps <dir>` (maps from `mapper_check --map-out <dir>`): no new
  failures. Already failing before Step 2: the stairs LiDAR map (a far-step cell 10 cm high) and
  slanted_pads (no plan).
- Then a sim run on `stairs` and `slope_30deg`.

## Tools and data

- **Replay harness** (built 2026-11-11 in a session scratchpad, which may be gone). To rebuild:
  - mapper_check's raycaster (`World`, the 40 × 512 Mid-360, 2 cm range noise) cast from the
    Gazebo truth pose of a real run: `data/data_lio/lio_*.csv` columns `truth_x/y/z/roll/pitch/
    yaw` are the LiDAR frame at 10 Hz (roll ≈ 180°, upside down). Apply the truth attitude to
    the sensor's native −7…+52° rays.
  - Option: map through the FAST-LIO pose (`lio_*` columns) instead of the truth.
  - A copy of `elevation_mapper.cpp` that records each point's outcome in `hit()` (face
    ignored, attributed, raise, below, below-run, fused) to see which points set each cell.
  - "Then climb": append mapper_check's stairs flight after the replay.
  - Scores: per-riser Δz, % over the edge line, lower/upper cell error, flat-tread spread,
    the planner's steep mask (`Map2D5::steep_mask` rule) against the known map.
  - Last copy: `/tmp/claude-1000/-home-zweminhtetaung-CrazySim/1b115702-335e-47f2-b53c-bcdb4b19c79b/scratchpad/h/`.
- **Runs used:**
  - Stairs: `data/data_lio/lio_2026-11-11_18-25-03.csv`; mapper snapshots
    `data/data_elevation_map/snapshot_2026-11-11_18-24-31_*.json` (`elevation`, `raw`, `hits`,
    `known`).
  - Slope: `data/data_lio/lio_2026-11-11_18-20-39.csv`, snapshots `snapshot_2026-11-11_18-20-11_*`.

## Related (not in Step 2)

- **Stray riser points** (the far-step "ramp"): about 1 in 10 riser points escape the face
  filter. Most of them are the middle of three rings hitting a riser 1.7–1.9 m away, where the
  rings are 4.5–5 cm apart, under `face.min_dz` (5 cm). The filter also flags 60–75% of the real
  tread points beside a riser as face points. Far away, the escaped points outnumber the tread
  points 5:1 to 200:1, and the fusion lets them drag the cells beside the riser 6–12 cm up or
  down. Edge attribution then drops the tread points that would correct a sunk cell. It happens
  with zero range noise too. In the real run it also hit 31 of 136 near risers (at the
  staircase's sides, seen at an angle).
- The real map is 2–3× noisier on flat treads than the replay (±1.5 vs ±0.5 cm). Not explained;
  a run with `pose_source:=gazebo` would tell FAST-LIO timing from the Gazebo sensor.
