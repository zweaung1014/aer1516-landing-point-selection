# Changelog

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
