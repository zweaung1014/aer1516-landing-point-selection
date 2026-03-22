# Changelog

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
