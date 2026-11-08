// The known 2.5D maps of each scenario (header-only, no ROS), shared by
// elevation_map_publisher_node, the planner's offline ballistic_plan_check and
// hopcopter_perception's offline mapper_check.
//
// Scenarios mirror the maps in hopcopter-ballistic-planning/maps/. "low_wall"
// and "flat" are painted in world coordinates. The others ("stairs",
// "stairs_with_curb", "slope_30deg", "cliff_gap", "maze") are painted in their
// reference file's frame and placed so its START sits at world (0, 0), where
// the robot spawns: origin = -START. "lilypads" and "slanted_pads" have no
// reference file and are painted in world coordinates. Each has a Gazebo twin,
// worlds/crazysim_<scenario>.sdf. stairs, stairs_with_curb and cliff_gap are
// edited from the reference files so the robot's planner (robot_params()) can
// plan them; see CHANGELOG.md.
#pragma once

#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace elevation_map_publisher
{

constexpr double kObstacle = std::numeric_limits<double>::infinity();

// Geometry defaults and the reference goal, in world coordinates.
struct Scenario
{
  double size_x, size_y, origin_x, origin_y;
  double goal_x, goal_y;
};

inline const std::map<std::string, Scenario> & scenarios()
{
  static const std::map<std::string, Scenario> kScenarios = {
    {"flat", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
    {"low_wall", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
    {"stairs", {5.0, 5.0, -0.5, -4.5, 4.0, -2.0}},
    {"stairs_with_curb", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
    {"slope_30deg", {5.0, 5.0, -0.5, -4.5, 4.25, -2.0}},
    {"cliff_gap", {15.0, 5.0, -0.5, -2.5, 9.5, 0.5}},
    {"maze", {15.0, 5.0, -0.5, -2.5, 14.0, 0.0}},
    {"lilypads", {16.0, 6.0, -2.0, -3.0, 11.75, 0.0}},
    {"slanted_pads", {19.0, 6.0, -2.0, -3.0, 15.85, 0.0}},
  };
  return kScenarios;
}

namespace detail
{
// paint_region's center-inclusion rule: lo <= v < hi, NaN unbounded.
inline bool in(double v, double lo, double hi)
{
  const double eps = 1e-9;
  return (std::isnan(lo) || v >= lo - eps) && (std::isnan(hi) || v < hi - eps);
}
}  // namespace detail

// Terrain at the cell center (x, y) in the reference file's frame;
// kObstacle for an infinitely tall column.
inline double reference_z(const std::string & scenario, double x, double y)
{
  using detail::in;
  const double kNone = std::numeric_limits<double>::quiet_NaN();  // unbounded side
  const double slope = std::tan(30.0 * M_PI / 180.0);
  if (scenario == "stairs") {
    // maps/stairs.py resized to human stairs, about half the robot's 0.37 m
    // standing height per riser: eight 0.18 m risers on 0.3 m treads from
    // x 1.5 (was 0.6 on 0.3 from x 2.0, five steps). The top is 1.44 m from
    // x 3.6 to the map edge.
    if (!in(y, 1.0, 4.0) || x < 1.5) {
      return 0.0;
    }
    return 0.18 * std::min(8, 1 + static_cast<int>(std::floor((x - 1.5 + 1e-9) / 0.3)));
  }
  if (scenario == "stairs_with_curb") {
    // maps/stairs_with_curb.py with 0.15 m curbs on 1.0 m treads (was 0.6 m on
    // 0.4 m from x 2.0). Full width: no bypass.
    const double curb = 0.15;
    if (in(x, 1.2, 1.4)) {return 0.40 + curb;}
    if (in(x, 1.4, 2.4)) {return 0.40;}
    if (in(x, 2.4, 2.6)) {return 0.80 + curb;}
    if (in(x, 2.6, 3.6)) {return 0.80;}
    if (in(x, 3.6, 3.8)) {return 1.20 + curb;}
    if (in(x, 3.8, kNone)) {return 1.20;}
    return 0.0;
  }
  if (scenario == "slope_30deg") {
    // maps/slope_30deg.py, unchanged: 30 deg from x 1.0, platform from 4.5.
    if (!in(y, 1.0, 4.0) || x < 1.0) {
      return 0.0;
    }
    return slope * (std::min(x, 4.5) - 1.0);
  }
  if (scenario == "cliff_gap") {
    // maps/cliff_gap.py with a 1.0 m gap (was 1.5 m: x 6.5-8.0).
    if (!in(y, 1.0, 4.0) || x < 2.0) {
      return 0.0;
    }
    const double platform = 1.80, x_slope_top = 2.0 + 3.118;
    if (x <= x_slope_top) {return slope * (x - 2.0);}
    if (x <= 6.5) {return platform;}
    if (x < 7.5) {return 0.0;}
    if (x <= 11.5) {return platform;}
    if (x <= 11.8) {return platform - 0.6;}
    if (x <= 12.1) {return platform - 1.2;}
    return 0.0;
  }
  if (scenario == "maze") {
    // maps/maze.py: the 17 scattered poles, a 3 m corridor (y 1.0-4.0) between
    // two infinitely tall side walls (y 0.8-1.0 and 4.0-4.2), and the 0.5 m
    // wall at x 7.35-7.65 across the full width. Each pole is the 2 x 2 cells
    // whose centers lie in its 0.2 m footprint (set_obstacle_region rounds out
    // to 3-4 cells).
    static const double poles[][2] = {
      {2.0, 1.5}, {2.5, 3.5}, {3.5, 2.5}, {4.0, 4.2}, {4.5, 0.8}, {5.5, 3.0}, {5.8, 1.5},
      {6.5, 4.0}, {6.7, 2.2}, {8.5, 2.8}, {8.7, 4.2}, {9.5, 1.5}, {10.0, 3.5}, {10.5, 2.0},
      {11.5, 4.0}, {12.0, 1.2}, {12.5, 3.0}};
    if (in(y, 0.8, 1.0) || in(y, 4.0, 4.2)) {
      return kObstacle;  // corridor side walls
    }
    for (const auto & p : poles) {
      if (in(x, p[0] - 0.1, p[0] + 0.1) && in(y, p[1] - 0.1, p[1] + 0.1)) {
        return kObstacle;
      }
    }
    return in(x, 7.35, 7.65) ? 0.5 : 0.0;
  }
  if (scenario == "lilypads") {
    // No reference file: painted in world coordinates (world = reference +
    // the default origin (-2, -3)), the robot starting at world (0, 0). Two
    // 2.0 m platforms 10 m apart (start x -1.5 to 1.0, goal x 11.0 to 12.5)
    // and 20 cylindrical pads between them, 1.8-2.2 m tall, r 0.35-0.75 m.
    // Everything else is ground at 0: a 2 m pit, deliberately finite (an
    // OBSTACLE there would forbid flying over it too). A cell is a pad if its
    // center lies strictly inside the pad's circle; no cell center sits on a
    // rim at these radii.
    const double wx = x - 2.0, wy = y - 3.0;
    if (in(wx, -1.5, 1.0) && in(wy, -2.0, 2.0)) {return 2.0;}   // start platform
    if (in(wx, 11.0, 12.5) && in(wy, -1.5, 1.5)) {return 2.0;}  // goal platform
    // {x, y, radius, top z}, rows from the start platform toward the goal.
    static const double pads[][4] = {
      {2.0, 0.9, 0.60, 1.8},
      {3.4, 1.6, 0.40, 2.0}, {3.5, 0.2, 0.35, 1.9}, {3.3, -1.3, 0.75, 2.2},
      {4.9, 1.7, 0.40, 2.1}, {4.9, 0.4, 0.70, 2.0}, {5.0, -1.3, 0.40, 1.9},
      {6.4, 1.8, 0.40, 1.9}, {6.4, 0.6, 0.40, 2.0}, {6.4, -0.6, 0.40, 2.1}, {6.4, -1.8, 0.40, 2.0},
      {7.8, 1.8, 0.40, 2.0}, {7.8, 0.7, 0.35, 1.9}, {7.8, -0.5, 0.40, 2.0}, {7.8, -1.7, 0.40, 2.1},
      {9.1, 1.2, 0.40, 2.1}, {9.1, 0.0, 0.35, 2.0}, {9.1, -1.2, 0.40, 1.9},
      {10.2, 0.6, 0.35, 2.0}, {10.2, -0.6, 0.35, 2.1}};
    for (const auto & p : pads) {
      const double dx = wx - p[0], dy = wy - p[1];
      if (dx * dx + dy * dy < p[2] * p[2]) {
        return p[3];
      }
    }
    return 0.0;
  }
  if (scenario == "slanted_pads") {
    // No reference file: painted in world coordinates (world = reference +
    // the default origin (-2, -3)), the robot starting at world (0, 0). Two
    // 2.0 m platforms (start x -1.5 to 1.0, goal x 15.1 to 16.6) and seven
    // 1 x 1 m sloped pads between them in two columns that face each other:
    // each column pad is low on its inner edge (toward y 0), the last pad C
    // low on its -x edge. Every top stays within 2.0 +- 0.2 m. Consecutive
    // pads are 1.9 m apart in x, so two pads of one column are 2.8 m apart
    // edge to edge, past the longest hop (~2.4 m): the plan has to zigzag.
    // Everything else is ground at 0, a finite 2 m pit as in lilypads. Edges
    // are multiples of 0.1, so no cell center sits on one. The Gazebo twin
    // (crazysim_slanted_pads.sdf) builds each top as a tilted slab whose top
    // face is this plane.
    const double wx = x - 2.0, wy = y - 3.0;
    if (in(wx, -1.5, 1.0) && in(wy, -2.0, 2.0)) {return 2.0;}   // start platform
    if (in(wx, 15.1, 16.6) && in(wy, -1.5, 1.5)) {return 2.0;}  // goal platform
    // {x_lo, x_hi, y_lo, y_hi, mid z, slope deg, axis (0 x, 1 y), sign}:
    // z = mid + sign * tan(slope) * (coordinate along axis - pad center).
    static const double pads[][8] = {
      {1.8, 2.8, 0.3, 1.3, 2.00, 15.0, 1, +1},     // L1
      {3.7, 4.7, -1.3, -0.3, 2.05, 12.0, 1, -1},   // R1
      {5.6, 6.6, 0.3, 1.3, 1.98, 18.0, 1, +1},     // L2
      {7.5, 8.5, -1.3, -0.3, 2.00, 20.0, 1, -1},   // R2
      {9.4, 10.4, 0.3, 1.3, 2.03, 14.0, 1, +1},    // L3
      {11.3, 12.3, -1.3, -0.3, 1.97, 16.0, 1, -1}, // R3
      {13.2, 14.2, -0.5, 0.5, 2.00, 15.0, 0, +1}};  // C
    for (const auto & p : pads) {
      if (in(wx, p[0], p[1]) && in(wy, p[2], p[3])) {
        const double u = p[6] == 0 ? wx - 0.5 * (p[0] + p[1]) : wy - 0.5 * (p[2] + p[3]);
        return p[4] + p[7] * std::tan(p[5] * M_PI / 180.0) * u;
      }
    }
    return 0.0;
  }
  throw std::invalid_argument("no reference map for scenario '" + scenario + "'");
}

// The low wall, in world coordinates (y unbounded: full map width).
struct LowWall
{
  double height = 0.4, x_min = 2.5, x_max = 2.7;
};

// The terrain the node publishes at the cell centre (x, y), world coordinates,
// before the obstacle threshold: kObstacle for an infinitely tall column.
inline double world_z(
  const std::string & scenario, double x, double y, double default_z = 0.0,
  const LowWall & wall = LowWall{})
{
  const auto it = scenarios().find(scenario);
  if (it == scenarios().end()) {
    throw std::invalid_argument("unknown scenario '" + scenario + "'");
  }
  if (scenario == "flat") {
    return default_z;
  }
  if (scenario == "low_wall") {
    const double eps = 1e-9;  // paint_region's center-inclusion rule
    return x >= wall.x_min - eps && x < wall.x_max - eps ? wall.height : default_z;
  }
  return default_z + reference_z(scenario, x - it->second.origin_x, y - it->second.origin_y);
}

}  // namespace elevation_map_publisher
