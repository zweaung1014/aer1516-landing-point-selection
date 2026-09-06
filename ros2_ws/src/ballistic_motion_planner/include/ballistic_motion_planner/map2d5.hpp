// C++ port of hopcopter-ballistic-planning/map2d5.py (Map2D5).
//
// A 2.5D map: a 2D grid of elevation values (meters). OBSTACLE (-1.0) is a
// sentinel meaning an infinitely tall column, not an elevation. Unlike the
// Python original, the map carries a world-frame origin offset (origin_x,
// origin_y) for the bottom-left corner; the Python behavior is the special
// case origin = (0, 0). Row/col indexing matches Python: [row][col] = [y][x].
//
// Pure C++ (no ROS) so the planner core is testable standalone. Derived
// fields (steep mask, inflated field, normals, standable mask) are computed
// on demand with no memo caches: the planner builds a fresh map per plan.
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace ballistic
{

struct Normal
{
  double x, y, z;
};

class Map2D5
{
public:
  static constexpr double OBSTACLE = -1.0;

  Map2D5(
    double size_x, double size_y, double resolution,
    double origin_x = 0.0, double origin_y = 0.0, double default_z = 0.0);

  int rows() const {return rows_;}
  int cols() const {return cols_;}
  double resolution() const {return resolution_;}
  double size_x() const {return size_x_;}
  double size_y() const {return size_y_;}
  double origin_x() const {return origin_x_;}
  double origin_y() const {return origin_y_;}

  double & at(int row, int col) {return grid_[static_cast<size_t>(row) * cols_ + col];}
  double at(int row, int col) const {return grid_[static_cast<size_t>(row) * cols_ + col];}

  // Python: (row, col) = (clamp(int(y/res)), clamp(int(x/res))) — truncation
  // toward zero (matches Python int()), then clamp into range.
  std::pair<int, int> world_to_grid(double x, double y) const;
  // Cell centers: (origin + (idx + 0.5) * res).
  std::pair<double, double> grid_to_world(int row, int col) const;
  // Half-open: origin <= x < origin + size.
  bool is_within_bounds(double x, double y) const;

  // Set every cell whose CENTER lies in [x_min, x_max) x [y_min, y_max) to z.
  // Bounds are world coords; NaN means unbounded on that side (Python None).
  void paint_region(
    double z,
    double x_min, double x_max, double y_min, double y_max);
  void set_obstacle_region(double x_min, double y_min, double x_max, double y_max);

  // Boolean grid (row-major, uint8): which cells are terrain EDGES.
  // A cell is steep iff some 8-neighbour differs by more than min_grade * dist
  // (dist = res orthogonal, res*sqrt(2) diagonal); both cells of a pair are
  // marked. OBSTACLE cells are steep unconditionally and so is anything
  // adjacent to one; finite_only excludes obstacles and obstacle-adjacency.
  std::vector<uint8_t> steep_mask(double min_grade, bool finite_only) const;

  // True within `reach` of a true source cell (strict <, matching Python).
  std::vector<uint8_t> dilate_bool(const std::vector<uint8_t> & source, double reach) const;

  // SHIPPED Python behavior (map2d5.py has the steep-edge dilation commented
  // out with a "# TEMP" note): obstacle cells dilated by radius + clearance,
  // finite steep edges block only themselves (no dilation).
  std::vector<uint8_t> standable_mask(
    double radius, double clearance, double steep_grade) const;

  // Terrain EDGES dilated sideways by radius (+ lookup_pad = res*sqrt(2)/2),
  // maxed against local ground (load-bearing). OBSTACLE -> +inf.
  std::vector<double> inflated_field(double radius, double steep_grade) const;

  // Per-cell outward unit normals from min-|.| one-sided differences (NEVER a
  // central difference). OBSTACLE cells get (0, 0, 1). n.z > 0 always.
  std::vector<Normal> surface_normals() const;

private:
  // Per-axis slope in the min-|.| sense; axis 1 = along columns (world x),
  // axis 0 = along rows (world y). OBSTACLE cells join no difference.
  std::vector<double> min_abs_slope(int axis) const;

  double size_x_, size_y_, resolution_, origin_x_, origin_y_;
  int rows_, cols_;
  std::vector<double> grid_;
};

}  // namespace ballistic
