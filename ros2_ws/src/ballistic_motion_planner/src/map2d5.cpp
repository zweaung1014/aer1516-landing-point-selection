#include "ballistic_motion_planner/map2d5.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ballistic
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}

Map2D5::Map2D5(
  double size_x, double size_y, double resolution,
  double origin_x, double origin_y, double default_z)
: size_x_(size_x), size_y_(size_y), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y)
{
  cols_ = static_cast<int>(std::lround(size_x / resolution));
  rows_ = static_cast<int>(std::lround(size_y / resolution));
  grid_.assign(static_cast<size_t>(rows_) * cols_, default_z);
}

std::pair<int, int> Map2D5::world_to_grid(double x, double y) const
{
  // Python int() truncates toward zero; std::trunc matches exactly.
  int col = static_cast<int>(std::trunc((x - origin_x_) / resolution_));
  int row = static_cast<int>(std::trunc((y - origin_y_) / resolution_));
  col = std::max(0, std::min(col, cols_ - 1));
  row = std::max(0, std::min(row, rows_ - 1));
  return {row, col};
}

std::pair<double, double> Map2D5::grid_to_world(int row, int col) const
{
  return {origin_x_ + (col + 0.5) * resolution_,
          origin_y_ + (row + 0.5) * resolution_};
}

bool Map2D5::is_within_bounds(double x, double y) const
{
  const double lx = x - origin_x_;
  const double ly = y - origin_y_;
  return lx >= 0.0 && lx < size_x_ && ly >= 0.0 && ly < size_y_;
}

void Map2D5::paint_region(
  double z, double x_min, double x_max, double y_min, double y_max)
{
  const double eps = 1e-9;
  for (int r = 0; r < rows_; ++r) {
    const double cy = origin_y_ + (r + 0.5) * resolution_;
    if (!std::isnan(y_min) && !(cy >= y_min - eps)) {continue;}
    if (!std::isnan(y_max) && !(cy < y_max - eps)) {continue;}
    for (int c = 0; c < cols_; ++c) {
      const double cx = origin_x_ + (c + 0.5) * resolution_;
      if (!std::isnan(x_min) && !(cx >= x_min - eps)) {continue;}
      if (!std::isnan(x_max) && !(cx < x_max - eps)) {continue;}
      at(r, c) = z;
    }
  }
}

void Map2D5::set_obstacle_region(double x_min, double y_min, double x_max, double y_max)
{
  // Python: inclusive cell-index box from world_to_grid of both corners.
  auto [r_min, c_min] = world_to_grid(x_min, y_min);
  auto [r_max, c_max] = world_to_grid(x_max, y_max);
  for (int r = r_min; r <= r_max; ++r) {
    for (int c = c_min; c <= c_max; ++c) {
      at(r, c) = OBSTACLE;
    }
  }
}

std::vector<uint8_t> Map2D5::steep_mask(double min_grade, bool finite_only) const
{
  const size_t n = grid_.size();
  std::vector<uint8_t> obs(n);
  for (size_t i = 0; i < n; ++i) {obs[i] = grid_[i] == OBSTACLE;}

  std::vector<uint8_t> mask(n, 0);
  if (!finite_only) {mask = obs;}

  const double res = resolution_;
  const double diag = res * std::sqrt(2.0);
  // Four offsets cover all eight neighbours; each pair marks both ends.
  // Pairs (a=(r,c), b=(r+dr, c+dc)): east (0,+1), north (+1,0), NE (+1,+1),
  // NW (a=(r,c+1), b=(r+1,c)) — mirroring the Python slice pairs.
  struct Off {int a_dc; int b_dr; int b_dc; double dist;};
  const Off offs[4] = {
    {0, 0, 1, res},    // east:  a=(r,c),   b=(r,c+1)
    {0, 1, 0, res},    // north: a=(r,c),   b=(r+1,c)
    {0, 1, 1, diag},   // NE:    a=(r,c),   b=(r+1,c+1)
    {1, 1, 0, diag},   // NW:    a=(r,c+1), b=(r+1,c)
  };
  for (const auto & o : offs) {
    const int r_max = rows_ - o.b_dr;
    for (int r = 0; r < r_max; ++r) {
      for (int c = 0; c + std::max(o.a_dc, o.b_dc) < cols_; ++c) {
        const size_t ia = static_cast<size_t>(r) * cols_ + (c + o.a_dc);
        const size_t ib = static_cast<size_t>(r + o.b_dr) * cols_ + (c + o.b_dc);
        bool steep = std::abs(grid_[ib] - grid_[ia]) > min_grade * o.dist;
        if (finite_only) {
          if (obs[ia] || obs[ib]) {steep = false;}
        } else {
          steep = steep || obs[ia] || obs[ib];
        }
        if (steep) {
          mask[ia] = 1;
          mask[ib] = 1;
        }
      }
    }
  }

  if (finite_only) {
    for (size_t i = 0; i < n; ++i) {
      if (obs[i]) {mask[i] = 0;}
    }
  }
  return mask;
}

std::vector<uint8_t> Map2D5::dilate_bool(
  const std::vector<uint8_t> & source, double reach) const
{
  const int r_cells = static_cast<int>(std::ceil(reach / resolution_));
  std::vector<uint8_t> out(grid_.size(), 0);
  for (int dr = -r_cells; dr <= r_cells; ++dr) {
    for (int dc = -r_cells; dc <= r_cells; ++dc) {
      if (std::hypot(static_cast<double>(dr), static_cast<double>(dc)) *
        resolution_ >= reach) {continue;}
      // out[r][c] |= source[r + dr][c + dc] (out-of-bounds source = false),
      // matching the padded-shift idiom in Python.
      const int r_lo = std::max(0, -dr), r_hi = std::min(rows_, rows_ - dr);
      const int c_lo = std::max(0, -dc), c_hi = std::min(cols_, cols_ - dc);
      for (int r = r_lo; r < r_hi; ++r) {
        const size_t o = static_cast<size_t>(r) * cols_;
        const size_t s = static_cast<size_t>(r + dr) * cols_ + dc;
        for (int c = c_lo; c < c_hi; ++c) {
          out[o + c] = out[o + c] | source[s + c];
        }
      }
    }
  }
  return out;
}

std::vector<uint8_t> Map2D5::standable_mask(
  double radius, double clearance, double steep_grade) const
{
  const double reach = radius + clearance;
  std::vector<uint8_t> obs(grid_.size());
  for (size_t i = 0; i < grid_.size(); ++i) {obs[i] = grid_[i] == OBSTACLE;}
  const auto obstacle_blocked = dilate_bool(obs, reach);
  // Shipped Python behavior ("# TEMP" in map2d5.py): steep edges are NOT
  // dilated — a finite steep edge blocks only its own cell.
  const auto edge_blocked = steep_mask(steep_grade, true);

  std::vector<uint8_t> out(grid_.size());
  for (size_t i = 0; i < grid_.size(); ++i) {
    out[i] = !obstacle_blocked[i] && !edge_blocked[i];
  }
  return out;
}

std::vector<double> Map2D5::inflated_field(double radius, double steep_grade) const
{
  const double lookup_pad = resolution_ * std::sqrt(2.0) / 2.0;
  const double reach = radius + lookup_pad;
  const int r_cells = static_cast<int>(std::ceil(reach / resolution_));

  const size_t n = grid_.size();
  std::vector<double> filled(n);
  for (size_t i = 0; i < n; ++i) {
    filled[i] = grid_[i] == OBSTACLE ? kInf : grid_[i];
  }
  const auto steep = steep_mask(steep_grade, false);
  std::vector<double> src(n);
  for (size_t i = 0; i < n; ++i) {src[i] = steep[i] ? filled[i] : -kInf;}

  std::vector<double> out(n, -kInf);
  for (int dr = -r_cells; dr <= r_cells; ++dr) {
    for (int dc = -r_cells; dc <= r_cells; ++dc) {
      if (std::hypot(static_cast<double>(dr), static_cast<double>(dc)) *
        resolution_ >= reach) {continue;}
      const int r_lo = std::max(0, -dr), r_hi = std::min(rows_, rows_ - dr);
      const int c_lo = std::max(0, -dc), c_hi = std::min(cols_, cols_ - dc);
      for (int r = r_lo; r < r_hi; ++r) {
        const size_t o = static_cast<size_t>(r) * cols_;
        const size_t s = static_cast<size_t>(r + dr) * cols_ + dc;
        for (int c = c_lo; c < c_hi; ++c) {
          out[o + c] = std::max(out[o + c], src[s + c]);
        }
      }
    }
  }
  // A cell always bounds its own ground; also restores +inf on OBSTACLE cells.
  for (size_t i = 0; i < n; ++i) {out[i] = std::max(out[i], filled[i]);}
  return out;
}

std::vector<double> Map2D5::min_abs_slope(int axis) const
{
  const size_t n = grid_.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> fwd(n, nan), bwd(n, nan);

  auto idx = [this](int r, int c) {return static_cast<size_t>(r) * cols_ + c;};

  if (axis == 1) {
    for (int r = 0; r < rows_; ++r) {
      for (int c = 0; c + 1 < cols_; ++c) {
        const double za = grid_[idx(r, c)], zb = grid_[idx(r, c + 1)];
        if (za == OBSTACLE || zb == OBSTACLE) {continue;}
        const double d = (zb - za) / resolution_;
        fwd[idx(r, c)] = d;
        bwd[idx(r, c + 1)] = d;
      }
    }
  } else {
    for (int r = 0; r + 1 < rows_; ++r) {
      for (int c = 0; c < cols_; ++c) {
        const double za = grid_[idx(r, c)], zb = grid_[idx(r + 1, c)];
        if (za == OBSTACLE || zb == OBSTACLE) {continue;}
        const double d = (zb - za) / resolution_;
        fwd[idx(r, c)] = d;
        bwd[idx(r + 1, c)] = d;
      }
    }
  }

  std::vector<double> out(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    const double mf = std::isnan(fwd[i]) ? kInf : std::abs(fwd[i]);
    const double mb = std::isnan(bwd[i]) ? kInf : std::abs(bwd[i]);
    const double v = mf <= mb ? fwd[i] : bwd[i];
    out[i] = std::isnan(v) ? 0.0 : v;
  }
  return out;
}

std::vector<Normal> Map2D5::surface_normals() const
{
  const auto s_x = min_abs_slope(1);
  const auto s_y = min_abs_slope(0);
  std::vector<Normal> out(grid_.size());
  for (size_t i = 0; i < grid_.size(); ++i) {
    if (grid_[i] == OBSTACLE) {
      out[i] = {0.0, 0.0, 1.0};
      continue;
    }
    const double nx = -s_x[i], ny = -s_y[i], nz = 1.0;
    const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    out[i] = {nx / norm, ny / norm, nz / norm};
  }
  return out;
}

}  // namespace ballistic
