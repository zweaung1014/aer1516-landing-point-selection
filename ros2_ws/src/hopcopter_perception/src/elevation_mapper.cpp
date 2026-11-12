#include "hopcopter_perception/elevation_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace hopcopter_perception
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
// A run of hits well below a cell's estimate that agree with each other (std
// below this, at least this many) means the estimate itself is wrong (an
// outlier replaced it, or the pose jumped): the run replaces it. A wall's face
// under its top never agrees with itself like this.
constexpr int kBelowRun = 30;
constexpr double kBelowStd = 0.015;
}

ElevationMapper::ElevationMapper(const MapperParams & p)
: p_(p)
{
  cols_ = static_cast<int>(std::lround(p_.size_x / p_.resolution));
  rows_ = static_cast<int>(std::lround(p_.size_y / p_.resolution));
  const size_t n = static_cast<size_t>(rows_) * cols_;
  h_.assign(n, kNaN);
  var_.assign(n, kInf);
  ub_.assign(3 * n, std::numeric_limits<float>::infinity());
  over_.assign(3 * n, -std::numeric_limits<float>::infinity());
  hits_.assign(n, 0);
  fresh_.assign(n, 0);
  clear_.assign(n, 0);
  flags_.assign(n, 0);
  below_n_.assign(n, 0);
  below_mean_.assign(n, 0.0f);
  below_m2_.assign(n, 0.0f);
}

bool ElevationMapper::cell_of(double x, double y, int & row, int & col) const
{
  const double fx = (x - p_.origin_x) / p_.resolution;
  const double fy = (y - p_.origin_y) / p_.resolution;
  if (!(fx >= 0.0 && fy >= 0.0)) {
    return false;
  }
  col = static_cast<int>(fx);
  row = static_cast<int>(fy);
  return col < cols_ && row < rows_;
}

bool ElevationMapper::top_seen(size_t i) const
{
  return (flags_[i] & (kAssumed | kContact)) || !p_.require_top_confirmation ||
         over_[3 * i + 2] > h_[i] + p_.confirm_margin;
}

bool ElevationMapper::confirmed(size_t i) const
{
  if (std::isnan(h_[i])) {
    return false;
  }
  if (flags_[i] & (kAssumed | kContact)) {
    return true;
  }
  return fresh_[i] >= p_.min_hits && top_seen(i);
}

double ElevationMapper::elevation(size_t i) const
{
  // Only faces reached it (a tread above the sensor, a pole's inside): no ground
  // to land on. It stays unseen, at its bound (or an obstacle) for clearance.
  if (flags_[i] & kFace) {
    return kNaN;
  }
  return confirmed(i) ? h_[i] : kNaN;
}

double ElevationMapper::upper_bound(size_t i) const
{
  if (!std::isnan(h_[i]) && !top_seen(i)) {
    return kNaN;  // hits whose top no ray has seen: maybe taller than anything that reached it
  }
  // A height with too few hits yet is no landing, but the rays over it still bound it.
  const float b = ub_[3 * i + 2];
  return std::isfinite(b) ? static_cast<double>(b) : kNaN;
}

void ElevationMapper::integrate_scan(
  const Eigen::Vector3d & origin, const std::vector<Eigen::Vector3d> & points)
{
  ++n_scans_;
  classify_faces(points);
  // Carve first, then the hits: within one scan a hit wins over a ray that
  // grazed past it.
  ray_ok_.assign(points.size(), 1);
  if (p_.ray_carving) {
    for (size_t k = 0; k < points.size(); ++k) {
      const double r = (points[k] - origin).norm();
      if (r >= p_.min_range && r <= p_.carve_max_range) {
        ray_ok_[k] = carve(origin, points[k], r);
      }
    }
  }
  for (size_t k = 0; k < points.size(); ++k) {
    if (!ray_ok_[k]) {
      continue;
    }
    const auto & pt = points[k];
    const Eigen::Vector3d d = pt - origin;
    const double r = d.norm();
    if (r < p_.min_range || r > p_.max_update_range) {
      continue;
    }
    int row, col;
    if (!cell_of(pt.x(), pt.y(), row, col)) {
      continue;
    }
    const double dxy = std::hypot(d.x(), d.y());
    const double sin_el = std::abs(d.z()) / r;
    const double cos_el = dxy / r;
    const double s_r = p_.range_sigma * sin_el;
    const double s_a = r * p_.attitude_sigma * cos_el;
    HitRay ray{pt.x(), pt.y(), 0.0, 0.0, 0.0};
    if (p_.edge_attribution && dxy > 1e-6) {
      ray.ux = d.x() / dxy;
      ray.uy = d.y() / dxy;
      ray.band = p_.edge_sigmas * p_.range_sigma * cos_el;
    }
    hit(index(row, col), pt.z(), s_r * s_r + s_a * s_a + p_.min_sigma * p_.min_sigma, face_[k], ray);
  }
}

void ElevationMapper::classify_faces(const std::vector<Eigen::Vector3d> & points)
{
  const size_t n = points.size();
  face_.assign(n, 0);
  if (!p_.face_filter || n == 0) {
    return;
  }
  // Buckets face_radius wide: every partner of a point lies in its own bucket or
  // one of the eight around it.
  const double w = p_.face_radius;
  auto key_of = [w](double x, double y, int dx, int dy) {
      const int64_t bx = static_cast<int64_t>(std::floor(x / w)) + dx;
      const int64_t by = static_cast<int64_t>(std::floor(y / w)) + dy;
      return (static_cast<uint64_t>(bx) << 32) ^ static_cast<uint64_t>(static_cast<uint32_t>(by));
    };
  buckets_.resize(n);
  for (size_t k = 0; k < n; ++k) {
    buckets_[k] = {key_of(points[k].x(), points[k].y(), 0, 0), static_cast<uint32_t>(k)};
  }
  std::sort(buckets_.begin(), buckets_.end());
  const double r2 = p_.face_radius * p_.face_radius;
  const double s2 = p_.face_min_slope * p_.face_min_slope;
  long n_face = 0;
  for (size_t k = 0; k < n; ++k) {
    const Eigen::Vector3d & a = points[k];
    bool face = false;
    for (int dx = -1; dx <= 1 && !face; ++dx) {
      for (int dy = -1; dy <= 1 && !face; ++dy) {
        const uint64_t key = key_of(a.x(), a.y(), dx, dy);
        auto it = std::lower_bound(buckets_.begin(), buckets_.end(),
            std::make_pair(key, uint32_t{0}));
        for (; it != buckets_.end() && it->first == key; ++it) {
          const Eigen::Vector3d & b = points[it->second];
          const double dz = std::abs(a.z() - b.z());
          if (dz < p_.face_min_dz || dz > p_.face_max_dz) {
            continue;
          }
          const double ex = a.x() - b.x(), ey = a.y() - b.y();
          const double d2 = ex * ex + ey * ey;
          if (d2 <= r2 && dz * dz >= s2 * d2) {
            face = true;
            break;
          }
        }
      }
    }
    face_[k] = face;
    n_face += face;
  }
  n_points_ += static_cast<long>(n);
  n_face_points_ += n_face;
}

bool ElevationMapper::attributable(size_t i, double z, double var, const HitRay & ray) const
{
  if (ray.band <= 0.0) {
    return false;
  }
  for (const double f : {-1.0, -0.5, 0.5, 1.0}) {
    int r, c;
    if (!cell_of(ray.x + f * ray.band * ray.ux, ray.y + f * ray.band * ray.uy, r, c)) {
      continue;
    }
    const size_t j = index(r, c);
    if (j == i || std::isnan(h_[j])) {
      continue;
    }
    if (h_[j] >= z - p_.mahalanobis_threshold * std::sqrt(var + var_[j])) {
      return true;
    }
  }
  return false;
}

void ElevationMapper::hit(size_t i, double z, double var, bool face, const HitRay & ray)
{
  if (face) {
    // A face point says nothing about the ground of the cell it fell in (it may
    // be a few cm in front of its face). Where ground hits or a contact made the
    // estimate, they decide.
    if (!std::isnan(h_[i]) && !(flags_[i] & (kFace | kAssumed))) {
      ++n_face_ignored_;
      return;
    }
  } else if (flags_[i] & kFace) {
    // The cell's first ground hit replaces what only faces said.
    ++n_hits_;
    ++hits_[i];
    h_[i] = z;
    var_[i] = var;
    flags_[i] &= static_cast<uint8_t>(~kFace);
    fresh_[i] = 1;
    clear_[i] = 0;
    reset_below(i);
    return;
  }
  if (!std::isnan(h_[i]) && !(flags_[i] & (kAssumed | kContact)) &&
    fresh_[i] >= p_.edge_min_support &&
    z - h_[i] > p_.mahalanobis_threshold * std::sqrt(var_[i] + var) &&
    attributable(i, z, var, ray))
  {
    // It would raise this cell, but a neighbour as high explains it: noise
    // moved it here past that neighbour's edge.
    ++n_attributed_;
    return;
  }
  ++n_hits_;
  ++hits_[i];
  if (std::isnan(h_[i]) || (flags_[i] & kAssumed)) {
    h_[i] = z;
    var_[i] = var;
    flags_[i] &= static_cast<uint8_t>(~kAssumed);
    if (face) {
      flags_[i] |= kFace;
    }
    fresh_[i] = 1;
    clear_[i] = 0;
    reset_below(i);
    return;
  }
  const double d = z - h_[i];
  const double s = std::sqrt(var_[i] + var);
  if (d > p_.mahalanobis_threshold * s) {
    // A higher surface: it replaces the estimate (unless the estimate is a foot
    // contact, which nothing outranks).
    if (!(flags_[i] & kContact)) {
      h_[i] = z;
      var_[i] = var;
      fresh_[i] = 1;
      clear_[i] = 0;
      reset_below(i);
    }
    return;
  }
  if (d < -p_.mahalanobis_threshold * s) {
    // Below the surface (a wall's face under its top): ignored, but a run of
    // such hits that agree with each other replaces a wrong estimate.
    var_[i] = std::max(var_[i], std::min(var_[i] + p_.multi_height_noise,
      p_.max_inflated_sigma * p_.max_inflated_sigma));
    if (flags_[i] & kContact) {
      return;
    }
    const int n = ++below_n_[i];
    const double delta = z - below_mean_[i];
    below_mean_[i] += static_cast<float>(delta / n);
    below_m2_[i] += static_cast<float>(delta * (z - below_mean_[i]));
    if (n >= kBelowRun && below_m2_[i] / (n - 1) < kBelowStd * kBelowStd) {
      h_[i] = below_mean_[i];
      var_[i] = std::max(static_cast<double>(below_m2_[i]) / (n - 1) / n, var / n);
      fresh_[i] = static_cast<uint16_t>(n);
      clear_[i] = 0;
      reset_below(i);
    }
    return;
  }
  // Kalman fusion.
  const double v0 = var_[i];
  h_[i] = (var * h_[i] + v0 * z) / (v0 + var);
  var_[i] = std::max(v0 * var / (v0 + var), p_.min_cell_sigma * p_.min_cell_sigma);
  if (fresh_[i] < 65535) {
    ++fresh_[i];
  }
  clear_[i] = 0;
  // A run of lower hits is correcting this estimate: a hit a neighbour explains
  // (a stray point from its edge) does not stop it.
  if (below_n_[i] == 0 || !attributable(i, z, var, ray)) {
    reset_below(i);
  }
}

void ElevationMapper::reset_below(size_t i)
{
  below_n_[i] = 0;
  below_mean_[i] = 0.0f;
  below_m2_[i] = 0.0f;
}

// Trace the ray o -> p through the grid (2D DDA) up to carve_end_skip before
// the hit. Over each cell it crosses, the ray proves the terrain lower than the
// ray's highest point there: that is the bound (plus margins). A height the
// ray passes clearly below cannot be there and is removed. The ray's lowest
// point over the cell confirms any height below it.
bool ElevationMapper::carve(const Eigen::Vector3d & o, const Eigen::Vector3d & p, double range)
{
  const double Dx = p.x() - o.x(), Dy = p.y() - o.y(), Dz = p.z() - o.z();
  const double dxy = std::hypot(Dx, Dy);
  if (dxy <= p_.carve_end_skip + 1e-9) {
    return true;
  }
  double s_lo = 0.0;
  double s_hi = 1.0 - p_.carve_end_skip / dxy;
  // Clip [s_lo, s_hi] to the map rectangle (Liang-Barsky).
  const double x0 = p_.origin_x, x1 = p_.origin_x + cols_ * p_.resolution;
  const double y0 = p_.origin_y, y1 = p_.origin_y + rows_ * p_.resolution;
  auto clip = [&](double q, double d, double lo, double hi) {
      // lo <= q + s d <= hi
      if (std::abs(d) < 1e-12) {
        return q >= lo && q <= hi;
      }
      double a = (lo - q) / d, b = (hi - q) / d;
      if (a > b) {std::swap(a, b);}
      s_lo = std::max(s_lo, a);
      s_hi = std::min(s_hi, b);
      return s_lo < s_hi;
    };
  if (!clip(o.x(), Dx, x0, x1) || !clip(o.y(), Dy, y0, y1)) {
    return true;
  }
  const double res = p_.resolution;
  const double sx = o.x() + s_lo * Dx, sy = o.y() + s_lo * Dy;
  int col = std::clamp(static_cast<int>(std::floor((sx - x0) / res)), 0, cols_ - 1);
  int row = std::clamp(static_cast<int>(std::floor((sy - y0) / res)), 0, rows_ - 1);
  const int step_c = Dx > 0 ? 1 : -1, step_r = Dy > 0 ? 1 : -1;
  const double d_sx = std::abs(Dx) > 1e-12 ? res / std::abs(Dx) : kInf;
  const double d_sy = std::abs(Dy) > 1e-12 ? res / std::abs(Dy) : kInf;
  double next_sx = kInf, next_sy = kInf;
  if (std::abs(Dx) > 1e-12) {
    const double bx = x0 + (col + (Dx > 0 ? 1 : 0)) * res;
    next_sx = (bx - o.x()) / Dx;
  }
  if (std::abs(Dy) > 1e-12) {
    const double by = y0 + (row + (Dy > 0 ? 1 : 0)) * res;
    next_sy = (by - o.y()) / Dy;
  }
  // The bound's margin grows with the attitude error's lever arm.
  const double margin = p_.carve_margin + 2.0 * range * p_.attitude_sigma;
  double s_in = s_lo;
  while (s_in < s_hi) {
    const double s_out = std::min({next_sx, next_sy, s_hi});
    // Only a ray through the middle of the cell says anything about the terrain
    // at its centre, which is what a 2.5D cell stands for: one that clips a
    // corner can miss a pad covering the rest.
    const double cx = cell_x(col) - o.x(), cy = cell_y(row) - o.y();
    const double s_c = std::clamp((cx * Dx + cy * Dy) / (dxy * dxy), s_in, s_out);
    if (std::hypot(s_c * Dx - cx, s_c * Dy - cy) <= p_.carve_center_tolerance) {
      const double za = o.z() + s_in * Dz, zb = o.z() + s_out * Dz;
      const double z_top = std::max(za, zb), z_low = std::min(za, zb);
      const size_t i = index(row, col);
      if (!std::isnan(h_[i]) && h_[i] > z_top + p_.carve_clear_margin &&
        (flags_[i] & kContact || (fresh_[i] >= p_.trusted_hits && top_seen(i))))
      {
        ++n_through_;
        return false;  // through a surface it should have hit: carve nothing past it
      }
      // keep the three lowest bounds and the three highest passes
      float b = static_cast<float>(z_top + margin);
      float * u = &ub_[3 * i];
      if (b < u[2]) {
        if (b < u[1]) {
          u[2] = u[1];
          if (b < u[0]) {
            u[1] = u[0];
            u[0] = b;
          } else {
            u[1] = b;
          }
        } else {
          u[2] = b;
        }
      }
      float a = static_cast<float>(z_low);
      float * v = &over_[3 * i];
      if (a > v[2]) {
        if (a > v[1]) {
          v[2] = v[1];
          if (a > v[0]) {
            v[1] = v[0];
            v[0] = a;
          } else {
            v[1] = a;
          }
        } else {
          v[2] = a;
        }
      }
      if (!std::isnan(h_[i]) && !(flags_[i] & kContact) &&
        h_[i] > z_top + p_.carve_clear_margin && ++clear_[i] >= p_.carve_clear_rays)
      {
        h_[i] = kNaN;
        var_[i] = kInf;
        flags_[i] &= static_cast<uint8_t>(~(kAssumed | kFace));
        fresh_[i] = 0;
        clear_[i] = 0;
        reset_below(i);
        ++n_cleared_;
      }
    }
    if (s_out >= s_hi) {
      break;
    }
    if (next_sx <= next_sy) {
      col += step_c;
      next_sx += d_sx;
    } else {
      row += step_r;
      next_sy += d_sy;
    }
    if (col < 0 || col >= cols_ || row < 0 || row >= rows_) {
      break;
    }
    s_in = s_out;
  }
  return true;
}

void ElevationMapper::add_foot_contact(double x, double y, double z)
{
  int row, col;
  if (!cell_of(x, y, row, col)) {
    return;
  }
  const size_t i = index(row, col);
  h_[i] = z;
  var_[i] = p_.contact_sigma * p_.contact_sigma;
  flags_[i] = kContact;
  clear_[i] = 0;
  reset_below(i);
  if (!have_contact_ && p_.start_disk_radius > 0.0) {
    // The blind zone the robot takes off over: assume level ground at the
    // foot's height, until the LiDAR says otherwise.
    const int n = static_cast<int>(std::ceil(p_.start_disk_radius / p_.resolution)) + 1;
    for (int r = std::max(0, row - n); r <= std::min(rows_ - 1, row + n); ++r) {
      for (int c = std::max(0, col - n); c <= std::min(cols_ - 1, col + n); ++c) {
        const size_t j = index(r, c);
        if (!std::isnan(h_[j]) || std::hypot(cell_x(c) - x, cell_y(r) - y) > p_.start_disk_radius) {
          continue;
        }
        h_[j] = z;
        var_[j] = 0.05 * 0.05;
        flags_[j] = kAssumed;
      }
    }
  }
  have_contact_ = true;
}

}  // namespace hopcopter_perception
