// Offline check of the elevation mapper (stage 3 of docs/lidar_mapping_plan.md),
// no ROS graph and no Gazebo. For each scenario it flies a simulated Mid-360
// (mounted upside down 0.4 m above the base: 40 rings from -52 to +7 deg, 512
// azimuths, 2 cm range noise) along the robot's known-map plan: hanging at the
// spawn pose for 1 s, the drop, six hops in place, one hop per waypoint, and
// hops in place at the goal, with a foot contact at every touchdown. The scans
// are cast against the scenario's known map (elevation_map_publisher's
// scenarios.hpp, evaluated exactly, with infinitely tall columns 3 m tall as in
// the Gazebo worlds) inside a 1.2 m arena wall. It then scores the map as the
// stage-3 pass criteria do:
//  - height error on observed cells: median <= 2 cm, 95th percentile <= 5 cm;
//  - no false obstacles (> 10 cm above the truth) away from terrain edges;
//  - errors over 10 cm only within one cell of an edge;
// Reported, not failed on (docs/lidar_map_quality_plan.md): phantoms, cells reading
// more than 5 cm above every terrain under them (the floor in front of a face taking
// its height, a rim seen from above read a cell wider), at planning time and at the
// end; sunken cells, more than 5 cm below it all; and the error on flat terrain.
//  - no upper bound below the terrain (that would let the planner fly into it);
//  - along the plan: every landing cell measured, every cell under every arc
//    measured or bounded.
// --shift <m> moves the map's lattice by that much in x and y against the world, so
// faces that sit on cell borders in the worlds fall inside cells.
// --no-face-filter and --no-edge-attribution turn those off (both off: the mapper before Fix 2).
// --map-out <dir> writes the map at planning time (after the warm-up hops) to
// <dir>/lidar_map_<scenario>.txt, for ballistic_plan_check --lidar-maps <dir>:
// "rows cols resolution origin_x origin_y", then the elevation and the
// upper_bound, row by row (row = y), nan where there is none.
// Usage: mapper_check [scenario ...] [--pose-noise <deg>] [--shift <m>] [--no-face-filter]
//   [--no-edge-attribution] [--map-out <dir>] [--json <dir>] [--verbose].
// Exits 1 if a scenario fails.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "elevation_map_publisher/scenarios.hpp"
#include "hopcopter_perception/elevation_mapper.hpp"

using hopcopter_perception::ElevationMapper;
using hopcopter_perception::MapperParams;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr double kG = 9.55;          // the idle fall's g'
constexpr double kLeg = 0.366;       // CoM height at touchdown
constexpr double kMast = 0.4;        // sensor above the base
constexpr double kTallCap = 3.0;     // an "infinitely tall" column in the Gazebo worlds

struct Waypoint
{
  double x, y, apex;
};

// The known-map plans (ballistic_planner_node's defaults, start (0, 0), the
// reference goal), as printed by the planner: x, y, target apex.
const std::map<std::string, std::vector<Waypoint>> kPlans = {
  {"flat", {{0.05, 0.05, 0.8}, {1.35, 0.05, 0.721}, {2.65, 0.05, 0.721}, {4.05, 0.05, 0.700}}},
  {"low_wall", {{0.05, 0.05, 0.8}, {1.75, 0.05, 0.727}, {3.35, 0.05, 0.767}, {4.05, 0.05, 0.943}}},
  {"stairs", {{0.05, 0.05, 0.8}, {0.75, -0.95, 0.742}, {0.85, -1.25, 1.084}, {1.15, -1.25, 1.265},
      {1.45, -1.25, 1.265}, {1.75, -1.25, 1.265}, {2.05, -1.25, 1.265}, {2.35, -1.25, 1.265},
      {2.65, -1.25, 1.265}, {4.05, -1.95, 1.060}}},
  {"slope_30deg", {{0.05, 0.05, 0.8}, {0.75, -0.95, 0.844}, {1.15, -1.05, 0.984},
      {1.55, -1.15, 0.984}, {1.95, -1.25, 0.984}, {2.35, -1.35, 0.984}, {2.75, -1.45, 0.984},
      {3.15, -1.55, 0.984}, {3.55, -1.65, 0.984}, {4.25, -1.95, 0.982}}},
  {"cliff_gap", {{0.05, 0.05, 0.8}, {0.95, 0.05, 0.700}, {1.85, 0.15, 0.903}, {2.25, 0.15, 0.985},
      {2.65, 0.15, 0.985}, {3.05, 0.15, 0.985}, {3.45, 0.15, 0.985}, {3.85, 0.15, 0.985},
      {4.25, 0.15, 0.985}, {5.15, 0.25, 0.914}, {5.85, 0.25, 0.723}, {7.15, 0.35, 0.720},
      {8.35, 0.45, 0.746}, {9.55, 0.55, 0.746}}},
  {"maze", {{0.05, 0.05, 0.8}, {1.75, 0.45, 0.705}, {3.95, 0.25, 0.700}, {6.05, 0.05, 0.700},
      {7.85, -0.15, 0.849}, {9.95, -0.15, 0.700}, {12.05, -0.15, 0.700}, {14.05, 0.05, 0.756}}},
  {"lilypads", {{0.05, 0.05, 0.8}, {1.65, 0.65, 0.500}, {3.35, 0.15, 0.821}, {4.75, -0.05, 0.934},
      {6.45, -0.35, 0.839}, {8.05, -0.55, 0.640}, {10.05, -0.55, 0.800}, {11.75, 0.05, 0.604}}},
  {"slanted_pads", {{0.05, 0.05, 0.8}, {0.75, 0.35, 0.715}, {2.45, 0.45, 0.606},
      {4.15, -0.45, 0.769}, {5.85, 0.45, 0.654}, {7.95, -0.55, 0.842}, {9.95, 0.45, 0.885},
      {11.75, -0.55, 0.870}, {13.55, -0.05, 1.061}, {15.85, 0.05, 1.036}}},
};

// The world the synthetic LiDAR sees: the scenario's terrain inside its map,
// ground elsewhere, and a 1.2 m arena wall 2 m outside the map. Rastered at
// 1 cm for speed.
class World
{
public:
  explicit World(const std::string & sc)
  {
    const auto & s = elevation_map_publisher::scenarios().at(sc);
    x0_ = s.origin_x - 2.0;
    y0_ = s.origin_y - 2.0;
    nx_ = static_cast<int>(std::lround((s.size_x + 4.0) / res_));
    ny_ = static_cast<int>(std::lround((s.size_y + 4.0) / res_));
    z_.resize(static_cast<size_t>(nx_) * ny_);
    for (int j = 0; j < ny_; ++j) {
      for (int i = 0; i < nx_; ++i) {
        const double x = x0_ + (i + 0.5) * res_, y = y0_ + (j + 0.5) * res_;
        double z = 0.0;
        if (x >= s.origin_x && x < s.origin_x + s.size_x && y >= s.origin_y &&
          y < s.origin_y + s.size_y)
        {
          z = std::min(elevation_map_publisher::world_z(sc, x, y), kTallCap);
        }
        if (i == 0 || j == 0 || i == nx_ - 1 || j == ny_ - 1) {
          z = 1.2;
        }
        z_[static_cast<size_t>(j) * nx_ + i] = static_cast<float>(z);
      }
    }
  }

  double z(double x, double y) const
  {
    const int i = std::clamp(static_cast<int>((x - x0_) / res_), 0, nx_ - 1);
    const int j = std::clamp(static_cast<int>((y - y0_) / res_), 0, ny_ - 1);
    return z_[static_cast<size_t>(j) * nx_ + i];
  }

  // First hit of the ray o + t u (unit u) within max_range, or false. The raster
  // is 1 cm, so a ray can still graze past a corner by under a centimetre.
  bool cast(const Eigen::Vector3d & o, const Eigen::Vector3d & u, double max_range, double & t_hit) const
  {
    const double step = 0.01;  // small enough not to tunnel through a pole's corner
    double t_prev = 0.0;
    for (double t = step; t <= max_range; t += step) {
      const Eigen::Vector3d p = o + t * u;
      if (p.z() <= z(p.x(), p.y())) {
        double lo = t_prev, hi = t;  // bisect the crossing
        for (int k = 0; k < 6; ++k) {
          const double mid = 0.5 * (lo + hi);
          const Eigen::Vector3d q = o + mid * u;
          (q.z() <= z(q.x(), q.y()) ? hi : lo) = mid;
        }
        t_hit = hi;
        return true;
      }
      t_prev = t;
      if (p.z() > kTallCap + 0.5 && u.z() >= 0.0) {
        return false;  // above everything and climbing
      }
    }
    return false;
  }

private:
  double x0_, y0_, res_ = 0.01;
  int nx_, ny_;
  std::vector<float> z_;
};

struct Pose
{
  Eigen::Vector3d base;  // base position (level body)
  bool touchdown;        // a touchdown happened since the previous scan
  Eigen::Vector3d foot;
  bool plan_time = false;  // the last scan before the first steered hop
};

// Base positions sampled at 10 Hz along the run.
std::vector<Pose> trajectory(const std::string & sc, const World & w)
{
  const auto & plan = kPlans.at(sc);
  const double spawn_z = (sc == "lilypads" || sc == "slanted_pads") ? 3.4 : 1.34128;
  struct Hop {Eigen::Vector2d a, b; double ga, gb, apex;};
  std::vector<Hop> hops;
  auto ground = [&](double x, double y) {return w.z(x, y);};
  const Eigen::Vector2d p0(0.0, 0.0);
  for (int k = 0; k < 6; ++k) {
    hops.push_back({p0, p0, ground(0, 0), ground(0, 0), 0.8});
  }
  for (size_t i = 1; i < plan.size(); ++i) {
    const Eigen::Vector2d a(plan[i - 1].x, plan[i - 1].y), b(plan[i].x, plan[i].y);
    hops.push_back({i == 1 ? p0 : a, b, ground(a.x(), a.y()), ground(b.x(), b.y()), plan[i].apex});
  }
  const Eigen::Vector2d g(plan.back().x, plan.back().y);
  for (int k = 0; k < 4; ++k) {
    hops.push_back({g, g, ground(g.x(), g.y()), ground(g.x(), g.y()), 0.8});
  }

  // Piecewise timeline: hold 1 s, drop, then each hop (rise + fall).
  std::vector<Pose> out;
  const double dt = 0.1;
  double t = 0.0;
  for (; t < 1.0 - 1e-9; t += dt) {
    out.push_back({{0.0, 0.0, spawn_z}, false, {}});
  }
  const double z_td0 = ground(0, 0) + kLeg;
  const double t_drop = std::sqrt(2.0 * (spawn_z - z_td0) / kG);
  double t0 = t;  // segment start
  auto add_until = [&](double t_end, auto && pos_at) {
      bool first = true;
      for (; t < t_end - 1e-9; t += dt) {
        out.push_back({pos_at(t), false, {}});
        (void)first;
      }
    };
  add_until(t0 + t_drop, [&](double tt) {
      const double s = tt - t0;
      return Eigen::Vector3d(0.0, 0.0, spawn_z - 0.5 * kG * s * s);
    });
  t0 += t_drop;
  bool pending_td = true;
  Eigen::Vector3d td_foot(0.0, 0.0, ground(0, 0));
  size_t n_hop = 0;
  for (const auto & h : hops) {
    if (n_hop++ == 6 && !out.empty()) {
      out.back().plan_time = true;
    }
    const double z0 = h.ga + kLeg, z1 = h.gb + kLeg, za = h.ga + kLeg + h.apex;
    const double t_up = std::sqrt(2.0 * (za - z0) / kG), t_dn = std::sqrt(2.0 * std::max(za - z1, 0.01) / kG);
    const double T = t_up + t_dn;
    const size_t first = out.size();
    add_until(t0 + T, [&](double tt) {
        const double s = tt - t0;
        const Eigen::Vector2d xy = h.a + (h.b - h.a) * (s / T);
        const double z = s < t_up ? za - 0.5 * kG * (t_up - s) * (t_up - s) :
        za - 0.5 * kG * (s - t_up) * (s - t_up);
        return Eigen::Vector3d(xy.x(), xy.y(), z);
      });
    if (pending_td && first < out.size()) {
      out[first].touchdown = true;
      out[first].foot = td_foot;
      pending_td = false;
    }
    t0 += T;
    pending_td = true;
    td_foot = Eigen::Vector3d(h.b.x(), h.b.y(), h.gb);
  }
  return out;
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return NAN;
  }
  std::sort(v.begin(), v.end());
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(q * (v.size() - 1) + 0.5));
  return v[k];
}

void write_map(const std::string & file, const ElevationMapper & m)
{
  std::ofstream f(file);
  const auto & p = m.params();
  f << m.rows() << " " << m.cols() << " " << p.resolution << " " << p.origin_x << " " << p.origin_y
    << "\n";
  for (int layer = 0; layer < 2; ++layer) {
    for (int r = 0; r < m.rows(); ++r) {
      for (int c = 0; c < m.cols(); ++c) {
        const size_t i = m.index(r, c);
        const double v = layer == 0 ? m.elevation(i) : m.upper_bound(i);
        f << (c ? " " : "");
        if (std::isfinite(v)) {f << v;} else {f << "nan";}
      }
      f << "\n";
    }
  }
}

void write_json(
  const std::string & file, const std::string & sc, const ElevationMapper & m,
  const std::vector<double> & truth, const std::vector<Pose> & traj)
{
  std::ofstream f(file);
  const auto & p = m.params();
  auto arr = [&](const char * key, auto getter) {
      f << "\"" << key << "\": [";
      for (int i = 0; i < m.rows() * m.cols(); ++i) {
        const double v = getter(static_cast<size_t>(i));
        f << (i ? "," : "");
        if (std::isfinite(v)) {f << v;} else {f << "null";}
      }
      f << "],\n";
    };
  f << "{\n\"scenario\": \"" << sc << "\",\n\"origin_x\": " << p.origin_x << ",\n\"origin_y\": "
    << p.origin_y << ",\n\"resolution\": " << p.resolution << ",\n\"rows\": " << m.rows()
    << ",\n\"cols\": " << m.cols() << ",\n";
  arr("elevation", [&](size_t i) {return m.elevation(i);});
  arr("upper_bound", [&](size_t i) {return m.upper_bound(i);});
  arr("raw", [&](size_t i) {return m.raw_height(i);});
  arr("hits", [&](size_t i) {return static_cast<double>(m.hits(i));});
  arr("flags", [&](size_t i) {return static_cast<double>(m.flags(i));});
  arr("known", [&](size_t i) {return truth[i];});
  f << "\"trajectory\": [";
  for (size_t i = 0; i < traj.size(); ++i) {
    f << (i ? "," : "") << "[" << traj[i].base.x() << "," << traj[i].base.y() << "," <<
      traj[i].base.z() << "]";
  }
  f << "],\n\"format\": 1\n}\n";
}

// Along the plan: landing cells without a usable height, and arc samples
// over cells neither measured nor bounded (the planner treats those as walls).
std::pair<int, int> plan_coverage(const ElevationMapper & m, const std::vector<Waypoint> & plan)
{
  int land_unmeasured = 0, arc_unknown = 0;
  for (size_t k = 1; k < plan.size(); ++k) {
    int r, c;
    if (m.cell_of(plan[k].x, plan[k].y, r, c) && std::isnan(m.elevation(m.index(r, c)))) {
      ++land_unmeasured;
    }
    const double dx = plan[k].x - plan[k - 1].x, dy = plan[k].y - plan[k - 1].y;
    const int n = std::max(2, static_cast<int>(std::hypot(dx, dy) / 0.03));
    for (int j = 1; j < n; ++j) {
      const double x = plan[k - 1].x + dx * j / n, y = plan[k - 1].y + dy * j / n;
      if (m.cell_of(x, y, r, c)) {
        const size_t i = m.index(r, c);
        if (std::isnan(m.elevation(i)) && std::isnan(m.upper_bound(i))) {++arc_unknown;}
      }
    }
  }
  return {land_unmeasured, arc_unknown};
}

struct Result
{
  bool pass = true;
  std::vector<std::string> fails;
};

Result check(
  const std::string & sc, double pose_noise_deg, double shift, bool face_filter,
  bool edge_attribution, const std::string & map_dir, const std::string & json_dir, bool verbose)
{
  const auto & s = elevation_map_publisher::scenarios().at(sc);
  MapperParams p;
  p.size_x = s.size_x;
  p.size_y = s.size_y;
  p.origin_x = s.origin_x + shift;
  p.origin_y = s.origin_y + shift;
  p.attitude_sigma = std::max(0.002, pose_noise_deg * kDeg);
  p.face_filter = face_filter;
  p.edge_attribution = edge_attribution;
  ElevationMapper m(p);
  const World w(sc);
  const auto traj = trajectory(sc, w);

  // The sensor: upside down, so ring elevations -52..+7 deg in the body frame.
  std::vector<Eigen::Vector3d> dirs;
  for (int k = 0; k < 40; ++k) {
    const double el = (-52.0 + 59.0 * k / 39.0) * kDeg;
    for (int j = 0; j < 512; ++j) {
      const double az = -M_PI + 2.0 * M_PI * j / 512.0;
      dirs.emplace_back(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
    }
  }
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.02), att(0.0, pose_noise_deg * kDeg);
  double ms = 0.0;
  std::vector<Eigen::Vector3d> pts;
  // Cells reading more than 5 cm above every terrain under them (see the scoring below).
  auto count_phantoms = [&]() {
      int n = 0;
      for (int r = 0; r < m.rows(); ++r) {
        for (int c = 0; c < m.cols(); ++c) {
          const size_t i = m.index(r, c);
          const double e = m.elevation(i);
          if (std::isnan(e) || (m.flags(i) & (ElevationMapper::kAssumed | ElevationMapper::kContact))) {
            continue;
          }
          double hi = -INFINITY;
          for (int a = 0; a < 10; ++a) {
            for (int b = 0; b < 10; ++b) {
              hi = std::max(hi, w.z(m.cell_x(c) + (a - 4.5) * 0.01, m.cell_y(r) + (b - 4.5) * 0.01));
            }
          }
          if (e > hi + 0.05) {
            ++n;
            if (verbose) {
              std::printf("  phantom at planning time at (%.2f, %.2f): measured %.3f, terrain up to %.3f\n",
                m.cell_x(c), m.cell_y(r), e, hi);
            }
          }
        }
      }
      return n;
    };
  int phantom_at_plan = -1;
  long flat_pts = 0, flat_faces = 0;
  std::pair<int, int> at_plan{-1, -1};
  int observed_at_plan = 0;
  for (const auto & pose : traj) {
    if (pose.touchdown) {
      m.add_foot_contact(pose.foot.x(), pose.foot.y(), pose.foot.z());
    }
    const Eigen::Vector3d o = pose.base + Eigen::Vector3d(0, 0, kMast);
    // The mapper gets the scan through a pose with an attitude error (a LIO-like
    // error), so the points are rotated about the sensor.
    const Eigen::Matrix3d err = (Eigen::AngleAxisd(att(rng), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(att(rng), Eigen::Vector3d::UnitY())).toRotationMatrix();
    pts.clear();
    for (const auto & u : dirs) {
      double t;
      if (w.cast(o, u, 40.0, t)) {
        pts.push_back(o + err * ((t + noise(rng)) * u));
      }
    }
    const auto t0 = std::chrono::steady_clock::now();
    m.integrate_scan(o, pts);
    ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // False face points: points on terrain that is flat for 10 cm around them.
    const auto & faces = m.last_faces();
    for (size_t k = 0; k < pts.size() && k < faces.size(); k += 7) {
      const auto & q = pts[k];
      const double zt = w.z(q.x(), q.y());
      if (std::abs(q.z() - zt) > 0.03) {continue;}
      bool flat = true;
      for (int a = -2; a <= 2 && flat; ++a) {
        for (int b = -2; b <= 2 && flat; ++b) {
          flat = std::abs(w.z(q.x() + 0.05 * a, q.y() + 0.05 * b) - zt) < 0.005;
        }
      }
      if (flat) {
        ++flat_pts;
        flat_faces += faces[k];
      }
    }
    if (pose.plan_time) {
      at_plan = plan_coverage(m, kPlans.at(sc));
      phantom_at_plan = count_phantoms();
      if (!map_dir.empty()) {
        write_map(map_dir + "/lidar_map_" + sc + ".txt", m);
      }
      observed_at_plan = 0;
      for (int i = 0; i < m.rows() * m.cols(); ++i) {
        observed_at_plan += !std::isnan(m.elevation(static_cast<size_t>(i)));
      }
    }
  }

  // Truth on the lattice: the published known map (+inf for an obstacle).
  const int R = m.rows(), C = m.cols();
  std::vector<double> truth(static_cast<size_t>(R) * C);
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      double z = static_cast<float>(elevation_map_publisher::world_z(sc, m.cell_x(c), m.cell_y(r)));
      truth[m.index(r, c)] = z >= 5.0 ? INFINITY : z;
    }
  }
  // Within one cell of an edge of the real geometry (the known map rounds edges
  // to the lattice): the terrain over the 3 x 3 cells around it spans > 5 cm.
  const double cell = m.params().resolution;
  auto near_edge = [&](int r, int c) {
      double lo = INFINITY, hi = -INFINITY;
      for (double x = m.cell_x(c) - 1.5 * cell; x <= m.cell_x(c) + 1.5 * cell; x += 0.01) {
        for (double y = m.cell_y(r) - 1.5 * cell; y <= m.cell_y(r) + 1.5 * cell; y += 0.01) {
          const double z = w.z(x, y);
          lo = std::min(lo, z);
          hi = std::max(hi, z);
        }
      }
      return hi - lo > 0.05;
    };
  // What the planner's clearance test sees, roughly: the height, else the bound,
  // else a wall, maxed over the 5 x 5 cells around (it dilates terrain edges by
  // robot_radius + side_clearance, 0.18 m).
  std::vector<double> field(static_cast<size_t>(R) * C), dilated(field.size(), -INFINITY);
  for (size_t i = 0; i < field.size(); ++i) {
    const double e = m.elevation(i), ub = m.upper_bound(i);
    field[i] = !std::isnan(e) ? e : (!std::isnan(ub) ? ub : INFINITY);
  }
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      for (int dr = -2; dr <= 2; ++dr) {
        for (int dc = -2; dc <= 2; ++dc) {
          const int rr = r + dr, cc = c + dc;
          if (rr >= 0 && cc >= 0 && rr < R && cc < C) {
            dilated[m.index(r, c)] = std::max(dilated[m.index(r, c)], field[m.index(rr, cc)]);
          }
        }
      }
    }
  }
  // The terrain under a cell's whole footprint (1 cm samples): a 2.5D cell may
  // read anything from its lowest to its highest terrain (a face inside it makes
  // either defensible), but not above or below all of it.
  auto footprint = [&](int r, int c, double & lo, double & hi) {
      lo = INFINITY;
      hi = -INFINITY;
      for (int a = 0; a < 10; ++a) {
        for (int b = 0; b < 10; ++b) {
          const double z = w.z(m.cell_x(c) + (a - 4.5) * 0.01, m.cell_y(r) + (b - 4.5) * 0.01);
          lo = std::min(lo, z);
          hi = std::max(hi, z);
        }
      }
    };
  int phantom = 0, sunken = 0;
  double worst_phantom = 0.0, worst_sunken = 0.0;
  std::vector<double> err_flat;  // cells whose footprint is one flat surface
  std::vector<double> err, err_all;  // away from edges; everywhere
  int n_obs = 0, false_flat = 0, false_edge = 0, missed = 0, far_err = 0, bound_viol = 0;
  int bounded = 0, unbounded_unseen = 0, finite_truth = 0, unconfirmed = 0, bound_uncovered = 0;
  double worst_bound = 0.0;
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      const size_t i = m.index(r, c);
      const double tz = truth[i];
      finite_truth += std::isfinite(tz);
      const double e = m.elevation(i);
      if (m.measured(i) && !m.confirmed(i)) {++unconfirmed;}
      if (!std::isnan(e)) {
        if (m.flags(i) & (ElevationMapper::kAssumed | ElevationMapper::kContact)) {
          continue;  // not a LiDAR measurement
        }
        ++n_obs;
        double lo, hi;
        footprint(r, c, lo, hi);
        if (e > hi + 0.05) {
          ++phantom;
          worst_phantom = std::max(worst_phantom, e - hi);
          if (verbose) {
            std::printf("  phantom at (%.2f, %.2f): measured %.3f, terrain under it %.3f..%.3f, "
              "flags %d, hits %u\n", m.cell_x(c), m.cell_y(r), e, lo, hi, m.flags(i), m.hits(i));
          }
        }
        if (e < lo - 0.05) {
          ++sunken;
          worst_sunken = std::max(worst_sunken, lo - e);
          if (verbose) {
            std::printf("  sunken at (%.2f, %.2f): measured %.3f, terrain under it %.3f..%.3f\n",
              m.cell_x(c), m.cell_y(r), e, lo, hi);
          }
        }
        if (hi - lo < 0.005) {err_flat.push_back(std::abs(e - lo));}
        const double tc = std::min(tz, kTallCap);
        const double d = e - tc;
        if (std::isfinite(tz)) {
          err_all.push_back(std::abs(d));
          if (!near_edge(r, c)) {err.push_back(std::abs(d));}
        }
        if (d > 0.10) {(near_edge(r, c) ? false_edge : false_flat)++;}
        if (d < -0.10) {++missed;}
        if (std::abs(d) > 0.10 && !near_edge(r, c)) {
          ++far_err;
          if (verbose) {
            std::printf("  far error at (%.2f, %.2f): measured %.3f, truth %.3f, hits %u\n",
              m.cell_x(c), m.cell_y(r), e, tc, m.hits(i));
          }
        }
        continue;
      }
      const double ub = m.upper_bound(i);
      if (std::isnan(ub)) {
        ++unbounded_unseen;
        continue;
      }
      ++bounded;
      const double tc = std::min(tz, kTallCap);
      if (ub < tc - 0.05) {
        const bool covered = dilated[i] >= tc - 0.05;
        if (verbose) {
          std::printf("  bound below the terrain at (%.2f, %.2f): bound %.3f, truth %.3f%s\n",
            m.cell_x(c), m.cell_y(r), ub, tc, covered ? " (a neighbour covers it)" : "");
        }
        ++bound_viol;
        bound_uncovered += !covered;
        worst_bound = std::max(worst_bound, tc - ub);
      }
    }
  }
  const double med = percentile(err, 0.5), p95 = percentile(err, 0.95), mx = percentile(err, 1.0);

  // Along the plan, after flying it.
  const auto [land_unmeasured, arc_unknown] = plan_coverage(m, kPlans.at(sc));

  Result res;
  auto fail = [&](bool bad, const std::string & why) {
      if (bad) {res.pass = false; res.fails.push_back(why);}
    };
  fail(!(med <= 0.02), "median error away from edges > 2 cm");
  fail(!(p95 <= 0.05), "95th percentile error away from edges > 5 cm");
  fail(false_flat > 0, "false obstacles away from edges");
  fail(far_err > 0, "errors over 10 cm away from edges");
  fail(bound_uncovered > 0, "upper bounds below the terrain, not covered by a neighbour");
  fail(land_unmeasured > 0, "landing cells not measured");
  fail(arc_unknown > 0, "cells under the plan's arcs neither measured nor bounded");

  std::printf("== %s (%zu scans, mapper %.1f ms/scan; face points %.1f%%, %ld face hits ignored, "
    "%ld raises credited to a neighbour)\n", sc.c_str(), traj.size(), ms / traj.size(),
    100.0 * m.face_points() / std::max(1L, m.points_classified()), m.face_hits_ignored(),
    m.hits_attributed());
  std::printf("  points on flat terrain taken for face points: %.2f%% of %ld sampled\n",
    100.0 * flat_faces / std::max(1L, flat_pts), flat_pts);
  std::printf("  at planning time (after the warm-up hops): %d cells usable, %d phantom; plan: %d "
    "landing cells unmeasured, %d arc samples neither measured nor bounded\n", observed_at_plan,
    phantom_at_plan, at_plan.first, at_plan.second);
  std::printf("  observed %d of %d finite cells (%.0f%%); bounded unseen %d; unbounded %d; "
    "unconfirmed %d; heights cleared by rays %ld\n",
    n_obs, finite_truth, 100.0 * n_obs / std::max(1, finite_truth), bounded, unbounded_unseen,
    unconfirmed, m.heights_cleared());
  std::printf("  height error away from edges: median %.3f, p95 %.3f, max %.3f m (all cells: "
    "median %.3f, p95 %.3f)\n", med, p95, mx, percentile(err_all, 0.5), percentile(err_all, 0.95));
  std::printf("  > 10 cm: %d above the truth by an edge, %d above it elsewhere, %d below; "
    "%d away from edges\n", false_edge, false_flat, missed, far_err);
  std::printf("  footprint: %d phantom (worst %.3f m above), %d sunken (worst %.3f m below); flat "
    "cells (%zu): p95 %.3f, max %.3f m\n", phantom, worst_phantom, sunken, worst_sunken,
    err_flat.size(), percentile(err_flat, 0.95), percentile(err_flat, 1.0));
  std::printf("  upper bounds below the terrain: %d (worst %.3f m), %d not covered by a neighbour\n",
    bound_viol, worst_bound, bound_uncovered);
  std::printf("  plan, flown: %d landing cells unmeasured, %d arc samples neither measured nor bounded\n",
    land_unmeasured, arc_unknown);
  for (const auto & f : res.fails) {
    std::printf("  FAIL: %s\n", f.c_str());
  }
  if (!json_dir.empty()) {
    const std::string file = json_dir + "/mapper_check_" + sc + ".json";
    write_json(file, sc, m, truth, traj);
    std::printf("  wrote %s\n", file.c_str());
  }
  return res;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> scs;
  double pose_noise = 0.0, shift = 0.0;
  bool face_filter = true, edge_attribution = true;
  std::string json_dir, map_dir;
  bool verbose = false;
  for (int a = 1; a < argc; ++a) {
    if (!std::strcmp(argv[a], "--pose-noise") && a + 1 < argc) {
      pose_noise = std::atof(argv[++a]);
    } else if (!std::strcmp(argv[a], "--shift") && a + 1 < argc) {
      shift = std::atof(argv[++a]);
    } else if (!std::strcmp(argv[a], "--no-face-filter")) {
      face_filter = false;
    } else if (!std::strcmp(argv[a], "--no-edge-attribution")) {
      edge_attribution = false;
    } else if (!std::strcmp(argv[a], "--verbose")) {
      verbose = true;
    } else if (!std::strcmp(argv[a], "--map-out") && a + 1 < argc) {
      map_dir = argv[++a];
    } else if (!std::strcmp(argv[a], "--json") && a + 1 < argc) {
      json_dir = argv[++a];
    } else {
      scs.emplace_back(argv[a]);
    }
  }
  if (scs.empty()) {
    for (const auto & kv : kPlans) {scs.push_back(kv.first);}
  }
  int failures = 0;
  std::vector<std::string> summary;
  for (const auto & sc : scs) {
    if (!kPlans.count(sc)) {
      std::printf("unknown scenario %s\n", sc.c_str());
      return 2;
    }
    const auto r = check(sc, pose_noise, shift, face_filter, edge_attribution, map_dir, json_dir,
      verbose);
    failures += !r.pass;
    summary.push_back(sc + ": " + (r.pass ? "pass" : "FAIL"));
  }
  std::printf("\n== Summary (pose attitude noise %.2f deg, lattice shift %.3f m, face filter %s, "
    "edge attribution %s)\n", pose_noise, shift, face_filter ? "on" : "off",
    edge_attribution ? "on" : "off");
  for (const auto & s : summary) {std::printf("  %s\n", s.c_str());}
  std::printf("%d scenario(s) failed\n", failures);
  return failures ? 1 : 0;
}
