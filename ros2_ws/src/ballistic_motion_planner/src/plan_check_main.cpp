// Offline check of the momentum_aware planner (stage 3 of
// docs/follow_planner_plan.md). No ROS graph needed. Plans the node's maps and
// three reference maps with the node's defaults (robot_params()), and checks
// every plan:
//  - every landing's tilt, the stop at the goal included, is within tau_max;
//  - flown with chained, unbinned speeds (each leg speed from the previous
//    hop's own landing speed, not its speed bin) for eta 0.71-0.85, every hop
//    keeps the controller's 40 deg leg-angle guard, and its push stays within
//    one push's capacity;
//  - the three plans docs/follow_planner_plan.md expects come out unchanged.
// It also reports search effort against the reference search on the same
// problem. Then it plans on partial LiDAR maps (docs/lidar_mapping_plan.md,
// stage 4): low_wall with the wall's shadow unseen but bounded, lilypads with
// the pit unseen, an unmeasured start (the guard refuses it) and a goal not yet
// seen (a frontier plan). On those, no hop may land on an unseen cell, and every
// arc must clear the bound. Exits 1 if any check fails.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ballistic_motion_planner/ballistic_physics.hpp"
#include "ballistic_motion_planner/frontier.hpp"
#include "ballistic_motion_planner/hopping_astar.hpp"
#include "ballistic_motion_planner/map2d5.hpp"
#include "elevation_map_publisher/scenarios.hpp"

using ballistic::Hop;
using ballistic::Map2D5;
using ballistic::PlannerParams;
using Path = std::vector<std::pair<double, double>>;

namespace
{
constexpr double kGuard = 40.0 * M_PI / 180.0;  // the controller's leg-angle guard (stage 4)

double deg(double rad) {return rad * 180.0 / M_PI;}

// elevation_map_publisher's maps: 5 x 5 m at 0.1 m, origin (-0.5, -2.5); the
// wall as grid_map stores it (float).
Map2D5 node_map(const std::string & scenario)
{
  Map2D5 m(5.0, 5.0, 0.1, -0.5, -2.5);
  if (scenario == "low_wall") {
    m.paint_region(static_cast<double>(0.4f), 2.5, 2.7, std::nan(""), std::nan(""));
  }
  return m;
}

// hopcopter-ballistic-planning/maps/<name>.py, in that repo's frame.
Map2D5 reference_map(const std::string & name)
{
  const double nan = std::nan("");
  Map2D5 m(5.0, 5.0, 0.1);
  if (name == "stairs") {
    for (int i = 0; i < 5; ++i) {
      const double x_min = 2.0 + i * 0.3;
      m.paint_region((i + 1) * 0.6, x_min, i < 4 ? x_min + 0.3 : nan, 1.0, 4.0);
    }
  } else if (name == "low_stairs") {
    m.paint_region(0.2, 2.0, 2.5, 1.5, 3.5);
    m.paint_region(0.4, 2.5, 3.0, 1.5, 3.5);
    m.paint_region(0.6, 3.0, 3.5, 1.5, 3.5);
  } else if (name == "tall_stairs") {
    m.paint_region(0.4, 2.0, 2.6, nan, nan);
    m.paint_region(0.8, 2.6, 3.2, nan, nan);
    m.paint_region(1.2, 3.2, nan, nan, nan);
  }
  return m;
}

struct Case
{
  std::string name;
  std::string map;  // node maps: low_wall, flat; reference maps: ref:<name>
  double sx, sy, gx, gy;
  Path expect;      // the plan docs/follow_planner_plan.md expects; empty: none
  bool reference_config = false;  // ballistic_parity_test's parameters
};

struct Run
{
  std::optional<Path> path;
  std::vector<Hop> hops;
  std::vector<ballistic::Stance> stances;
  long expansions = 0;
  long tilt_rejects = 0, push_rejects = 0;
  bool recheck_failed = false;
  double ms = 0.0;
};

Run run(const Map2D5 & map, const Case & c, const PlannerParams & p)
{
  Run r;
  const auto t0 = std::chrono::steady_clock::now();
  ballistic::HoppingAStarPlanner planner(map, {c.sx, c.sy}, {c.gx, c.gy}, p);
  r.path = planner.plan();
  r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  r.hops = planner.path_hops();
  r.stances = planner.path_stances();
  r.expansions = planner.n_expansions();
  r.tilt_rejects = planner.n_tilt_rejects();
  r.push_rejects = planner.n_push_rejects();
  r.recheck_failed = planner.recheck_failed();
  return r;
}

// Push time [s] that injects e [J/kg] from v_z0 under the planner's push model.
double push_time(double e, double v_z0, const PlannerParams & p)
{
  if (e <= 0.0) {
    return 0.0;
  }
  const double decel = p.g - p.push.accel;
  const double k = p.push.accel - (p.g - p.push.g_coast);
  const double disc = v_z0 * v_z0 - 2.0 * decel * e / k;
  if (disc < 0.0) {
    return NAN;
  }
  return (v_z0 - std::sqrt(disc)) / decel - p.push.t_offset;
}

struct Chain
{
  bool ok = true;
  std::string why;
  double min_leg = 90.0;     // deg
  double max_push = 0.0;     // injection / capacity
  double max_tilt = 0.0;     // deg, flown landings including the stop
};

// Fly the plan with chained, unbinned speeds at this eta. The controller
// (stage 4) re-solves each takeoff from its measured arrival: the planned v_h
// (the planned apex and range), alpha_leg = acos(v_h / v_leg), and a push
// that makes up the planned apex. If the leg alone already reaches the apex,
// it flies the steep no-push root instead.
Chain chain(const PlannerParams & p, const std::vector<Hop> & hops, double eta)
{
  Chain out;
  double v_arr = ballistic::seed_landing_speed(p);
  ballistic::Vec3 v_in{0.0, 0.0, -1.0};
  for (size_t k = 0; k < hops.size(); ++k) {
    const Hop & h = hops[k];
    const double v_leg = std::sqrt(eta) * v_arr;
    char buf[160];
    if (!(h.v_h <= v_leg * std::cos(kGuard))) {
      std::snprintf(buf, sizeof(buf), "hop %zu: leg angle %.1f deg < 40", k + 1,
        h.v_h <= v_leg ? deg(std::acos(h.v_h / v_leg)) : 0.0);
      out.ok = false;
      out.why = buf;
      return out;
    }
    double alpha_leg = std::acos(h.v_h / v_leg);
    double v_h = h.v_h;
    double v_land = h.v_g;
    const double v_z0 = v_leg * std::sin(alpha_leg);
    const double v_z_arc = std::sqrt(std::max(0.0, h.v_s * h.v_s - h.v_h * h.v_h));
    if (v_z0 >= v_z_arc) {
      const auto iv = ballistic::speed_tan_interval(h.X, h.Z, v_leg * v_leg, p.g);
      alpha_leg = std::atan(iv->second);
      v_h = ballistic::xdot(h.X, h.Z, alpha_leg, p.g);
      v_land = std::sqrt(v_leg * v_leg - 2.0 * p.g * h.Z);
    } else {
      const double e = 0.5 * (v_z_arc * v_z_arc - v_z0 * v_z0);
      const double cap = ballistic::push_capacity(v_z0, p.push, p.g);
      out.max_push = std::max(out.max_push, e / cap);
      if (e > cap) {
        std::snprintf(buf, sizeof(buf), "hop %zu: push %.2f J/kg > capacity %.2f", k + 1, e, cap);
        out.ok = false;
        out.why = buf;
        return out;
      }
    }
    out.min_leg = std::min(out.min_leg, deg(alpha_leg));
    const double c = std::cos(alpha_leg);
    const auto st = ballistic::stance(
      v_in, {c * std::cos(h.heading), c * std::sin(h.heading), std::sin(alpha_leg)}, p.k2);
    out.max_tilt = std::max(out.max_tilt, deg(st.tilt));
    const double v_z = std::sqrt(std::max(0.0, v_land * v_land - v_h * v_h));
    v_in = {v_h * std::cos(h.heading), v_h * std::sin(h.heading), -v_z};
    v_arr = v_land;
  }
  const auto stop = ballistic::stance(v_in, {0.0, 0.0, 1.0}, p.k2);
  out.max_tilt = std::max(out.max_tilt, deg(stop.tilt));
  return out;
}

void print_plan(const Run & r, const PlannerParams & p)
{
  std::printf("  path:");
  for (const auto & [x, y] : *r.path) {
    std::printf(" (%.2f,%.2f)", x, y);
  }
  std::printf("\n  hop     X      Z  head  alpha a_leg  v_leg   v_h  J/kg  push_s  apex  tilt\n");
  for (size_t i = 0; i < r.hops.size(); ++i) {
    const auto & h = r.hops[i];
    const double e = h.e_inject / p.mass;
    const double v_z0 = h.v_leg * std::sin(h.alpha_leg);
    std::printf("  %3zu %5.2f %+6.2f %5.0f  %5.1f %5.1f  %5.2f %5.2f %5.2f  %6.3f  %4.2f  %4.1f\n",
      i + 1, h.X, h.Z, deg(h.heading), deg(h.alpha_s), deg(h.alpha_leg), h.v_leg, h.v_h, e,
      push_time(e, v_z0, p), h.apex_drop + h.Z, deg(r.stances[i].tilt));
  }
  std::printf("  stop tilt %.1f deg\n", deg(r.stances.back().tilt));
}
}  // namespace

int main()
{
  std::vector<Case> cases = {
    // The plans docs/follow_planner_plan.md expects (stage 3), as re-planned after
    // the bottom/side clearance split (robot_radius 0.15, side 0.03, bottom 0.2).
    {"low_wall (4,0)", "low_wall", 0, 0, 4.0, 0.0,
      {{0.05, 0.05}, {1.75, 0.05}, {3.35, 0.05}, {4.05, 0.05}}},
    {"low_wall (4,1.5)", "low_wall", 0, 0, 4.0, 1.5,
      {{0.05, 0.05}, {0.75, 0.35}, {1.95, 0.95}, {3.25, 1.25}, {4.05, 1.55}}},
    {"flat (4,0)", "flat", 0, 0, 4.0, 0.0,
      {{0.05, 0.05}, {1.35, 0.05}, {2.65, 0.05}, {4.05, 0.05}}},
    // The rest of the 13-case matrix.
    {"low_wall (4,-1.5)", "low_wall", 0, 0, 4.0, -1.5, {}},
    {"low_wall (4,2.2)", "low_wall", 0, 0, 4.0, 2.2, {}},
    {"low_wall (3.5,-2)", "low_wall", 0, 0, 3.5, -2.0, {}},
    {"low_wall (1.5,2)", "low_wall", 0, 0, 1.5, 2.0, {}},
    {"low_wall (2,-2)", "low_wall", 0, 0, 2.0, -2.0, {}},
    {"low_wall (0,2)", "low_wall", 0, 0, 0.0, 2.0, {}},
    {"low_wall (1,-1)", "low_wall", 0, 0, 1.0, -1.0, {}},
    {"flat (0.5,2)", "flat", 0, 0, 0.5, 2.0, {}},
    {"low_stairs", "ref:low_stairs", 0.5, 2.5, 4.5, 2.5, {}},
    {"stairs", "ref:stairs", 0.5, 4.5, 4.5, 2.5, {}},
    {"tall_stairs", "ref:tall_stairs", 0.5, 2.5, 4.5, 2.5, {}},
    // Search effort on the reference configuration (ballistic_parity_test's).
    {"stairs, reference config", "ref:stairs", 0.5, 4.5, 4.5, 2.5, {}, true},
  };

  PlannerParams reference;  // ballistic_parity_test's configuration
  reference.h_initial = 1.0;
  reference.min_apex = 0.3;
  reference.bottom_clearance = 0.05;
  reference.side_clearance = 0.05;
  reference.V_g_max = std::sqrt(2.0 * reference.g * 2.0);
  reference.V_max = std::sqrt(reference.eta * reference.V_g_max * reference.V_g_max +
    2.0 * reference.e_inject_max / reference.mass);

  int failures = 0;
  std::vector<std::string> summary;
  for (const auto & c : cases) {
    const Map2D5 map = c.map.rfind("ref:", 0) == 0 ? reference_map(c.map.substr(4)) :
      node_map(c.map);
    PlannerParams p = c.reference_config ? reference : ballistic::robot_params();
    p.momentum_aware = true;
    PlannerParams p_ref = p;
    p_ref.momentum_aware = false;
    ballistic::validate_params(p);

    std::printf("\n== %s: (%.2f, %.2f) -> (%.2f, %.2f)\n", c.name.c_str(), c.sx, c.sy, c.gx, c.gy);
    const Run r = run(map, c, p);
    const Run legacy = run(map, c, p_ref);
    char line[256];
    std::snprintf(line, sizeof(line), "search: %ld expansions, %.0f ms (reference search %ld, %.0f ms)"
      "; tilt rejects %ld, push rejects %ld", r.expansions, r.ms, legacy.expansions, legacy.ms,
      r.tilt_rejects, r.push_rejects);
    std::printf("  %s\n", line);
    if (legacy.path) {
      double worst = 0.0;
      for (const auto & st : legacy.stances) {
        worst = std::max(worst, deg(st.tilt));
      }
      std::printf("  reference search: %zu hops, max landing tilt %.1f deg (stop %.1f)\n",
        legacy.hops.size(), worst, deg(legacy.stances.back().tilt));
    } else {
      std::printf("  reference search: no path\n");
    }
    if (!r.path) {
      std::printf("  NO PATH%s\n", r.recheck_failed ? " (failed the exact re-check)" : "");
      summary.push_back(c.name + ": no path");
      if (!c.expect.empty() || r.recheck_failed) {
        ++failures;
      }
      continue;
    }
    print_plan(r, p);

    std::vector<std::string> fails;
    for (size_t i = 0; i < r.stances.size(); ++i) {
      if (r.stances[i].tilt > p.tau_max + 1e-9) {
        fails.push_back("landing " + std::to_string(i) + " tilt " +
          std::to_string(deg(r.stances[i].tilt)));
      }
    }
    if (!c.expect.empty()) {
      bool same = r.path->size() == c.expect.size();
      for (size_t i = 0; same && i < c.expect.size(); ++i) {
        same = std::abs((*r.path)[i].first - c.expect[i].first) < 1e-6 &&
          std::abs((*r.path)[i].second - c.expect[i].second) < 1e-6;
      }
      std::printf("  expected plan: %s\n", same ? "same" : "DIFFERENT");
      if (!same) {
        fails.push_back("not the expected plan");
      }
    }
    // Flown with chained, unbinned speeds.
    double min_leg = 90.0, max_push = 0.0, max_tilt = 0.0;
    for (int i = 0; i <= 14; ++i) {
      const double eta = 0.71 + 0.01 * i;
      const Chain ch = chain(p, r.hops, eta);
      if (!ch.ok) {
        fails.push_back("eta " + std::to_string(eta).substr(0, 4) + ": " + ch.why);
        continue;
      }
      min_leg = std::min(min_leg, ch.min_leg);
      max_push = std::max(max_push, ch.max_push);
      max_tilt = std::max(max_tilt, ch.max_tilt);
    }
    std::printf("  flown, eta 0.71-0.85: leg angle >= %.1f deg, push <= %.0f%% of capacity, "
      "landing tilt <= %.1f deg\n", min_leg, 100.0 * max_push, max_tilt);
    for (const auto & f : fails) {
      std::printf("  FAIL: %s\n", f.c_str());
    }
    failures += static_cast<int>(!fails.empty());
    double worst = 0.0;
    for (const auto & st : r.stances) {
      worst = std::max(worst, deg(st.tilt));
    }
    std::snprintf(line, sizeof(line), "%s: %zu hops, max tilt %.1f, %s", c.name.c_str(),
      r.hops.size(), worst, fails.empty() ? "pass" : "FAIL");
    summary.push_back(line);
  }

  // ---- Partial LiDAR maps (stage 4 of docs/lidar_mapping_plan.md) ----
  {
    const PlannerParams p = ballistic::robot_params();
    // A scenario's known map, as elevation_map_publisher publishes it.
    auto known = [](const std::string & sc) {
        const auto & g = elevation_map_publisher::scenarios().at(sc);
        Map2D5 m(g.size_x, g.size_y, 0.1, g.origin_x, g.origin_y);
        for (int r = 0; r < m.rows(); ++r) {
          for (int c = 0; c < m.cols(); ++c) {
            const auto [x, y] = m.grid_to_world(r, c);
            const double z = static_cast<float>(elevation_map_publisher::world_z(sc, x, y));
            m.at(r, c) = z >= 5.0 ? Map2D5::OBSTACLE : z;
          }
        }
        return m;
      };
    // Mark the cells pred() selects unseen, with the bound bound(x, y), as
    // planner_node's convertMap reads a LiDAR map.
    auto unsee = [](Map2D5 & m, auto pred, auto bound) {
        std::vector<uint8_t> landable(static_cast<size_t>(m.rows()) * m.cols(), 1);
        for (int r = 0; r < m.rows(); ++r) {
          for (int c = 0; c < m.cols(); ++c) {
            const auto [x, y] = m.grid_to_world(r, c);
            if (pred(x, y, m.at(r, c))) {
              landable[static_cast<size_t>(r) * m.cols() + c] = 0;
              m.at(r, c) = bound(x, y);
            }
          }
        }
        m.set_landable(std::move(landable), 0.1, 0.3, 0.08);  // the node's standable.* margins
      };
    auto check_partial = [&](const std::string & name, const Map2D5 & m, double sx, double sy,
      double gx, double gy) {
        std::printf("\n== partial map: %s: (%.2f, %.2f) -> (%.2f, %.2f)\n", name.c_str(), sx, sy, gx, gy);
        Case c{name, "", sx, sy, gx, gy, {}};
        const Run r = run(m, c, p);
        std::vector<std::string> fails;
        if (!r.path) {
          fails.push_back("no path");
        } else {
          print_plan(r, p);
          for (size_t i = 1; i < r.path->size(); ++i) {
            const auto [row, col] = m.world_to_grid((*r.path)[i].first, (*r.path)[i].second);
            if (!m.landable(row, col)) {
              fails.push_back("waypoint " + std::to_string(i) + " lands on an unseen cell");
            }
          }
          for (const auto & st : r.stances) {
            if (st.tilt > p.tau_max + 1e-9) {fails.push_back("landing tilt over the cap");}
          }
        }
        for (const auto & f : fails) {std::printf("  FAIL: %s\n", f.c_str());}
        failures += static_cast<int>(!fails.empty());
        summary.push_back("partial " + name + ": " + (fails.empty() ? std::to_string(r.hops.size()) +
          " hops, pass" : std::string("FAIL")));
      };

    // low_wall seen from the start's apex: the ground behind the wall (x 2.7-3.6)
    // hidden, bounded by the rays that grazed the wall's top, 0.4 m down to 0.
    Map2D5 lw = known("low_wall");
    unsee(lw, [](double x, double, double) {return x >= 2.7 && x < 3.6;},
      [](double x, double) {return 0.4 * (3.6 - x) / 0.9 + 0.05;});
    check_partial("low_wall, shadow unseen (bound 0.45 -> 0.05)", lw, 0.0, 0.0, 4.0, 0.0);

    // low_wall as the 2026-11-07 FAST-LIO run mapped it: the face hits raised the cell
    // in front of the wall (0.35) and the one behind it (0.4), and the shadow beyond is
    // bounded at the wall's height. The wall then reads as a 0.4 m platform from x 2.4
    // to 2.8, and the planner landed on it; the robot came down on its front edge. No
    // hop may land on x 2.3-2.9 now.
    Map2D5 wt = known("low_wall");
    unsee(wt, [](double x, double, double) {return x >= 2.8 && x < 3.6;},
      [](double x, double) {return x < 3.0 ? 0.45 : 0.4 * (3.6 - x) / 0.6 + 0.05;});
    for (int r = 0; r < wt.rows(); ++r) {
      for (int c = 0; c < wt.cols(); ++c) {
        const double x = wt.grid_to_world(r, c).first;
        if (x > 2.4 && x < 2.5) {wt.at(r, c) = 0.35;}
        if (x > 2.7 && x < 2.8) {wt.at(r, c) = 0.4;}
      }
    }
    {
      Case c{"wall top", "", 0.0, 0.0, 4.0, 0.0, {}};
      const Run r = run(wt, c, p);
      bool on_wall = !r.path;
      if (r.path) {
        for (const auto & [x, y] : *r.path) {on_wall = on_wall || (x > 2.3 && x < 2.9);}
        std::printf("\n== partial map: low_wall, wall widened by face hits: %zu hops%s\n", r.hops.size(),
          on_wall ? ", LANDS ON THE WALL" : ", none on the wall");
        print_plan(r, p);
      } else {
        std::printf("\n== partial map: low_wall, wall widened by face hits: NO PATH\n");
      }
      failures += on_wall;
      summary.push_back(std::string("partial low_wall, wall widened by face hits: ") +
        (on_wall ? "FAIL" : "pass"));
    }

    // lilypads from the start platform: the pit (ground at 0) hidden behind the
    // pads, bounded at 1.5 m, below every pad top (1.8-2.2).
    Map2D5 lp = known("lilypads");
    unsee(lp, [](double, double, double z) {return z < 0.5;}, [](double, double) {return 1.5;});
    check_partial("lilypads, pit unseen (bound 1.5)", lp, 0.0, 0.0, 11.75, 0.0);

    // The start guard: an unmeasured start is refused.
    Map2D5 us = known("low_wall");
    unsee(us, [](double x, double y, double) {return std::hypot(x, y) < 0.3;},
      [](double, double) {return 0.4;});
    const bool refused = !ballistic::start_measured(us, 0.0, 0.0) &&
      ballistic::start_measured(us, 1.0, 0.0);
    std::printf("\n== partial map: unmeasured start: the guard %s it\n", refused ? "refuses" : "ACCEPTS");
    failures += !refused;
    summary.push_back(std::string("partial unmeasured start: ") + (refused ? "refused, pass" : "FAIL"));

    // A goal not seen yet: flat with everything past x 2.5 unseen and unbounded
    // (OBSTACLE). The frontier is the reachable cell nearest the goal.
    Map2D5 fr = known("flat");
    unsee(fr, [](double x, double, double) {return x >= 2.5;},
      [](double, double) {return Map2D5::OBSTACLE;});
    const auto standable = fr.standable_mask(p.robot_radius, p.side_clearance, p.steep_grade);
    const auto cand = ballistic::frontier_candidates(fr, standable, {0.0, 0.0}, {4.0, 0.0});
    if (cand.empty()) {
      std::printf("\n== partial map: frontier: no candidate\n  FAIL\n");
      ++failures;
      summary.push_back("partial frontier: FAIL");
    } else {
      const auto [fx, fy] = fr.grid_to_world(cand.front().first, cand.front().second);
      std::printf("\n== partial map: frontier: %zu candidates, nearest the goal (%.2f, %.2f)\n",
        cand.size(), fx, fy);
      check_partial("flat, goal unseen: frontier plan", fr, 0.0, 0.0, fx, fy);
    }
  }

  std::printf("\n== Summary\n");
  for (const auto & s : summary) {
    std::printf("  %s\n", s.c_str());
  }
  std::printf("%d case(s) failed\n", failures);
  return failures ? 1 : 0;
}
