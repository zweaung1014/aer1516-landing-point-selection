// Non-ROS parity check: builds the low_wall map exactly as
// hopcopter-ballistic-planning/maps/low_wall.py does (origin (0,0), Python
// frame), plans START (0.5, 2.5) -> GOAL (4.5, 2.5) with config.py's shipped
// values, and prints main.py's hop-table format plus the search counters.
// Compare against `python main.py low_wall` in the Python repo.
#include <cmath>
#include <cstdio>

#include "ballistic_motion_planner/hopping_astar.hpp"
#include "ballistic_motion_planner/map2d5.hpp"

int main()
{
  using ballistic::Map2D5;
  using ballistic::PlannerParams;

  // Banker's-rounding probe: nearbyint under FE_TONEAREST must round
  // half-to-even to match Python round().
  if (std::nearbyint(0.5) != 0.0 || std::nearbyint(1.5) != 2.0 ||
    std::nearbyint(2.5) != 2.0)
  {
    std::printf("FAIL: FP environment does not round half-to-even\n");
    return 1;
  }

  PlannerParams p;
  p.g = 9.81;
  p.mass = 0.8;
  p.eta = 0.7;
  p.h_initial = 1.0;
  p.min_apex = 0.3;
  p.e_inject_max = p.mass * p.g * 1.0;                       // 7.848 J
  p.V_g_max = std::sqrt(2.0 * p.g * 2.0);                    // MAX_LANDING_APEX = 2.0
  p.V_max = std::sqrt(p.eta * p.V_g_max * p.V_g_max + 2.0 * p.e_inject_max / p.mass);
  p.speed_bin = 0.25;
  p.mu = 1.2;
  p.w_energy = 0.84;
  p.robot_radius = 0.03;
  p.leg_length = 0.4;
  p.min_clearance_gate = 0.05;
  p.steep_grade = std::tan(60.0 * M_PI / 180.0);
  p.arc_max_step = 0.05;
  p.hop_scan_step = 0.1;
  p.hop_scan_step_ref_radius = 1.0;
  p.min_hop_radius = 0.0;

  ballistic::validate_params(p);
  std::printf("validate_params: OK\n");

  // maps/low_wall.py: 5x5 m @ 0.1 m, wall x in [1.7, 1.9], height 0.4, full y.
  Map2D5 map(5.0, 5.0, 0.1);
  const double nan = std::nan("");
  map.paint_region(0.4, 1.7, 1.9, nan, nan);

  ballistic::HoppingAStarPlanner planner(map, {0.5, 2.5}, {4.5, 2.5}, p);
  auto path = planner.plan();

  if (!path) {
    std::printf("No path found.\n");
    return 1;
  }

  std::printf("Path found with %zu waypoints.\n", path->size());
  std::printf("%3s  %5s %6s  %6s  %5s %5s  %5s  %6s\n",
    "hop", "X", "Z", "alpha", "v_s", "v_g", "drop", "E_inj");
  const auto & hops = planner.path_hops();
  for (size_t i = 0; i < hops.size(); ++i) {
    const auto & h = hops[i];
    std::printf("%3zu  %5.2f %+6.2f  %5.1f°  %5.2f %5.2f  %5.2f  %6.2f J\n",
      i + 1, h.X, h.Z, h.alpha_s * 180.0 / M_PI, h.v_s, h.v_g,
      h.apex_drop, h.e_inject);
  }

  std::printf("\nwaypoints:\n");
  for (const auto & [x, y] : *path) {
    std::printf("  (%.6f, %.6f)\n", x, y);
  }
  std::printf("\nn_expansions=%ld n_edge_checks=%ld n_edges_accepted=%ld\n",
    planner.n_expansions(), planner.n_edge_checks(), planner.n_edges_accepted());

  // High-precision hop dump for the numeric parity diff.
  std::printf("\nprecise hops (alpha_s v_s v_g v_g_in e_inject apex_drop X Z hop_radius):\n");
  for (const auto & h : hops) {
    std::printf("%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n",
      h.alpha_s, h.v_s, h.v_g, h.v_g_in, h.e_inject, h.apex_drop, h.X, h.Z,
      h.hop_radius);
  }
  return 0;
}
