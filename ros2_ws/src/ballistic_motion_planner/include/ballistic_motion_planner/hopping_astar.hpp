// C++ port of HoppingAStarPlanner (hopcopter-ballistic-planning/
// hopping_astar_planner.py). A* over (cell, speed_bin) states: energy carries
// between hops, so a cell is one node PER arrival speed. Loop order, rounding,
// and tie-breaking replicate the Python exactly so the two produce identical
// paths, hop tables, and expansion counts on the same map (the parity test
// pins this). One known, benign divergence: Python uses np.hypot in the
// heuristic/edge-cost distance and this build of numpy differs from libm's
// hypot by 1 ulp on some inputs, which can perturb expansion ORDER near
// f-cost ties — paths and hop physics stay bit-identical, but n_expansions
// may differ by a handful on large searches (observed: 25790 vs 25791 on the
// stairs map; low_wall matches exactly). The A/B baseline knobs
// (disable_clearance, mu=None, charge_momentum=false) are not ported —
// always the full model.
#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "ballistic_motion_planner/map2d5.hpp"

namespace ballistic
{

struct PlannerParams
{
  // Edge-cost exchange rate, m per J.
  double w_energy = 0.84;
  // Ballistic / clearance.
  double g = 9.81;
  double V_max = 7.345;         // global backstop; derive from the others
  double mu = 1.2;
  double robot_radius = 0.03;
  double leg_length = 0.4;
  double min_clearance_gate = 0.05;
  double steep_grade = 1.7320508075688772;  // tan(60 deg)
  double arc_max_step = 0.05;
  // Energy chain.
  double mass = 0.8;
  double eta = 0.8;
  double e_inject_max = 7.848;
  double min_apex = 0.3;
  double h_initial = 1.0;
  double V_g_max = 7.004;       // derive: sqrt(2 g MAX_LANDING_APEX)
  double speed_bin = 0.25;
  // Neighbor generation.
  double hop_scan_step = 0.1;
  double hop_scan_step_ref_radius = 1.0;
  double min_hop_radius = 0.0;
};

// Port of config.py's import-time asserts. Each guards a silent
// plan-returns-nothing failure; throws std::runtime_error with the reason.
void validate_params(const PlannerParams & p);

struct Hop
{
  double alpha_s;    // takeoff angle, rad from horizontal
  double v_s;        // CoM takeoff speed
  double v_g;        // CoM landing speed (exact, unbinned)
  double v_g_in;     // binned arrival speed at the takeoff cell
  double e_inject;   // propeller energy this hop, J
  double apex_drop;  // apex-to-landing fall, m
  double X;          // horizontal distance
  double Z;          // elevation change
  double hop_radius; // disk radius available at departure
};

class HoppingAStarPlanner
{
public:
  HoppingAStarPlanner(
    const Map2D5 & map, std::pair<double, double> start,
    std::pair<double, double> goal, const PlannerParams & params);

  // Runs A*. Returns the path as world (x, y) waypoints (cell centers; first
  // = start cell, last = goal cell), or nullopt. path_hops() then has one
  // entry per edge. Read the takeoff angle from there — never re-derive it
  // from the endpoints (it depends on the whole path leading up to the hop).
  std::optional<std::vector<std::pair<double, double>>> plan();

  const std::vector<Hop> & path_hops() const {return path_hops_;}
  long n_expansions() const {return n_expansions_;}
  long n_edge_checks() const {return n_edge_checks_;}
  long n_edges_accepted() const {return n_edges_accepted_;}
  double v_g_initial() const {return v_g_initial_;}
  std::pair<int, int> start_cell() const {return start_cell_;}
  std::pair<int, int> goal_cell() const {return goal_cell_;}

private:
  struct Neighbor
  {
    uint64_t state;
    double edge_cost;
    Hop hop;
  };

  int speed_bin_index(double v_g) const;
  uint64_t make_state(int row, int col, int bin) const;

  double heuristic(int row, int col) const;
  void generate_hop_neighbors(uint64_t state, std::vector<Neighbor> & out);
  std::optional<std::pair<double, Hop>> validate_and_cost(
    int cur_row, int cur_col, double current_z, int nb_row, int nb_col,
    double v_g_in);
  double edge_cost(
    int cur_row, int cur_col, int nb_row, int nb_col, const Hop & hop) const;

  const Map2D5 & map_;
  PlannerParams p_;
  double v_g_initial_;
  std::pair<int, int> start_cell_, goal_cell_;

  // Precomputed once per planner, like the Python constructor.
  std::vector<uint8_t> standable_;
  std::vector<Normal> normals_;
  std::vector<double> inflated_;

  std::vector<Hop> path_hops_;
  long n_expansions_ = 0;
  long n_edge_checks_ = 0;
  long n_edges_accepted_ = 0;
};

}  // namespace ballistic
