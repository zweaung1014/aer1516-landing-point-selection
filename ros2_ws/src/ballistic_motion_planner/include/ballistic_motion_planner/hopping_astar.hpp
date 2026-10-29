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
//
// PlannerParams::momentum_aware switches to a search for the robot this repo
// flies (stage 3 of docs/follow_planner_plan.md), which the reference planner
// does not have. It departs from the reference's rule that every hop flies its
// least-injection angle (banking). Off, the port runs unchanged.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ballistic_motion_planner/ballistic_physics.hpp"
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
  double min_clearance_gate = 0.5; // was 0.2
  double steep_grade = 1.7320508075688772;  // tan(60 deg)
  double arc_max_step = 0.05;
  // Energy chain.
  double mass = 0.8;
  double eta = 0.7;
  double e_inject_max = 7.848;
  double min_apex = 0.7;
  double h_initial = 0.8;
  double V_g_max = 7.004;       // derive: sqrt(2 g MAX_LANDING_APEX)
  double speed_bin = 0.25;
  // Neighbor generation.
  double hop_scan_step = 0.1;
  double hop_scan_step_ref_radius = 1.0;
  double min_hop_radius = 0.0;

  // The robot this repo flies (stage 3 of docs/follow_planner_plan.md). false
  // runs the reference search unchanged (ballistic_parity_test pins it). true:
  //  - the push is vertical (feasible_alpha_interval_vertical_push), the leg
  //    angle is at least alpha_leg_min, and no hop injects more than one push
  //    delivers (push);
  //  - the start's arrival speed is the landing from a warm-up apex of
  //    h_initial, sqrt(2 g h_initial): the push, not the leg, sustains it;
  //  - every landing must redirect the robot into its next hop within tau_max
  //    of tilt under hopcopter's stance model. The start is a vertical arrival,
  //    and the goal must allow a stop. Each state keeps the arrival direction of
  //    its cheapest path; the key stays (cell, speed bin);
  //  - besides the least-injection angle, each hop tries bank_bins steeper
  //    angles, each landing one speed bin faster.
  bool momentum_aware = false;
  double alpha_leg_min = 42.0 * M_PI / 180.0;  // the controller's 40 deg guard, plus margin
  PushModel push;
  double k2 = 2.469;                     // stance model, as hopcopter.py builds it
  double tau_max = 20.0 * M_PI / 180.0;  // landing tilt cap (D4)
  int bank_bins = 4;
};

// The robot this repo flies: the node's defaults. PlannerParams' own defaults
// stay the reference planner's.
PlannerParams robot_params();

// Arrival speed the search seeds the start with.
double seed_landing_speed(const PlannerParams & p);

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
  // The takeoff the robot flies. The reference search's leg launches along the
  // arc, so there alpha_leg = alpha_s and v_leg = v_s (filled after the search).
  double heading = 0.0;    // takeoff heading, rad (world, atan2(dy, dx))
  double alpha_leg = 0.0;  // leg launch angle, rad from horizontal
  double v_leg = 0.0;      // leg launch speed
  double v_h = 0.0;        // horizontal speed of the arc
  double beta = 0.0;       // descent angle at the landing, rad below horizontal
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
  // The stances along the path, from its exact headings and descent angles:
  // [0] the start (a vertical arrival) into hop 0, [i] hop i-1's landing into
  // hop i, and [n] the stop at the goal. Computed in both modes.
  const std::vector<Stance> & path_stances() const {return path_stances_;}
  long n_expansions() const {return n_expansions_;}
  long n_edge_checks() const {return n_edge_checks_;}
  long n_edges_accepted() const {return n_edges_accepted_;}
  // momentum_aware: hops dropped for their landing tilt, and for their push
  long n_tilt_rejects() const {return n_tilt_rejects_;}
  long n_push_rejects() const {return n_push_rejects_;}
  // momentum_aware: the search found a path, but its re-check with the exact
  // arrival directions failed, so plan() returned nullopt
  bool recheck_failed() const {return recheck_failed_;}
  double path_cost() const {return path_cost_;}
  double v_g_initial() const {return v_g_initial_;}
  std::pair<int, int> start_cell() const {return start_cell_;}
  std::pair<int, int> goal_cell() const {return goal_cell_;}

  // Hops the search would take from the start (validate_params' probe).
  size_t first_hop_count();

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
  // Every landing cell the hop lattice reaches from (row, col), in the
  // reference order: the scanline disk, then the goal snap.
  template<typename Visit>
  void for_each_target(int row, int col, double hop_radius, Visit && visit) const;
  void generate_hop_neighbors(uint64_t state, std::vector<Neighbor> & out);
  std::optional<std::pair<double, Hop>> validate_and_cost(
    int cur_row, int cur_col, double current_z, int nb_row, int nb_col,
    double v_g_in);
  double edge_cost(
    int cur_row, int cur_col, int nb_row, int nb_col, const Hop & hop) const;

  // momentum_aware search
  std::optional<std::vector<std::pair<double, double>>> plan_momentum();
  // arrival: the hop that reached state, or nullptr for the start
  void momentum_neighbors(uint64_t state, const Hop * arrival, std::vector<Neighbor> & out);
  std::optional<Interval> momentum_interval(
    int cur_row, int cur_col, double current_z, int nb_row, int nb_col, double v_g_in);
  Vec3 arrival_velocity(const Hop * arrival, double v_g_in) const;
  std::vector<std::pair<double, double>> reconstruct(
    uint64_t state,
    const std::unordered_map<uint64_t, std::pair<uint64_t, Hop>> & came_from);
  // Fill the reference search's takeoff fields and path_stances_.
  void annotate_path(const std::vector<std::pair<double, double>> & path);

  const Map2D5 & map_;
  PlannerParams p_;
  double v_g_initial_;
  std::pair<int, int> start_cell_, goal_cell_;

  // Precomputed once per planner, like the Python constructor.
  std::vector<uint8_t> standable_;
  std::vector<Normal> normals_;
  std::vector<double> inflated_;

  std::vector<Hop> path_hops_;
  std::vector<Stance> path_stances_;
  long n_expansions_ = 0;
  long n_edge_checks_ = 0;
  long n_edges_accepted_ = 0;
  long n_tilt_rejects_ = 0;
  long n_push_rejects_ = 0;
  bool recheck_failed_ = false;
  double path_cost_ = 0.0;
};

}  // namespace ballistic
