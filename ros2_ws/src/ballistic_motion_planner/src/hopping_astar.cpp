#include "ballistic_motion_planner/hopping_astar.hpp"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "ballistic_motion_planner/ballistic_physics.hpp"

namespace ballistic
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kBinBits = 6;   // speed bins fit in 6 bits (v_g <= V_max ~ 7.35,
constexpr int kBinMax = 63;   // bin <= 30 at speed_bin = 0.25)
constexpr int kMaxBankBins = 8;
constexpr uint64_t kStop = ~0ull;  // momentum_aware: the terminal state, stopped at the goal
}

void validate_params(const PlannerParams & p)
{
  auto fail = [](const std::string & msg) {
      throw std::runtime_error("PlannerParams validation failed: " + msg);
    };

  // Banker's-rounding environment guard: speed_bin_index relies on
  // FE_TONEAREST (round-half-to-even) to match Python round().
  if (std::fegetround() != FE_TONEAREST) {
    fail("FP rounding mode is not FE_TONEAREST; speed binning would diverge from Python");
  }

  if (!(p.mu > 0.0)) {
    fail("MU must be positive");
  }
  if (!(p.steep_grade > p.mu)) {
    fail("steep_grade <= mu: the inflated field, not friction, would silently become "
      "the binding standability limit (plan() returns nothing on graded maps)");
  }

  if (p.bank_bins < 0 || p.bank_bins > kMaxBankBins) {
    fail("bank_bins must be in [0, " + std::to_string(kMaxBankBins) + "]");
  }

  // First-hop derived values.
  const double v_s_min_1 = std::sqrt(p.eta) * seed_landing_speed(p);
  const double v_s_max_1 = std::sqrt(std::min(
      v_s_min_1 * v_s_min_1 + 2.0 * p.e_inject_max / p.mass, p.V_max * p.V_max));
  const double R1 = v_s_max_1 * v_s_max_1 / p.g;

  const double flat_disc = std::pow(p.V_max, 4) - p.g * p.g * R1 * R1;
  if (!(M_PI / 2.0 - std::atan(p.mu) <
    std::atan((p.V_max * p.V_max + std::sqrt(flat_disc)) / (p.g * R1))))
  {
    fail("mu too low: the friction cone's flat-ground floor sits above the "
      "leg-energy ceiling at the first hop's radius — no flat hop is feasible");
  }

  if (!(seed_landing_speed(p) <= p.V_g_max)) {
    fail("h_initial implies a seed landing speed above V_g_max — the robot "
      "could not survive its own initial condition");
  }

  // Some first hop must exist: expand the start once, on flat ground, with the
  // search's own edge test. (The reference checked only the 45 deg hop at the
  // start's full radius against min_apex, which a realistic injection budget or
  // the warm-up seed fails although shorter hops exist.)
  const double half = std::ceil(R1) + 1.0;
  const Map2D5 flat(2.0 * half, 2.0 * half, 0.1);
  HoppingAStarPlanner probe(flat, {half, half}, {half, half}, p);
  if (probe.first_hop_count() == 0) {
    fail("no first hop: from a flat start at the seeded speed the search accepts no hop");
  }
}

PlannerParams robot_params()
{
  PlannerParams p;
  p.momentum_aware = true;
  p.mass = 0.3609;                    // hopcopter's hop_mass
  p.eta = 0.77;                       // stage 0, in place (0.71-0.81)
  p.e_inject_max = 4.0 * p.mass;      // J: about 4 J/kg per hop
  p.min_clearance_gate = 0.3;         // D3
  p.w_energy = 0.84 * 0.8 / p.mass;   // m/J: the reference trade-off per kg (0.84 at 0.8 kg)
  p.h_initial = 0.8;                  // the warm-up apex (hopcopter's desired_rel_z)
  p.min_apex = 0.7;
  p.V_g_max = std::sqrt(2.0 * p.g * 2.0);
  p.V_max = std::sqrt(p.eta * p.V_g_max * p.V_g_max + 2.0 * p.e_inject_max / p.mass);
  return p;
}

double seed_landing_speed(const PlannerParams & p)
{
  if (p.momentum_aware) {
    return std::sqrt(2.0 * p.g * p.h_initial);
  }
  // The reference: the leg alone sustains an apex of h_initial.
  return std::sqrt(2.0 * p.g * p.h_initial / p.eta);
}

HoppingAStarPlanner::HoppingAStarPlanner(
  const Map2D5 & map, std::pair<double, double> start,
  std::pair<double, double> goal, const PlannerParams & params)
: map_(map), p_(params)
{
  v_g_initial_ = seed_landing_speed(p_);
  standable_ = map_.standable_mask(p_.robot_radius, p_.min_clearance_gate, p_.steep_grade);
  normals_ = map_.surface_normals();
  inflated_ = map_.inflated_field(p_.robot_radius + p_.min_clearance_gate, p_.steep_grade);
  start_cell_ = map_.world_to_grid(start.first, start.second);
  goal_cell_ = map_.world_to_grid(goal.first, goal.second);
}

int HoppingAStarPlanner::speed_bin_index(double v_g) const
{
  // Python round() is banker's rounding (half-to-even); std::nearbyint under
  // FE_TONEAREST matches it. std::round (half-away-from-zero) does NOT.
  return static_cast<int>(std::nearbyint(v_g / p_.speed_bin));
}

uint64_t HoppingAStarPlanner::make_state(int row, int col, int bin) const
{
  if (bin > kBinMax) {
    throw std::runtime_error("speed bin exceeds encoding range");
  }
  return (static_cast<uint64_t>(row) * map_.cols() + col) << kBinBits |
         static_cast<uint64_t>(bin);
}

double HoppingAStarPlanner::heuristic(int row, int col) const
{
  const auto [x1, y1] = map_.grid_to_world(row, col);
  const auto [x2, y2] = map_.grid_to_world(goal_cell_.first, goal_cell_.second);
  return std::hypot(x1 - x2, y1 - y2);
}

std::optional<std::vector<std::pair<double, double>>> HoppingAStarPlanner::plan()
{
  if (p_.momentum_aware) {
    return plan_momentum();
  }
  n_expansions_ = 0;
  n_edge_checks_ = 0;
  n_edges_accepted_ = 0;
  path_hops_.clear();
  path_stances_.clear();

  const uint64_t start = make_state(
    start_cell_.first, start_cell_.second, speed_bin_index(v_g_initial_));

  struct Entry
  {
    double f;
    uint64_t counter;
    uint64_t state;
    bool operator>(const Entry & o) const
    {
      if (f != o.f) {return f > o.f;}
      return counter > o.counter;  // FIFO tie-break, like heapq's counter
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_set;
  uint64_t counter = 0;
  open_set.push({0.0, counter, start});

  std::unordered_map<uint64_t, double> g_cost;
  g_cost[start] = 0.0;
  // came_from[state] = (parent state, hop); start has no entry semantics of
  // Python's None — tracked by absence in parent map plus the start sentinel.
  std::unordered_map<uint64_t, std::pair<uint64_t, Hop>> came_from;

  const uint64_t goal_cell_key =
    static_cast<uint64_t>(goal_cell_.first) * map_.cols() + goal_cell_.second;

  std::vector<Neighbor> neighbors;
  while (!open_set.empty()) {
    const Entry e = open_set.top();
    open_set.pop();
    const uint64_t current = e.state;
    const int cur_row = static_cast<int>(current >> kBinBits) / map_.cols();
    const int cur_col = static_cast<int>(current >> kBinBits) % map_.cols();

    if ((current >> kBinBits) == goal_cell_key) {
      // Reconstruct: trace back from goal to start.
      std::vector<std::pair<int, int>> cells;
      std::vector<Hop> hops;
      uint64_t s = current;
      while (true) {
        cells.emplace_back(
          static_cast<int>(s >> kBinBits) / map_.cols(),
          static_cast<int>(s >> kBinBits) % map_.cols());
        const auto it = came_from.find(s);
        if (it == came_from.end()) {break;}
        hops.push_back(it->second.second);
        s = it->second.first;
      }
      std::reverse(cells.begin(), cells.end());
      std::reverse(hops.begin(), hops.end());
      path_hops_ = std::move(hops);

      std::vector<std::pair<double, double>> path;
      path.reserve(cells.size());
      for (const auto & [r, c] : cells) {
        path.push_back(map_.grid_to_world(r, c));
      }
      path_cost_ = g_cost.at(current);
      annotate_path(path);
      return path;
    }

    // Stale-entry skip (no closed set, matching Python).
    const auto git = g_cost.find(current);
    const double g_cur = git == g_cost.end() ? kInf : git->second;
    if (e.f > g_cur + heuristic(cur_row, cur_col)) {
      continue;
    }

    ++n_expansions_;
    generate_hop_neighbors(current, neighbors);
    for (const auto & nb : neighbors) {
      const double tentative_g = g_cur + nb.edge_cost;
      const auto nit = g_cost.find(nb.state);
      if (tentative_g < (nit == g_cost.end() ? kInf : nit->second)) {
        g_cost[nb.state] = tentative_g;
        came_from[nb.state] = {current, nb.hop};
        const int nb_row = static_cast<int>(nb.state >> kBinBits) / map_.cols();
        const int nb_col = static_cast<int>(nb.state >> kBinBits) % map_.cols();
        const double f = tentative_g + heuristic(nb_row, nb_col);
        ++counter;
        open_set.push({f, counter, nb.state});
      }
    }
  }
  return std::nullopt;  // no path found
}

template<typename Visit>
void HoppingAStarPlanner::for_each_target(
  int cur_row, int cur_col, double hop_radius, Visit && visit) const
{
  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);

  std::unordered_set<uint64_t> attempted;
  attempted.insert(static_cast<uint64_t>(cur_row) * map_.cols() + cur_col);

  // Lattice spacing widens as sqrt(radius/ref) beyond the reference radius so
  // candidate count grows ~linearly with radius, not quadratically.
  double step = p_.hop_scan_step;
  if (hop_radius > p_.hop_scan_step_ref_radius) {
    step *= std::sqrt(hop_radius / p_.hop_scan_step_ref_radius);
  }
  const double pad = step * std::sqrt(2.0) / 2.0;
  const double reach = hop_radius + pad;
  const double inner = std::max(0.0, p_.min_hop_radius - pad);
  const double inner2 = inner * inner;

  // Scanline circle fill: every point is inside the disk by construction.
  const int iy_max = static_cast<int>(std::ceil(reach / step));
  for (int iy = -iy_max; iy <= iy_max; ++iy) {
    const double y = iy * step;
    const double rad2 = reach * reach - y * y;
    if (rad2 <= 0.0) {continue;}
    const int ix_max = static_cast<int>(std::floor(std::sqrt(rad2) / step));
    for (int ix = -ix_max; ix <= ix_max; ++ix) {
      const double x = ix * step;
      if (x * x + y * y < inner2) {continue;}

      const double tx = cx + x;
      const double ty = cy + y;
      if (!map_.is_within_bounds(tx, ty)) {continue;}

      const auto [nb_row, nb_col] = map_.world_to_grid(tx, ty);
      const uint64_t cell_key = static_cast<uint64_t>(nb_row) * map_.cols() + nb_col;
      if (!attempted.insert(cell_key).second) {continue;}

      visit(nb_row, nb_col);
    }
  }

  // Goal-snap edge: land exactly on the goal when it's in range.
  const auto [gx, gy] = map_.grid_to_world(goal_cell_.first, goal_cell_.second);
  const uint64_t goal_key =
    static_cast<uint64_t>(goal_cell_.first) * map_.cols() + goal_cell_.second;
  if (std::hypot(gx - cx, gy - cy) <= hop_radius && attempted.count(goal_key) == 0) {
    visit(goal_cell_.first, goal_cell_.second);
  }
}

void HoppingAStarPlanner::generate_hop_neighbors(
  uint64_t state, std::vector<Neighbor> & out)
{
  out.clear();

  const int cur_row = static_cast<int>(state >> kBinBits) / map_.cols();
  const int cur_col = static_cast<int>(state >> kBinBits) % map_.cols();
  const int v_bin = static_cast<int>(state & kBinMax);
  const double v_g_in = v_bin * p_.speed_bin;
  const double hop_radius = max_hop_radius(
    v_g_in, p_.eta, p_.e_inject_max, p_.mass, p_.g, p_.V_max);
  const double current_z = map_.at(cur_row, cur_col);

  for_each_target(cur_row, cur_col, hop_radius, [&](int nb_row, int nb_col) {
      auto edge = validate_and_cost(cur_row, cur_col, current_z, nb_row, nb_col, v_g_in);
      if (edge) {
        edge->second.hop_radius = hop_radius;
        out.push_back({make_state(nb_row, nb_col, speed_bin_index(edge->second.v_g)),
            edge->first, edge->second});
      }
    });
}

std::optional<std::pair<double, Hop>> HoppingAStarPlanner::validate_and_cost(
  int cur_row, int cur_col, double current_z, int nb_row, int nb_col, double v_g_in)
{
  ++n_edge_checks_;
  const double neighbor_z = map_.at(nb_row, nb_col);

  // (a) Stance gate — also rejects OBSTACLE cells outright.
  if (!standable_[static_cast<size_t>(nb_row) * map_.cols() + nb_col]) {
    return std::nullopt;
  }

  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);
  const auto [nx, ny] = map_.grid_to_world(nb_row, nb_col);
  const double X = std::hypot(nx - cx, ny - cy);
  const double Z = neighbor_z - current_z;

  // (b) Energy band + min-apex + BEAM cones + landing-speed cap.
  const double v_s_min = std::sqrt(p_.eta) * v_g_in;
  const auto interval = feasible_alpha_interval(
    X, Z, p_.V_max, p_.g, p_.mu,
    normals_[static_cast<size_t>(cur_row) * map_.cols() + cur_col],
    normals_[static_cast<size_t>(nb_row) * map_.cols() + nb_col],
    std::atan2(ny - cy, nx - cx),
    v_s_min, p_.e_inject_max, p_.mass, p_.min_apex, p_.V_g_max);
  if (!interval) {
    return std::nullopt;
  }
  const auto [alpha_min, alpha_max] = *interval;

  // (c) Clearance floor — also chooses the flown angle.
  const auto alpha_c = clearance_floor_alpha(
    cx, cy, current_z, nx, ny, neighbor_z,
    map_, inflated_, p_.min_clearance_gate, p_.arc_max_step);
  if (!alpha_c) {
    return std::nullopt;  // centreline leaves the map
  }
  if (*alpha_c > alpha_max) {
    return std::nullopt;  // no feasible arc clears the terrain
  }
  const double alpha_floor = std::max(alpha_min, *alpha_c);

  // Least-injection angle within what survives.
  const double alpha_s = std::min(std::max(
      std::atan(min_energy_tan(X, Z)), alpha_floor), alpha_max);

  const double v_s = takeoff_speed(X, Z, alpha_s, p_.g);
  const double v_g = landing_speed(v_s, Z, p_.g);
  const double T = std::tan(alpha_s);
  Hop hop;
  hop.alpha_s = alpha_s;
  hop.v_s = v_s;
  hop.v_g = v_g;
  hop.v_g_in = v_g_in;
  hop.e_inject = injection_energy(v_s, v_s_min, p_.mass);
  hop.apex_drop = (X * T - 2.0 * Z) * (X * T - 2.0 * Z) / (4.0 * (X * T - Z));
  hop.X = X;
  hop.Z = Z;
  hop.hop_radius = 0.0;  // stamped by the caller

  ++n_edges_accepted_;
  return std::make_pair(edge_cost(cur_row, cur_col, nb_row, nb_col, hop), hop);
}

double HoppingAStarPlanner::edge_cost(
  int cur_row, int cur_col, int nb_row, int nb_col, const Hop & hop) const
{
  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);
  const auto [nx, ny] = map_.grid_to_world(nb_row, nb_col);
  const double xy_dist = std::hypot(nx - cx, ny - cy);

  // Momentum thrown away, at the BINNED landing speed (the speed the
  // successor state actually stores). The max(0, ...) is required, not
  // defensive: negative edge costs break A*.
  const double v_g_out = speed_bin_index(hop.v_g) * p_.speed_bin;
  const double ke_in = 0.5 * p_.mass * hop.v_g_in * hop.v_g_in;
  const double ke_out = 0.5 * p_.mass * v_g_out * v_g_out;
  const double e_momentum = std::max(0.0, ke_in - ke_out);

  return xy_dist + p_.w_energy * (hop.e_inject + e_momentum);
}

// ---------------------------------------------------------------------------
// momentum_aware search (stage 3 of docs/follow_planner_plan.md)
// ---------------------------------------------------------------------------

size_t HoppingAStarPlanner::first_hop_count()
{
  const uint64_t start = make_state(
    start_cell_.first, start_cell_.second, speed_bin_index(v_g_initial_));
  std::vector<Neighbor> out;
  if (p_.momentum_aware) {
    momentum_neighbors(start, nullptr, out);
  } else {
    generate_hop_neighbors(start, out);
  }
  return out.size();
}

std::optional<std::vector<std::pair<double, double>>> HoppingAStarPlanner::plan_momentum()
{
  n_expansions_ = 0;
  n_edge_checks_ = 0;
  n_edges_accepted_ = 0;
  n_tilt_rejects_ = 0;
  n_push_rejects_ = 0;
  recheck_failed_ = false;
  path_hops_.clear();
  path_stances_.clear();

  const uint64_t start = make_state(
    start_cell_.first, start_cell_.second, speed_bin_index(v_g_initial_));

  struct Entry
  {
    double f;
    uint64_t counter;
    uint64_t state;
    bool operator>(const Entry & o) const
    {
      if (f != o.f) {return f > o.f;}
      return counter > o.counter;
    }
  };
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open_set;
  uint64_t counter = 0;
  open_set.push({0.0, counter, start});

  std::unordered_map<uint64_t, double> g_cost;
  g_cost[start] = 0.0;
  // A state's arrival direction is that of the hop in its came_from entry: the
  // cheapest path's. The start has none (a vertical arrival).
  std::unordered_map<uint64_t, std::pair<uint64_t, Hop>> came_from;

  const uint64_t goal_cell_key =
    static_cast<uint64_t>(goal_cell_.first) * map_.cols() + goal_cell_.second;

  std::vector<Neighbor> neighbors;
  while (!open_set.empty()) {
    const Entry e = open_set.top();
    open_set.pop();
    const uint64_t current = e.state;

    if (current == kStop) {
      path_cost_ = g_cost.at(kStop);
      auto path = reconstruct(came_from.at(kStop).first, came_from);
      // Re-check with the exact arrival directions: a cheaper path found later
      // can replace a state's arrival after its hops were generated.
      annotate_path(path);
      for (const auto & st : path_stances_) {
        if (st.tilt > p_.tau_max + 1e-9) {
          recheck_failed_ = true;
          path_hops_.clear();
          path_stances_.clear();
          return std::nullopt;
        }
      }
      return path;
    }

    const int cur_row = static_cast<int>(current >> kBinBits) / map_.cols();
    const int cur_col = static_cast<int>(current >> kBinBits) % map_.cols();
    const double g_cur = g_cost.at(current);
    if (e.f > g_cur + heuristic(cur_row, cur_col)) {
      continue;  // stale entry
    }
    ++n_expansions_;

    std::optional<Hop> arrival;
    const auto cf = came_from.find(current);
    if (cf != came_from.end()) {
      arrival = cf->second.second;
    }
    const Hop * arr = arrival ? &*arrival : nullptr;

    if ((current >> kBinBits) == goal_cell_key) {
      // Stop here if this landing can take a vertical takeoff within tau_max.
      const Stance st = stance(
        arrival_velocity(arr, (current & kBinMax) * p_.speed_bin), {0.0, 0.0, 1.0}, p_.k2);
      if (st.tilt <= p_.tau_max) {
        const auto it = g_cost.find(kStop);
        if (it == g_cost.end() || g_cur < it->second) {
          g_cost[kStop] = g_cur;
          came_from[kStop] = {current, Hop{}};
          ++counter;
          open_set.push({g_cur, counter, kStop});
        }
      }
    }

    momentum_neighbors(current, arr, neighbors);
    for (const auto & nb : neighbors) {
      const double tentative_g = g_cur + nb.edge_cost;
      const auto nit = g_cost.find(nb.state);
      if (tentative_g < (nit == g_cost.end() ? kInf : nit->second)) {
        g_cost[nb.state] = tentative_g;
        came_from[nb.state] = {current, nb.hop};
        const int nb_row = static_cast<int>(nb.state >> kBinBits) / map_.cols();
        const int nb_col = static_cast<int>(nb.state >> kBinBits) % map_.cols();
        ++counter;
        open_set.push({tentative_g + heuristic(nb_row, nb_col), counter, nb.state});
      }
    }
  }
  return std::nullopt;  // no path found
}

Vec3 HoppingAStarPlanner::arrival_velocity(const Hop * arrival, double v_g_in) const
{
  if (!arrival) {
    return {0.0, 0.0, -v_g_in};  // the start: hopping in place
  }
  const double c = std::cos(arrival->beta);
  return {v_g_in * c * std::cos(arrival->heading), v_g_in * c * std::sin(arrival->heading),
    -v_g_in * std::sin(arrival->beta)};
}

std::optional<Interval> HoppingAStarPlanner::momentum_interval(
  int cur_row, int cur_col, double current_z, int nb_row, int nb_col, double v_g_in)
{
  ++n_edge_checks_;
  const double neighbor_z = map_.at(nb_row, nb_col);

  // (a) Stance gate — also rejects OBSTACLE cells outright.
  if (!standable_[static_cast<size_t>(nb_row) * map_.cols() + nb_col]) {
    return std::nullopt;
  }

  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);
  const auto [nx, ny] = map_.grid_to_world(nb_row, nb_col);
  const double X = std::hypot(nx - cx, ny - cy);
  const double Z = neighbor_z - current_z;

  // (b) Energy band + min-apex + BEAM cones (takeoff on the leg angle) +
  // landing-speed cap.
  const auto interval = feasible_alpha_interval_vertical_push(
    X, Z, p_.V_max, p_.g, p_.mu,
    normals_[static_cast<size_t>(cur_row) * map_.cols() + cur_col],
    normals_[static_cast<size_t>(nb_row) * map_.cols() + nb_col],
    std::atan2(ny - cy, nx - cx),
    std::sqrt(p_.eta) * v_g_in, p_.e_inject_max, p_.mass, p_.min_apex, p_.V_g_max,
    p_.alpha_leg_min);
  if (!interval) {
    return std::nullopt;
  }

  // (c) Clearance floor.
  const auto alpha_c = clearance_floor_alpha(
    cx, cy, current_z, nx, ny, neighbor_z,
    map_, inflated_, p_.min_clearance_gate, p_.arc_max_step);
  if (!alpha_c || *alpha_c > interval->second) {
    return std::nullopt;
  }
  ++n_edges_accepted_;
  return Interval{std::max(interval->first, *alpha_c), interval->second};
}

void HoppingAStarPlanner::momentum_neighbors(
  uint64_t state, const Hop * arrival, std::vector<Neighbor> & out)
{
  out.clear();

  const int cur_row = static_cast<int>(state >> kBinBits) / map_.cols();
  const int cur_col = static_cast<int>(state >> kBinBits) % map_.cols();
  const int v_bin = static_cast<int>(state & kBinMax);
  const double v_g_in = v_bin * p_.speed_bin;
  const double v_leg = std::sqrt(p_.eta) * v_g_in;
  const double hop_radius = max_hop_radius(
    v_g_in, p_.eta, p_.e_inject_max, p_.mass, p_.g, p_.V_max);
  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);
  const double current_z = map_.at(cur_row, cur_col);
  const Vec3 v_in = arrival_velocity(arrival, v_g_in);
  // A landing on the goal must allow a stop. Taking off vertically tilts the
  // landing by (1 - 1/k2)(90 deg - beta), so its descent angle beta must be at
  // least beta_stop.
  const double beta_stop = 0.5 * M_PI - p_.tau_max / (1.0 - 1.0 / p_.k2);

  for_each_target(cur_row, cur_col, hop_radius, [&](int nb_row, int nb_col) {
      const auto iv = momentum_interval(cur_row, cur_col, current_z, nb_row, nb_col, v_g_in);
      if (!iv) {
        return;
      }
      const auto [nx, ny] = map_.grid_to_world(nb_row, nb_col);
      const double X = std::hypot(nx - cx, ny - cy);
      const double Z = map_.at(nb_row, nb_col) - current_z;
      double alpha_floor = iv->first;
      const double alpha_max = iv->second;
      if (nb_row == goal_cell_.first && nb_col == goal_cell_.second && beta_stop > 0.0) {
        // The landing's descent angle: tan(beta) = tan(alpha) - 2Z/X.
        alpha_floor = std::max(alpha_floor, std::atan(std::tan(beta_stop) + 2.0 * Z / X));
        if (alpha_floor > alpha_max) {
          ++n_tilt_rejects_;
          return;
        }
      }

      // The least-injection angle, and bank_bins steeper ones that each land one
      // speed bin faster: spending more on a short hop can carry the energy a
      // later hop needs, where the least-injection angle alone would burn it
      // with a sideways hop.
      double alphas[1 + kMaxBankBins];
      int n_alphas = 0;
      const double a0 = std::min(std::max(std::atan(min_energy_tan(X, Z)), alpha_floor), alpha_max);
      alphas[n_alphas++] = a0;
      const int b0 = speed_bin_index(landing_speed(takeoff_speed(X, Z, a0, p_.g), Z, p_.g));
      for (int k = 1; k <= p_.bank_bins; ++k) {
        // Just inside bin b0 + k, on the steep root.
        const double v_g = (b0 + k - 0.5) * p_.speed_bin + 1e-9;
        const auto iv_b = speed_tan_interval(X, Z, v_g * v_g + 2.0 * p_.g * Z, p_.g);
        if (!iv_b) {
          continue;
        }
        const double a = std::atan(iv_b->second);
        if (a > a0 && a <= alpha_max) {
          alphas[n_alphas++] = a;
        }
      }

      const double ux = (nx - cx) / X;
      const double uy = (ny - cy) / X;
      for (int i = 0; i < n_alphas; ++i) {
        const double a = alphas[i];
        const double v_h = xdot(X, Z, a, p_.g);
        if (!(v_h <= v_leg)) {
          continue;  // outside the interval's leg-angle bound by rounding
        }
        const double c = v_h / v_leg;
        const double s = std::sqrt(1.0 - c * c);
        const Stance st = stance(v_in, {c * ux, c * uy, s}, p_.k2);
        if (st.tilt > p_.tau_max) {
          ++n_tilt_rejects_;
          continue;
        }
        // One push must make up the planned apex for any arrival speed in this
        // speed bin, so check it at the bin's bottom: the arc's v_h, from less leg.
        const double v_s = takeoff_speed(X, Z, a, p_.g);
        const double v_z_arc2 = v_s * v_s - v_h * v_h;
        const double v_leg_lo = std::sqrt(p_.eta) * (v_g_in - 0.5 * p_.speed_bin);
        const double v_z0_lo2 = v_leg_lo * v_leg_lo - v_h * v_h;
        if (v_z0_lo2 <= 0.0 ||
          0.5 * (v_z_arc2 - v_z0_lo2) > push_capacity(std::sqrt(v_z0_lo2), p_.push, p_.g))
        {
          ++n_push_rejects_;
          continue;
        }

        Hop hop;
        hop.alpha_s = a;
        hop.v_s = v_s;
        hop.v_g = landing_speed(v_s, Z, p_.g);
        hop.v_g_in = v_g_in;
        hop.e_inject = injection_energy(v_s, v_leg, p_.mass);
        const double T = std::tan(a);
        hop.apex_drop = (X * T - 2.0 * Z) * (X * T - 2.0 * Z) / (4.0 * (X * T - Z));
        hop.X = X;
        hop.Z = Z;
        hop.hop_radius = hop_radius;
        hop.heading = std::atan2(ny - cy, nx - cx);
        hop.alpha_leg = std::atan2(s, c);
        hop.v_leg = v_leg;
        hop.v_h = v_h;
        hop.beta = std::atan2(std::sqrt(std::max(0.0, hop.v_g * hop.v_g - v_h * v_h)), v_h);
        out.push_back({make_state(nb_row, nb_col, speed_bin_index(hop.v_g)),
            edge_cost(cur_row, cur_col, nb_row, nb_col, hop), hop});
      }
    });
}

std::vector<std::pair<double, double>> HoppingAStarPlanner::reconstruct(
  uint64_t state,
  const std::unordered_map<uint64_t, std::pair<uint64_t, Hop>> & came_from)
{
  std::vector<std::pair<int, int>> cells;
  std::vector<Hop> hops;
  uint64_t s = state;
  while (true) {
    cells.emplace_back(
      static_cast<int>(s >> kBinBits) / map_.cols(),
      static_cast<int>(s >> kBinBits) % map_.cols());
    const auto it = came_from.find(s);
    if (it == came_from.end()) {break;}
    hops.push_back(it->second.second);
    s = it->second.first;
  }
  std::reverse(cells.begin(), cells.end());
  std::reverse(hops.begin(), hops.end());
  path_hops_ = std::move(hops);

  std::vector<std::pair<double, double>> path;
  path.reserve(cells.size());
  for (const auto & [r, c] : cells) {
    path.push_back(map_.grid_to_world(r, c));
  }
  return path;
}

void HoppingAStarPlanner::annotate_path(const std::vector<std::pair<double, double>> & path)
{
  if (!p_.momentum_aware) {
    // The reference model's leg launches along the arc.
    for (size_t i = 0; i < path_hops_.size(); ++i) {
      Hop & h = path_hops_[i];
      h.heading = std::atan2(
        path[i + 1].second - path[i].second, path[i + 1].first - path[i].first);
      h.alpha_leg = h.alpha_s;
      h.v_leg = h.v_s;
      h.v_h = xdot(h.X, h.Z, h.alpha_s, p_.g);
      h.beta = std::atan2(std::sqrt(std::max(0.0, h.v_g * h.v_g - h.v_h * h.v_h)), h.v_h);
    }
  }
  path_stances_.clear();
  Vec3 v_in{0.0, 0.0, -1.0};  // the start: hopping in place
  for (const auto & h : path_hops_) {
    const double c = std::cos(h.alpha_leg);
    path_stances_.push_back(stance(
        v_in, {c * std::cos(h.heading), c * std::sin(h.heading), std::sin(h.alpha_leg)}, p_.k2));
    const double v_z = std::sqrt(std::max(0.0, h.v_g * h.v_g - h.v_h * h.v_h));
    v_in = {h.v_h * std::cos(h.heading), h.v_h * std::sin(h.heading), -v_z};
  }
  path_stances_.push_back(stance(v_in, {0.0, 0.0, 1.0}, p_.k2));  // the stop at the goal
}

}  // namespace ballistic
