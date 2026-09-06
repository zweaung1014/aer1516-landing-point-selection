#include "ballistic_motion_planner/hopping_astar.hpp"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
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

  // First-hop derived values (the one deterministic case checkable up front).
  const double v_s_min_1 = std::sqrt(2.0 * p.g * p.h_initial);  // sqrt(eta)*v_g_initial
  const double v_s_max_1 = std::sqrt(std::min(
      v_s_min_1 * v_s_min_1 + 2.0 * p.e_inject_max / p.mass, p.V_max * p.V_max));
  const double R1 = v_s_max_1 * v_s_max_1 / p.g;

  auto tan_hi = [&](double W) -> std::optional<double> {
      const double D = W * W - p.g * p.g * R1 * R1;
      if (D < 0.0) {return std::nullopt;}
      return (W + std::sqrt(D)) / (p.g * R1);
    };

  const double flat_disc = std::pow(p.V_max, 4) - p.g * p.g * R1 * R1;
  if (!(M_PI / 2.0 - std::atan(p.mu) <
    std::atan((p.V_max * p.V_max + std::sqrt(flat_disc)) / (p.g * R1))))
  {
    fail("mu too low: the friction cone's flat-ground floor sits above the "
      "leg-energy ceiling at the first hop's radius — no flat hop is feasible");
  }

  const auto T_ceil = tan_hi(v_s_max_1 * v_s_max_1);
  if (!T_ceil) {
    fail("the start's full-thrust budget cannot reach its own first-hop radius");
  }
  const auto t_lo = tan_hi(v_s_min_1 * v_s_min_1);
  const double T_floor = std::max({
    t_lo ? *t_lo : 0.0,
    4.0 * p.min_apex / R1,
    1.0 / p.mu});
  if (!(T_floor < *T_ceil)) {
    std::ostringstream oss;
    oss << "no takeoff angle survives the first hop: floor "
        << T_floor << " >= ceiling " << *T_ceil
        << " (tan) for a flat hop at the first hop's own radius";
    fail(oss.str());
  }

  if (!(std::sqrt(2.0 * p.g * p.h_initial / p.eta) <= p.V_g_max)) {
    fail("h_initial implies a seed landing speed above V_g_max — the robot "
      "could not survive its own initial condition");
  }
}

HoppingAStarPlanner::HoppingAStarPlanner(
  const Map2D5 & map, std::pair<double, double> start,
  std::pair<double, double> goal, const PlannerParams & params)
: map_(map), p_(params)
{
  v_g_initial_ = std::sqrt(2.0 * p_.g * p_.h_initial / p_.eta);
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
  n_expansions_ = 0;
  n_edge_checks_ = 0;
  n_edges_accepted_ = 0;
  path_hops_.clear();

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

  const auto [cx, cy] = map_.grid_to_world(cur_row, cur_col);
  const double current_z = map_.at(cur_row, cur_col);

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

      auto edge = validate_and_cost(cur_row, cur_col, current_z, nb_row, nb_col, v_g_in);
      if (edge) {
        edge->second.hop_radius = hop_radius;
        out.push_back({make_state(nb_row, nb_col, speed_bin_index(edge->second.v_g)),
            edge->first, edge->second});
      }
    }
  }

  // Goal-snap edge: land exactly on the goal when it's in range.
  const auto [gx, gy] = map_.grid_to_world(goal_cell_.first, goal_cell_.second);
  const uint64_t goal_key =
    static_cast<uint64_t>(goal_cell_.first) * map_.cols() + goal_cell_.second;
  if (std::hypot(gx - cx, gy - cy) <= hop_radius && attempted.count(goal_key) == 0) {
    auto edge = validate_and_cost(
      cur_row, cur_col, current_z, goal_cell_.first, goal_cell_.second, v_g_in);
    if (edge) {
      edge->second.hop_radius = hop_radius;
      out.push_back({make_state(goal_cell_.first, goal_cell_.second,
          speed_bin_index(edge->second.v_g)), edge->first, edge->second});
    }
  }
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

}  // namespace ballistic
