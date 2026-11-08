#include "ballistic_motion_planner/frontier.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace ballistic
{

bool start_measured(const Map2D5 & map, double x, double y)
{
  if (!map.is_within_bounds(x, y)) {
    return false;
  }
  const auto [r, c] = map.world_to_grid(x, y);
  return map.landable(r, c) && map.at(r, c) != Map2D5::OBSTACLE;
}

std::vector<std::pair<int, int>> frontier_candidates(
  const Map2D5 & map, const std::vector<uint8_t> & standable,
  std::pair<double, double> start, std::pair<double, double> goal, const FrontierParams & p)
{
  const int R = map.rows(), C = map.cols();
  const auto idx = [C](int r, int c) {return static_cast<size_t>(r) * C + c;};
  // the disk of offsets one flood step reaches
  std::vector<std::pair<int, int>> disk;
  const int n = static_cast<int>(std::ceil(p.hop_radius / map.resolution()));
  for (int dr = -n; dr <= n; ++dr) {
    for (int dc = -n; dc <= n; ++dc) {
      if ((dr || dc) && std::hypot(dr, dc) * map.resolution() <= p.hop_radius) {
        disk.emplace_back(dr, dc);
      }
    }
  }
  std::vector<uint8_t> obstacle(standable.size(), 0);
  for (int r = 0; r < R; ++r) {
    for (int c = 0; c < C; ++c) {
      obstacle[idx(r, c)] = map.at(r, c) == Map2D5::OBSTACLE;
    }
  }
  const auto near_obstacle = map.dilate_bool(obstacle, p.obstacle_margin);
  std::vector<uint8_t> seen(standable.size(), 0);
  std::deque<std::pair<int, int>> open;
  const auto [rs, cs] = map.world_to_grid(start.first, start.second);
  seen[idx(rs, cs)] = 1;
  open.emplace_back(rs, cs);
  std::vector<std::pair<int, int>> reached;
  while (!open.empty()) {
    const auto [r, c] = open.front();
    open.pop_front();
    const double z = map.at(r, c);
    for (const auto & [dr, dc] : disk) {
      const int rr = r + dr, cc = c + dc;
      if (rr < 0 || cc < 0 || rr >= R || cc >= C) {
        continue;
      }
      const size_t j = idx(rr, cc);
      if (seen[j] || !standable[j] || map.at(rr, cc) - z > p.max_rise) {
        continue;
      }
      seen[j] = 1;
      open.emplace_back(rr, cc);
      reached.emplace_back(rr, cc);
    }
  }
  const auto dist = [&](int r, int c) {
      const auto [x, y] = map.grid_to_world(r, c);
      return std::hypot(x - goal.first, y - goal.second);
    };
  const double d_start = std::hypot(start.first - goal.first, start.second - goal.second);
  std::vector<std::pair<int, int>> out;
  for (const auto & rc : reached) {
    if (dist(rc.first, rc.second) <= d_start - p.min_progress && !near_obstacle[idx(rc.first, rc.second)]) {
      out.push_back(rc);
    }
  }
  std::stable_sort(out.begin(), out.end(), [&](const auto & a, const auto & b) {
      return dist(a.first, a.second) < dist(b.first, b.second);
    });
  return out;
}

}  // namespace ballistic
