// Planning on a LiDAR map that is still filling in (docs/lidar_mapping_plan.md,
// "Planner changes"): the start guard and the frontier for goals the LiDAR has
// not seen yet. Pure C++, like the rest of ballistic_core.
#pragma once

#include <utility>
#include <vector>

#include "ballistic_motion_planner/map2d5.hpp"

namespace ballistic
{

// The robot's cell holds measured terrain. On a LiDAR map an unmeasured start
// carries an upper bound (or OBSTACLE) as its height, and the search would plan
// every hop from it.
bool start_measured(const Map2D5 & map, double x, double y);

struct FrontierParams
{
  double hop_radius = 1.5;   // [m] the flood's step: standable cells this close are neighbours
  double max_rise = 0.9;     // [m] no step climbs more (an apex of ~1 m is the most one push gives)
  double min_progress = 0.3; // [m] a frontier at least this much nearer the goal than the start
  // [m] a frontier keeps this far from unseen, unbounded (OBSTACLE) cells: nearer, the
  // clearance test's inflation (radius + side clearance + half a cell, 0.25 m) covers
  // the landing and no arc reaches it
  double obstacle_margin = 0.35;
};

// The standable cells a cheap 2D flood reaches from the start, nearest the goal
// first: hop-sized steps between standable cells that climb at most max_rise.
// It ignores clearance and energy, so the search may still fail on a candidate.
// Only cells at least min_progress nearer the goal than the start are returned.
std::vector<std::pair<int, int>> frontier_candidates(
  const Map2D5 & map, const std::vector<uint8_t> & standable,
  std::pair<double, double> start, std::pair<double, double> goal,
  const FrontierParams & p = FrontierParams{});

}  // namespace ballistic
