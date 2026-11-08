// The LiDAR elevation mapper's core (docs/lidar_mapping_plan.md, "The mapper").
// Pure C++ (Eigen only, no ROS), so the offline check (mapper_check) can drive it
// with synthetic scans.
//
// A 2.5D grid on the planner's cell-centre lattice (Map2D5: x = origin_x +
// (col + 0.5) res, row-major [row][col] = [y][x]). Each cell keeps:
//  - a height, a Kalman estimate with a variance in the style of Fankhauser et
//    al. (ANYmal's elevation_mapping): a hit well above the estimate replaces it
//    (a higher surface), a hit well below it is ignored and inflates the
//    variance (a wall's face under its top), anything in between fuses;
//  - an upper bound: every ray that passes over a cell without hitting it proves
//    the terrain there is lower than the ray. The bound is the third-lowest such
//    ray (plus a margin): one or two rays that pass through a surface (gpu_lidar
//    returns a few) do not lower it. A height that three rays pass clearly below
//    is removed.
//  - the rays that passed above it, which confirm a measured height: a cell that
//    only ever had hits, and fewer than three rays above them, may be the face of
//    something taller than any ray that reached it (a 3 m pole seen from 1.5 m);
//  - foot contacts, written as near-certain heights, and at the first contact a
//    disk of assumed heights around it (the blind zone the robot takes off over).
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace hopcopter_perception
{

struct MapperParams
{
  // Lattice (the planner's: cell centres at origin + (i + 0.5) res).
  double resolution = 0.1;
  double size_x = 5.0, size_y = 5.0;
  double origin_x = -0.5, origin_y = -2.5;

  // Heights.
  double min_range = 0.3;            // hits closer than this are dropped (the LIO blind radius)
  double max_update_range = 8.0;     // hits farther than this do not update heights
  double range_sigma = 0.02;         // LiDAR range noise, 1 sigma [m]
  double attitude_sigma = 0.002;     // pose attitude uncertainty, 1 sigma [rad]
  double min_sigma = 0.01;           // floor on a hit's height sigma [m]
  // Floor on a cell's own sigma: a cell is 10 cm wide, so its hits spread over
  // its slope; without a floor the fused variance collapses and the top of
  // that spread counts as "a higher surface".
  double min_cell_sigma = 0.01;
  double mahalanobis_threshold = 3.0;
  // [m^2] added to a cell's variance for every hit well below it, up to
  // max_inflated_sigma. Keep it small: a wall's top cell gets thousands of hits
  // on its face, and at 1e-4 they inflated it until they fused and sank the wall
  // (2026-11-07 low_wall replay). Wrong estimates recover through below-runs.
  double multi_height_noise = 0.0;
  double max_inflated_sigma = 0.03;  // [m]
  // A cell's height counts once this many hits agree on it (a lone outlier, e.g. a
  // gpu_lidar return 27 cm under the floor, never does).
  int min_hits = 2;
  double contact_sigma = 0.005;      // a foot contact's height sigma [m]

  // Upper bounds (ray carving).
  bool ray_carving = true;
  double carve_max_range = 30.0;     // rays longer than this carve nothing
  double carve_margin = 0.05;        // the bound sits this far above the ray
  double carve_end_skip = 0.15;      // the last metres before the hit, in xy, carve nothing
  double carve_clear_margin = 0.10;  // a height this many rays pass this far below is removed
  int carve_clear_rays = 3;
  // A ray that passes clearly below a height with this many hits (and a seen top) went
  // through a surface: a mixed pixel at a silhouette, which gpu_lidar returns at the
  // background's range, as real LiDARs do at edges. It carves nothing past that cell,
  // and its hit is dropped.
  int trusted_hits = 10;
  // The bound is the third-lowest ray over a cell, and a top counts as seen once three
  // rays have passed above it (fixed at 3 by the storage).
  double carve_center_tolerance = 0.03;  // a ray carves a cell only this close to its centre (xy)

  // A measured height counts only once some ray has passed above it (by this much).
  bool require_top_confirmation = true;
  double confirm_margin = 0.05;

  // Assumed ground around the first foot contact; 0 turns it off.
  double start_disk_radius = 0.6;
};

class ElevationMapper
{
public:
  static constexpr uint8_t kAssumed = 1;   // start disk: replaced by the first hit
  static constexpr uint8_t kContact = 2;   // a foot contact: near-certain

  explicit ElevationMapper(const MapperParams & p);

  const MapperParams & params() const {return p_;}
  int rows() const {return rows_;}
  int cols() const {return cols_;}
  size_t index(int row, int col) const {return static_cast<size_t>(row) * cols_ + col;}
  bool cell_of(double x, double y, int & row, int & col) const;
  double cell_x(int col) const {return p_.origin_x + (col + 0.5) * p_.resolution;}
  double cell_y(int row) const {return p_.origin_y + (row + 0.5) * p_.resolution;}

  // One scan: hit points and the sensor origin, both in the map frame.
  void integrate_scan(const Eigen::Vector3d & origin, const std::vector<Eigen::Vector3d> & points);
  // A foot contact at (x, y) with ground height z (map frame). The first one
  // also fills the start disk.
  void add_foot_contact(double x, double y, double z);
  bool have_contact() const {return have_contact_;}

  // What the planner reads. NaN where the cell has no usable height (never
  // measured, or measured but its top never confirmed).
  double elevation(size_t i) const;
  // The upper bound, NaN where no ray has passed over the cell. Cells whose
  // height is unconfirmed get NaN too: their terrain may be taller than any ray.
  double upper_bound(size_t i) const;
  double variance(size_t i) const {return var_[i];}
  uint32_t hits(size_t i) const {return hits_[i];}
  uint8_t flags(size_t i) const {return flags_[i];}
  // The raw estimate, confirmed or not (NaN never measured), for diagnostics.
  double raw_height(size_t i) const {return h_[i];}
  bool measured(size_t i) const {return !std::isnan(h_[i]);}
  // A usable height: enough hits (min_hits) and its top seen.
  bool confirmed(size_t i) const;
  // Some ray has passed above the height (or it is a contact or assumed).
  bool top_seen(size_t i) const;

  // Counters for the log.
  long scans() const {return n_scans_;}
  long heights_cleared() const {return n_cleared_;}
  long rays_through_surfaces() const {return n_through_;}
  long hits_used() const {return n_hits_;}

private:
  void hit(size_t i, double z, double var);
  // false: the ray passed through a trusted surface (see trusted_hits)
  bool carve(const Eigen::Vector3d & o, const Eigen::Vector3d & p, double range);
  void reset_below(size_t i);

  MapperParams p_;
  int rows_, cols_;
  // The three lowest rays over each cell (ub_[3 i] <= ub_[3 i + 1] <= ub_[3 i + 2]) and the
  // three highest (over_, descending): the bound is the third lowest, the confirmation the
  // third highest, so one or two rays that pass through a surface (gpu_lidar returns
  // some, e.g. 0.75 m past a wall's face) change neither.
  std::vector<double> h_, var_;
  std::vector<float> ub_, over_;
  std::vector<uint32_t> hits_;
  std::vector<uint16_t> fresh_;   // hits since the height was (re)initialized
  std::vector<uint8_t> clear_;    // rays that passed clearly below the height since then
  std::vector<uint8_t> flags_;
  // the current run of hits well below each estimate: count, mean, sum of squares
  std::vector<int> below_n_;
  std::vector<float> below_mean_, below_m2_;
  bool have_contact_ = false;
  long n_scans_ = 0, n_cleared_ = 0, n_hits_ = 0, n_through_ = 0;
  std::vector<char> ray_ok_;
};

}  // namespace hopcopter_perception
