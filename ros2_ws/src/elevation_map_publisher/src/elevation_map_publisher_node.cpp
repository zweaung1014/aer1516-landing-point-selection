// Publishes a known, hand-built 2.5D elevation map as grid_map_msgs/GridMap.
//
// First stage of the ballistic-planning pipeline: scenarios mirror the maps in
// hopcopter-ballistic-planning/maps/. "low_wall" and "flat" are painted in
// world coordinates. The others ("stairs", "stairs_with_curb", "slope_30deg",
// "cliff_gap", "maze") are painted in their reference file's frame and placed
// so its START sits at world (0, 0), where the robot spawns: origin = -START.
// "lilypads" has no reference file and is painted in world coordinates.
// Each has a Gazebo twin, worlds/crazysim_<scenario>.sdf. stairs,
// stairs_with_curb and cliff_gap are edited from the reference files so the
// robot's planner (robot_params()) can plan them; see CHANGELOG.md. Later this
// node will be replaced/extended to build the same map from the LiDAR point
// cloud, keeping /elevation_map as the boundary.
//
// OBSTACLE encoding on the wire: NaN in the "elevation" layer (grid_map's
// native "no value"; the RViz plugin renders it as a hole). Any painted cell
// whose height >= map.obstacle_height_threshold is emitted as NaN, i.e. an
// infinitely tall column. The planner converts NaN -> Map2D5::OBSTACLE (-1.0).

#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

namespace
{
constexpr double kObstacle = std::numeric_limits<double>::infinity();
constexpr double kNone = std::numeric_limits<double>::quiet_NaN();  // unbounded side

// Geometry defaults and the reference goal, in world coordinates.
struct Scenario
{
  double size_x, size_y, origin_x, origin_y;
  double goal_x, goal_y;
};

const std::map<std::string, Scenario> kScenarios = {
  {"flat", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
  {"low_wall", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
  {"stairs", {5.0, 5.0, -0.5, -4.5, 4.0, -2.0}},
  {"stairs_with_curb", {5.0, 5.0, -0.5, -2.5, 4.0, 0.0}},
  {"slope_30deg", {5.0, 5.0, -0.5, -4.5, 4.25, -2.0}},
  {"cliff_gap", {15.0, 5.0, -0.5, -2.5, 9.5, 0.5}},
  {"maze", {15.0, 5.0, -0.5, -2.5, 14.0, 0.0}},
  {"lilypads", {16.0, 6.0, -2.0, -3.0, 11.75, 0.0}},
};

// paint_region's center-inclusion rule: lo <= v < hi, NaN unbounded.
bool in(double v, double lo, double hi)
{
  const double eps = 1e-9;
  return (std::isnan(lo) || v >= lo - eps) && (std::isnan(hi) || v < hi - eps);
}

// Terrain at the cell center (x, y) in the reference file's frame;
// kObstacle for an infinitely tall column.
double reference_z(const std::string & scenario, double x, double y)
{
  const double slope = std::tan(30.0 * M_PI / 180.0);
  if (scenario == "stairs") {
    // maps/stairs.py with 0.4 m risers on 0.6 m treads (was 0.6 on 0.3 from
    // x 2.0), four steps: the top is 1.6 m from x 3.3 to the map edge.
    if (!in(y, 1.0, 4.0) || x < 1.5) {
      return 0.0;
    }
    return 0.4 * std::min(4, 1 + static_cast<int>(std::floor((x - 1.5 + 1e-9) / 0.6)));
  }
  if (scenario == "stairs_with_curb") {
    // maps/stairs_with_curb.py with 0.15 m curbs on 1.0 m treads (was 0.6 m on
    // 0.4 m from x 2.0). Full width: no bypass.
    const double curb = 0.15;
    if (in(x, 1.2, 1.4)) {return 0.40 + curb;}
    if (in(x, 1.4, 2.4)) {return 0.40;}
    if (in(x, 2.4, 2.6)) {return 0.80 + curb;}
    if (in(x, 2.6, 3.6)) {return 0.80;}
    if (in(x, 3.6, 3.8)) {return 1.20 + curb;}
    if (in(x, 3.8, kNone)) {return 1.20;}
    return 0.0;
  }
  if (scenario == "slope_30deg") {
    // maps/slope_30deg.py, unchanged: 30 deg from x 1.0, platform from 4.5.
    if (!in(y, 1.0, 4.0) || x < 1.0) {
      return 0.0;
    }
    return slope * (std::min(x, 4.5) - 1.0);
  }
  if (scenario == "cliff_gap") {
    // maps/cliff_gap.py with a 1.0 m gap (was 1.5 m: x 6.5-8.0).
    if (!in(y, 1.0, 4.0) || x < 2.0) {
      return 0.0;
    }
    const double platform = 1.80, x_slope_top = 2.0 + 3.118;
    if (x <= x_slope_top) {return slope * (x - 2.0);}
    if (x <= 6.5) {return platform;}
    if (x < 7.5) {return 0.0;}
    if (x <= 11.5) {return platform;}
    if (x <= 11.8) {return platform - 0.6;}
    if (x <= 12.1) {return platform - 1.2;}
    return 0.0;
  }
  if (scenario == "maze") {
    // maps/maze.py: the 17 scattered poles, a 3 m corridor (y 1.0-4.0) between
    // two infinitely tall side walls (y 0.8-1.0 and 4.0-4.2), and the 0.5 m
    // wall at x 7.35-7.65 across the full width. Each pole is the 2 x 2 cells
    // whose centers lie in its 0.2 m footprint (set_obstacle_region rounds out
    // to 3-4 cells).
    static const double poles[][2] = {
      {2.0, 1.5}, {2.5, 3.5}, {3.5, 2.5}, {4.0, 4.2}, {4.5, 0.8}, {5.5, 3.0}, {5.8, 1.5},
      {6.5, 4.0}, {6.7, 2.2}, {8.5, 2.8}, {8.7, 4.2}, {9.5, 1.5}, {10.0, 3.5}, {10.5, 2.0},
      {11.5, 4.0}, {12.0, 1.2}, {12.5, 3.0}};
    if (in(y, 0.8, 1.0) || in(y, 4.0, 4.2)) {
      return kObstacle;  // corridor side walls
    }
    for (const auto & p : poles) {
      if (in(x, p[0] - 0.1, p[0] + 0.1) && in(y, p[1] - 0.1, p[1] + 0.1)) {
        return kObstacle;
      }
    }
    return in(x, 7.35, 7.65) ? 0.5 : 0.0;
  }
  if (scenario == "lilypads") {
    // No reference file: painted in world coordinates (world = reference +
    // the default origin (-2, -3)), the robot starting at world (0, 0). Two
    // 2.0 m platforms 10 m apart (start x -1.5 to 1.0, goal x 11.0 to 12.5)
    // and 20 cylindrical pads between them, 1.8-2.2 m tall, r 0.35-0.75 m.
    // Everything else is ground at 0: a 2 m pit, deliberately finite (an
    // OBSTACLE there would forbid flying over it too). A cell is a pad if its
    // center lies strictly inside the pad's circle; no cell center sits on a
    // rim at these radii.
    const double wx = x - 2.0, wy = y - 3.0;
    if (in(wx, -1.5, 1.0) && in(wy, -2.0, 2.0)) {return 2.0;}   // start platform
    if (in(wx, 11.0, 12.5) && in(wy, -1.5, 1.5)) {return 2.0;}  // goal platform
    // {x, y, radius, top z}, rows from the start platform toward the goal.
    static const double pads[][4] = {
      {2.0, 0.9, 0.60, 1.8},
      {3.4, 1.6, 0.40, 2.0}, {3.5, 0.2, 0.35, 1.9}, {3.3, -1.3, 0.75, 2.2},
      {4.9, 1.7, 0.40, 2.1}, {4.9, 0.4, 0.70, 2.0}, {5.0, -1.3, 0.40, 1.9},
      {6.4, 1.8, 0.40, 1.9}, {6.4, 0.6, 0.40, 2.0}, {6.4, -0.6, 0.40, 2.1}, {6.4, -1.8, 0.40, 2.0},
      {7.8, 1.8, 0.40, 2.0}, {7.8, 0.7, 0.35, 1.9}, {7.8, -0.5, 0.40, 2.0}, {7.8, -1.7, 0.40, 2.1},
      {9.1, 1.2, 0.40, 2.1}, {9.1, 0.0, 0.35, 2.0}, {9.1, -1.2, 0.40, 1.9},
      {10.2, 0.6, 0.35, 2.0}, {10.2, -0.6, 0.35, 2.1}};
    for (const auto & p : pads) {
      const double dx = wx - p[0], dy = wy - p[1];
      if (dx * dx + dy * dy < p[2] * p[2]) {
        return p[3];
      }
    }
    return 0.0;
  }
  throw std::invalid_argument("no reference map for scenario '" + scenario + "'");
}
}  // namespace

class ElevationMapPublisher : public rclcpp::Node
{
public:
  ElevationMapPublisher()
  : Node("elevation_map_publisher")
  {
    scenario_ = this->declare_parameter("scenario", std::string("low_wall"));
    const auto it = kScenarios.find(scenario_);
    if (it == kScenarios.end()) {
      RCLCPP_FATAL(this->get_logger(), "Unknown scenario '%s'", scenario_.c_str());
      throw std::runtime_error("unknown scenario");
    }
    const Scenario & s = it->second;
    size_x_ = this->declare_parameter("map.size_x", s.size_x);
    size_y_ = this->declare_parameter("map.size_y", s.size_y);
    resolution_ = this->declare_parameter("map.resolution", 0.1);
    origin_x_ = this->declare_parameter("map.origin_x", s.origin_x);
    origin_y_ = this->declare_parameter("map.origin_y", s.origin_y);
    frame_id_ = this->declare_parameter("map.frame_id", std::string("world"));
    default_z_ = this->declare_parameter("map.default_z", 0.0);
    obstacle_height_threshold_ =
      this->declare_parameter("map.obstacle_height_threshold", 5.0);
    // Wall geometry in WORLD coordinates (y unbounded: full map width).
    wall_height_ = this->declare_parameter("wall.height", 0.4);
    wall_x_min_ = this->declare_parameter("wall.x_min", 2.5);
    wall_x_max_ = this->declare_parameter("wall.x_max", 2.7);
    publish_rate_ = this->declare_parameter("publish_rate", 1.0);

    buildMap();

    // transient_local so the planner can join late and still get the map; the
    // low-rate republish covers volatile subscribers (e.g. the RViz plugin).
    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      "/elevation_map", rclcpp::QoS(1).transient_local());
    publishMap();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishMap(); });

    RCLCPP_INFO(this->get_logger(),
      "Publishing '%s' elevation map: %.1fx%.1f m @ %.2f m, origin (%.2f, %.2f); "
      "reference goal (%.2f, %.2f)",
      scenario_.c_str(), size_x_, size_y_, resolution_, origin_x_, origin_y_,
      s.goal_x, s.goal_y);
  }

private:
  void buildMap()
  {
    map_ = grid_map::GridMap({"elevation"});
    map_.setFrameId(frame_id_);
    // Center-based geometry; with the defaults the center is (2.0, 0.0) and
    // the outermost cell centers sit at world -0.45 ... 4.45 / -2.45 ... 2.45,
    // matching the planner's Map2D5 cell-center lattice exactly.
    map_.setGeometry(
      grid_map::Length(size_x_, size_y_), resolution_,
      grid_map::Position(origin_x_ + size_x_ / 2.0, origin_y_ + size_y_ / 2.0));

    const double eps = 1e-9;  // paint_region's center-inclusion rule
    for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
      grid_map::Position pos;
      map_.getPosition(*it, pos);
      double z = default_z_;
      if (scenario_ == "low_wall") {
        if (pos.x() >= wall_x_min_ - eps && pos.x() < wall_x_max_ - eps) {
          z = wall_height_;
        }
      } else if (scenario_ != "flat") {
        z = default_z_ + reference_z(scenario_, pos.x() - origin_x_, pos.y() - origin_y_);
      }
      if (z >= obstacle_height_threshold_) {
        z = std::numeric_limits<double>::quiet_NaN();  // OBSTACLE: infinite column
      }
      map_.at("elevation", *it) = static_cast<float>(z);
    }
  }

  void publishMap()
  {
    map_.setTimestamp(this->now().nanoseconds());
    auto msg = grid_map::GridMapRosConverter::toMessage(map_);
    pub_->publish(std::move(msg));
  }

  double size_x_, size_y_, resolution_, origin_x_, origin_y_;
  double default_z_, obstacle_height_threshold_;
  double wall_height_, wall_x_min_, wall_x_max_;
  double publish_rate_;
  std::string frame_id_, scenario_;

  grid_map::GridMap map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ElevationMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
