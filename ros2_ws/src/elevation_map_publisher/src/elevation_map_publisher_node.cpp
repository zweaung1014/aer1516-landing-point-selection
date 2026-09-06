// Publishes a known, hand-built 2.5D elevation map as grid_map_msgs/GridMap.
//
// First stage of the ballistic-planning pipeline: scenarios mirror the maps in
// hopcopter-ballistic-planning/maps/ (currently "low_wall" and "flat"), painted
// in world coordinates. Later this node will be replaced/extended to build the
// same map from the LiDAR point cloud, keeping /elevation_map as the boundary.
//
// OBSTACLE encoding on the wire: NaN in the "elevation" layer (grid_map's
// native "no value"; the RViz plugin renders it as a hole). Any painted cell
// whose height >= map.obstacle_height_threshold is emitted as NaN, i.e. an
// infinitely tall column. The planner converts NaN -> Map2D5::OBSTACLE (-1.0).

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

class ElevationMapPublisher : public rclcpp::Node
{
public:
  ElevationMapPublisher()
  : Node("elevation_map_publisher")
  {
    size_x_ = this->declare_parameter("map.size_x", 5.0);
    size_y_ = this->declare_parameter("map.size_y", 5.0);
    resolution_ = this->declare_parameter("map.resolution", 0.1);
    origin_x_ = this->declare_parameter("map.origin_x", -0.5);
    origin_y_ = this->declare_parameter("map.origin_y", -2.5);
    frame_id_ = this->declare_parameter("map.frame_id", std::string("world"));
    default_z_ = this->declare_parameter("map.default_z", 0.0);
    obstacle_height_threshold_ =
      this->declare_parameter("map.obstacle_height_threshold", 5.0);
    scenario_ = this->declare_parameter("scenario", std::string("low_wall"));
    // Wall geometry in WORLD coordinates (y unbounded: full map width).
    wall_height_ = this->declare_parameter("wall.height", 0.4);
    wall_x_min_ = this->declare_parameter("wall.x_min", 1.2);
    wall_x_max_ = this->declare_parameter("wall.x_max", 1.4);
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
      "Publishing '%s' elevation map: %.1fx%.1f m @ %.2f m, origin (%.2f, %.2f)",
      scenario_.c_str(), size_x_, size_y_, resolution_, origin_x_, origin_y_);
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
        RCLCPP_FATAL(this->get_logger(), "Unknown scenario '%s'", scenario_.c_str());
        throw std::runtime_error("unknown scenario");
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
