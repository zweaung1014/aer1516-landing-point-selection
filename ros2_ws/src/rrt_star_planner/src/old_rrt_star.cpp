// ROS 2 + OMPL-based RRT* planner in C++
// Subscribes to FAST-LIO's registered point cloud, builds a 2D occupancy grid (z-band filtered),
// plans a 2D path using OMPL RRT*, and publishes the trajectory as a Marker (SPHERE_LIST).

#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

// TF2 headers for coordinate transforms
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// PCL headers for voxel grid downsampling
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// OMPL headers
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/geometric/PathGeometric.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

// 2D occupancy grid for collision checking in OMPL planning.
// Stores a grid of cells (0=free, 1=occupied) based on LiDAR points.
class GridMap {
public:
  // Constructor: Initializes the grid with given bounds and resolution.
  GridMap(double min_x, double max_x, double min_y, double max_y, double resolution)
  : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), resolution_(resolution) {
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
    grid_.assign(static_cast<size_t>(width_ * height_), 0);
  }

  // Clears the entire grid, setting all cells to free (0).
  inline void clear() {
    std::fill(grid_.begin(), grid_.end(), 0);
  }

  // Marks the cell at world coordinates (x, y) as occupied (1).
  inline void markOccupied(double x, double y) {
    int ix, iy;
    worldToGrid(x, y, ix, iy);
    if (inBounds(ix, iy)) {
      grid_[static_cast<size_t>(iy * width_ + ix)] = 1;
    }
  }

  // Checks if the cell at world coordinates (x, y) is free (0).
  inline bool isFree(double x, double y) const {
    int ix, iy;
    worldToGrid(x, y, ix, iy);
    if (!inBounds(ix, iy)) {
      return false;
    }
    return grid_[static_cast<size_t>(iy * width_ + ix)] == 0;
  }

  // Converts world coordinates (x, y) to grid indices (ix, iy).
  inline void worldToGrid(double x, double y, int &ix, int &iy) const {
    ix = static_cast<int>((x - min_x_) / resolution_);
    iy = static_cast<int>((y - min_y_) / resolution_);
  }

  // Checks if grid indices (ix, iy) are within bounds.
  inline bool inBounds(int ix, int iy) const {
    return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
  }

  double min_x() const { return min_x_; }
  double max_x() const { return max_x_; }
  double min_y() const { return min_y_; }
  double max_y() const { return max_y_; }
  int width() const { return width_; }
  int height() const { return height_; }

  // Fills an OccupancyGrid-compatible buffer with 0 (free) and 100 (occupied).
  void fillOccupancy(std::vector<int8_t> &out) const {
    out.resize(static_cast<size_t>(width_ * height_));
    for (int iy = 0; iy < height_; ++iy) {
      for (int ix = 0; ix < width_; ++ix) {
        const uint8_t v = grid_[static_cast<size_t>(iy * width_ + ix)];
        out[static_cast<size_t>(iy * width_ + ix)] = (v == 0) ? 0 : 100;
      }
    }
  }

  // Dilates occupied cells by a given radius in meters (disk-shaped inflation).
  void dilateOccupied(double radius_m) {
    if (radius_m <= 0.0) return;
    const int r_cells = static_cast<int>(std::ceil(radius_m / resolution_));
    if (r_cells <= 0) return;
    // Precompute neighbor offsets within a disk of radius r_cells
    std::vector<std::pair<int,int>> offsets;
    offsets.reserve((2*r_cells+1)*(2*r_cells+1));
    const int r2 = r_cells * r_cells;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        if (dx*dx + dy*dy <= r2) {
          offsets.emplace_back(dx, dy);
        }
      }
    }
    std::vector<uint8_t> inflated = grid_; // copy
    for (int iy = 0; iy < height_; ++iy) {
      for (int ix = 0; ix < width_; ++ix) {
        if (grid_[static_cast<size_t>(iy * width_ + ix)] != 0) {
          // mark neighbors occupied
          for (const auto &o : offsets) {
            const int nx = ix + o.first;
            const int ny = iy + o.second;
            if (inBounds(nx, ny)) {
              inflated[static_cast<size_t>(ny * width_ + nx)] = 1;
            }
          }
        }
      }
    }
    grid_.swap(inflated);
  }

private:
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double resolution_;
  int width_;
  int height_;
  std::vector<uint8_t> grid_;
};

// ROS 2 node that implements an OMPL-based RRT* planner.
// Subscribes to LiDAR PointCloud2 and goal poses, builds a 2D occupancy grid,
// plans a path using RRT*, and publishes the trajectory as a Marker.
class RRTStarPlannerNode : public rclcpp::Node {
public:
  // Constructor: Initializes the node, declares parameters, sets up subscribers/publishers, and starts the planning timer.
  RRTStarPlannerNode()
  : Node("rrt_star_planner") {
    // Declare parameters with defaults similar to the Python implementation
    declare_parameter<double>("map.min_x", -5.0);
    declare_parameter<double>("map.max_x", 8.0);
    declare_parameter<double>("map.min_y", -5.0);
    declare_parameter<double>("map.max_y", 5.0);
    declare_parameter<double>("map.resolution", 0.05);
    // Robot leg height with compression margin: 0.144m leg + ~0.1m spring compression + safety buffer
    declare_parameter<double>("robot.leg_height", 0.25);
    declare_parameter<double>("robot.ceiling_height", 1.5);  // Filter points above this height
    declare_parameter<double>("goal.x", 2.0);
    declare_parameter<double>("goal.y", 2.0);
    declare_parameter<double>("path.z", 0.8);
    declare_parameter<double>("planner.solve_time", 1.0);
    declare_parameter<double>("planner.point_spacing", 0.5);
    // Robot outer radius (meters). Derived from model.sdf.jinja: sqrt(2)*(74.25 mm) ≈ 0.105 m
    declare_parameter<double>("robot.radius", 0.35); //was 0.105    // Self-collision filtering radius to prevent robot position being marked occupied
    declare_parameter<double>("robot.self_collision_radius", 0.5); // was 0.15
    // Voxel filter leaf size for downsampling global map (meters)
    declare_parameter<double>("map.voxel_leaf_size", 0.1);
    // Region of interest radius around robot for point cloud cropping (meters)
    declare_parameter<double>("map.roi_radius", 5.0);
    // Grid update rate in Hz (throttles point cloud processing)
    declare_parameter<double>("map.update_rate", 10.0);
    // Load parameter values
    get_parameter("map.min_x", map_min_x_);
    get_parameter("map.max_x", map_max_x_);
    get_parameter("map.min_y", map_min_y_);
    get_parameter("map.max_y", map_max_y_);
    get_parameter("map.resolution", map_resolution_);
    get_parameter("robot.leg_height", leg_height_);
    get_parameter("robot.ceiling_height", ceiling_height_);
    get_parameter("goal.x", goal_x_);
    get_parameter("goal.y", goal_y_);
    get_parameter("path.z", path_z_);
    get_parameter("planner.solve_time", solve_time_);
    get_parameter("planner.point_spacing", point_spacing_);
    get_parameter("robot.radius", robot_radius_);
    get_parameter("robot.self_collision_radius", self_collision_radius_);
    get_parameter("map.voxel_leaf_size", voxel_leaf_size_);
    get_parameter("map.roi_radius", roi_radius_);
    get_parameter("map.update_rate", grid_update_rate_);

    grid_ = std::make_unique<GridMap>(map_min_x_, map_max_x_, map_min_y_, map_max_y_, map_resolution_);

    // TF2 setup for coordinate transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ompl_rrt_star_trajectory", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/rrt_star_grid", 10);
    traj_start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/trajectory_start_position", 10);

    // Subscribe to FAST-LIO's accumulated global map (stable, in camera_init frame)
    // Using /Laser_map instead of /cloud_registered for stable obstacle persistence
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Laser_map", rclcpp::SensorDataQoS(),
      std::bind(&RRTStarPlannerNode::cloudCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&RRTStarPlannerNode::goalCallback, this, std::placeholders::_1));

    // Subscribe to FAST-LIO odometry for robot position in odom frame
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&RRTStarPlannerNode::odomCallback, this, std::placeholders::_1));

    // Initialize robot position (will be updated by odometry callback)
    current_start_x_ = 0.0;
    current_start_y_ = 0.0;
    current_start_z_ = 0.4;
    have_current_start_ = false;  // Wait for odometry

    plan_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), //was 1000 ms
      std::bind(&RRTStarPlannerNode::tryPlanAndPublish, this));

    RCLCPP_INFO(get_logger(), "RRT* planner node initialized (hardware mode: camera_init frame).");
  }

private:
  // Callback for FAST-LIO odometry: Updates robot position in odom frame
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_start_x_ = msg->pose.pose.position.x;
    current_start_y_ = msg->pose.pose.position.y;
    current_start_z_ = msg->pose.pose.position.z;
    have_current_start_ = true;
    
    RCLCPP_DEBUG(get_logger(), "Robot position updated: (%.3f, %.3f, %.3f)",
                 current_start_x_, current_start_y_, current_start_z_);
  }

  // Callback for FAST-LIO global map: Downsamples with voxel filter, crops by ROI, filters by z-band, and marks occupied cells.
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      // Throttle grid updates based on configured rate
      const double update_interval = 1.0 / grid_update_rate_;
      if ((now() - last_grid_update_).seconds() < update_interval) {
        return;  // Skip this update - too soon
      }
      last_grid_update_ = now();

      // Convert ROS message to PCL point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*msg, *cloud_raw);
      const size_t raw_count = cloud_raw->size();

      // Voxel grid downsampling to reduce point count
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
      voxel_filter.setInputCloud(cloud_raw);
      voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_filter.filter(*cloud_filtered);

      RCLCPP_DEBUG(get_logger(), "Voxel downsampled from %zu to %zu points (%.1f%% reduction)",
                   raw_count, cloud_filtered->size(),
                   100.0 * (1.0 - static_cast<double>(cloud_filtered->size()) / raw_count));

      grid_->clear();  // Clear grid for new frame

      const double roi_radius_sq = roi_radius_ * roi_radius_;
      size_t point_count = 0;
      size_t roi_filtered = 0;

      for (const auto& pt : cloud_filtered->points) {
        const float x = pt.x;
        const float y = pt.y;
        const float z = pt.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        // ROI cropping: Skip points outside radius from robot position
        if (have_current_start_) {
          double dx = x - current_start_x_;
          double dy = y - current_start_y_;
          double dist_sq = dx * dx + dy * dy;

          if (dist_sq > roi_radius_sq) {
            ++roi_filtered;
            continue;  // Skip this point - outside ROI
          }

          // Self-collision filtering: Skip points too close to robot
          if (std::sqrt(dist_sq) < self_collision_radius_) {
            continue;
          }
        }

        // Filter ground points using robot-relative threshold and ceiling
        // Points below (robot_z - leg_height + margin) are considered ground and ignored
        // Points above ceiling_height are also ignored
        double ground_z_threshold = current_start_z_ - leg_height_ + 0.05;  // 5cm margin above ground
        if (z >= ground_z_threshold && z <= ceiling_height_) {
          grid_->markOccupied(x, y);
          ++point_count;
        }
      }

      // Inflate obstacles by robot outer radius to enforce clearance
      grid_->dilateOccupied(robot_radius_);

      have_grid_ = true;
      latest_stamp_ = msg->header.stamp;
      RCLCPP_DEBUG(get_logger(), "Updated occupancy grid: %zu points marked (%zu filtered by ROI).",
                   point_count, roi_filtered);

      // Publish debug OccupancyGrid for visualization in RViz
      publishOccupancyGrid();

    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error processing PointCloud2: %s", e.what());
    }
  }

  // Callback for goal pose messages: Updates the goal coordinates and triggers planning if grid is ready.
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
      goal_x_ = static_cast<double>(msg->pose.position.x);
      goal_y_ = static_cast<double>(msg->pose.position.y);
      goal_received_ = true;
      RCLCPP_INFO(get_logger(), "Received new goal: x=%.3f, y=%.3f", goal_x_, goal_y_);
      if (have_grid_) {
        tryPlanAndPublish();
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error in goalCallback: %s", e.what());
    }
  }

  // OMPL state validity checker: Returns true if the 2D state (x,y) is free in the occupancy grid.
  bool isStateValid(const ob::State *state) const {
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    const double x = s->values[0];
    const double y = s->values[1];
    return grid_->isFree(x, y);
  }

  // Attempts to plan a path using OMPL RRT* from start to goal, interpolates it, and publishes as a Marker trajectory.
  void tryPlanAndPublish() {
    if (!have_grid_ || !goal_received_) {
      return;
    }

    // OMPL setup for 2D planning
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, map_min_x_);
    bounds.setHigh(0, map_max_x_);
    bounds.setLow(1, map_min_y_);
    bounds.setHigh(1, map_max_y_);
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([this](const ob::State *s) { return this->isStateValid(s); });
    si->setup();

    ob::ScopedState<> start(space);
    // Require live robot pose for start; no parameter fallback
    if (!have_current_start_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No live robot pose; skipping plan.");
      return;
    }
    const double sx = current_start_x_;
    const double sy = current_start_y_;
    start[0] = sx;
    start[1] = sy;
    ob::ScopedState<> goal(space);
    goal[0] = goal_x_;
    goal[1] = goal_y_;

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTstar>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // Use a timed termination condition for planning duration
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(solve_time_);
    ob::PlannerStatus solved = planner->solve(ptc);

    if (solved) {
      ob::PathPtr path_ptr = pdef->getSolutionPath();
      auto gpath = std::dynamic_pointer_cast<og::PathGeometric>(path_ptr);
      if (!gpath) {
        RCLCPP_WARN(get_logger(), "Solution path is not geometric; skipping publish.");
        return;
      }
      
      // Calculate path length for distance-based interpolation
      double path_length = gpath->length();
      int num_points = static_cast<int>(std::ceil(path_length / point_spacing_));
      num_points = std::max(2, num_points);  // Ensure at least 2 points
      
      try {
        gpath->interpolate(num_points);
        RCLCPP_INFO(get_logger(), "Path length: %.2fm with %d points (avg spacing: %.3fm)", 
                   path_length, num_points, path_length/num_points);
      } catch (...) {
        // ignore interpolation errors
      }

      const auto &states = gpath->getStates();
      if (states.size() <= 1) {
        RCLCPP_WARN(get_logger(), "Planned path has insufficient points; skipping publish.");
        return;
      }

      std::vector<geometry_msgs::msg::Point> points;
      points.reserve(states.size());
      for (const auto &st : states) {
        const auto *rv = st->as<ob::RealVectorStateSpace::StateType>();
        geometry_msgs::msg::Point p;
        p.x = rv->values[0];
        p.y = rv->values[1];
        p.z = path_z_;
        points.push_back(p);
      }

      publishMarkerPath(points);
      publishTrajectoryStartPosition();
      RCLCPP_INFO(get_logger(), "Published trajectory with %zu points (id=%d).", points.size(), PATH_MARKER_ID);
    } else {
      RCLCPP_WARN(get_logger(), "RRT* failed to find a solution.");
    }
  }

  // Publishes current robot position when trajectory is ready for follower-gating
  void publishTrajectoryStartPosition() {
    if (!have_current_start_) {
      return;
    }
    
    geometry_msgs::msg::PoseStamped start_msg;
    start_msg.header.frame_id = planning_frame_;
    start_msg.header.stamp = now();
    start_msg.pose.position.x = current_start_x_;
    start_msg.pose.position.y = current_start_y_;
    start_msg.pose.position.z = path_z_;
    start_msg.pose.orientation.w = 1.0;
    
    traj_start_pub_->publish(start_msg);
    RCLCPP_INFO(get_logger(), "Published trajectory start position: (%.3f, %.3f)", 
                current_start_x_, current_start_y_);
  }

  // Publishes the planned path as a Marker: First deletes the previous marker, then adds a new SPHERE_LIST with the points.
  void publishMarkerPath(const std::vector<geometry_msgs::msg::Point> &points) {
    // DELETE previous marker
    visualization_msgs::msg::Marker del_marker;
    del_marker.header.frame_id = planning_frame_;
    del_marker.header.stamp = now();
    del_marker.action = visualization_msgs::msg::Marker::DELETE;
    del_marker.id = PATH_MARKER_ID;
    traj_pub_->publish(del_marker);

    // ADD new marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = now();
    marker.id = PATH_MARKER_ID;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.points = points;

    traj_pub_->publish(marker);
  }

  // Publishes the current occupancy grid as nav_msgs/OccupancyGrid for debugging.
  void publishOccupancyGrid() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.frame_id = planning_frame_;
    grid_msg.header.stamp = now();
    grid_msg.info.resolution = map_resolution_;
    grid_msg.info.width = static_cast<uint32_t>(grid_->width());
    grid_msg.info.height = static_cast<uint32_t>(grid_->height());
    grid_msg.info.origin.position.x = map_min_x_;
    grid_msg.info.origin.position.y = map_min_y_;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0; // identity

    std::vector<int8_t> data;
    grid_->fillOccupancy(data);
    grid_msg.data = std::move(data);

    grid_pub_->publish(grid_msg);
  }

private:
  // Parameters
  double map_min_x_{};
  double map_max_x_{};
  double map_min_y_{};
  double map_max_y_{};
  double map_resolution_{};
  double leg_height_{};  // Robot leg height with compression margin for ground filtering
  double ceiling_height_{};  // Filter points above this height
  double start_x_{};
  double start_y_{};
  double goal_x_{};
  double goal_y_{};
  double path_z_{};
  double solve_time_{};
  double point_spacing_{};
  double robot_radius_{};
  double self_collision_radius_{};
  double voxel_leaf_size_{};
  double roi_radius_{};
  double grid_update_rate_{};
  // Hardware frames (FAST-LIO)
  const std::string planning_frame_{"camera_init"};  // FAST-LIO world frame
  // Live start pose
  bool have_current_start_{false};
  double current_start_x_{};
  double current_start_y_{};
  double current_start_z_{};

  // State
  bool have_grid_{false};
  bool goal_received_{false};
  rclcpp::Time latest_stamp_{};
  rclcpp::Time last_grid_update_{0, 0, RCL_ROS_TIME};  // For throttling grid updates

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traj_start_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  // Grid
  std::unique_ptr<GridMap> grid_;

  // TF2 for coordinate transforms
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Marker id constant
  static constexpr int PATH_MARKER_ID = 400;
};

 // Main function: Initializes ROS 2, creates the planner node, and spins it.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RRTStarPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
