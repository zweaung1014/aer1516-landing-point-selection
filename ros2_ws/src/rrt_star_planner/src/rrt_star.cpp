// ROS 2 + OMPL-based RRT* planner in C++
// Subscribes to LiDAR PointCloud2, builds a 2D occupancy grid (z-band filtered),
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

// TF2 headers for coordinate transforms
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

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
// Supports multi-band detection and temporal smoothing for stable obstacle detection.
class GridMap {
public:
  // Constructor: Initializes the grid with given bounds and resolution.
  // history_size: number of frames for temporal smoothing (default 5)
  // min_bands_required: minimum number of z-bands an obstacle must appear in (default 1)
  // min_history_votes: minimum frames an obstacle must appear in to be marked occupied (default 2)
  GridMap(double min_x, double max_x, double min_y, double max_y, double resolution,
          int history_size = 5, int min_bands_required = 1, int min_history_votes = 2)
  : min_x_(min_x), max_x_(max_x), min_y_(min_y), max_y_(max_y), resolution_(resolution),
    history_size_(history_size), history_index_(0),
    min_bands_required_(min_bands_required), min_history_votes_(min_history_votes) {
    width_ = static_cast<int>(std::ceil((max_x_ - min_x_) / resolution_));
    height_ = static_cast<int>(std::ceil((max_y_ - min_y_) / resolution_));
    const size_t grid_size = static_cast<size_t>(width_ * height_);
    grid_.assign(grid_size, 0);
    
    // Initialize band grids (3 height bands)
    band_low_.assign(grid_size, 0);
    band_mid_.assign(grid_size, 0);
    band_high_.assign(grid_size, 0);
    
    // Initialize history buffer for temporal smoothing
    history_buffer_.resize(history_size_);
    for (auto& frame : history_buffer_) {
      frame.assign(grid_size, 0);
    }
  }

  // Clears all band grids for new frame processing.
  inline void clearBands() {
    std::fill(band_low_.begin(), band_low_.end(), 0);
    std::fill(band_mid_.begin(), band_mid_.end(), 0);
    std::fill(band_high_.begin(), band_high_.end(), 0);
  }

  // Marks the cell at world coordinates (x, y) in the appropriate z-band.
  // z_world: the world z-coordinate of the point
  // band_boundaries: [z_low_max, z_mid_max] - points below z_low_max go to low band,
  //                  between z_low_max and z_mid_max go to mid band, above go to high band
  inline void markOccupiedInBand(double x, double y, double z_world,
                                  double z_low_max, double z_mid_max) {
    int ix, iy;
    worldToGrid(x, y, ix, iy);
    if (inBounds(ix, iy)) {
      const size_t idx = static_cast<size_t>(iy * width_ + ix);
      if (z_world < z_low_max) {
        band_low_[idx] = 1;
      } else if (z_world < z_mid_max) {
        band_mid_[idx] = 1;
      } else {
        band_high_[idx] = 1;
      }
    }
  }

  // Finalizes the current frame: applies multi-band filtering and updates temporal history.
  // Returns the number of occupied cells in the final grid.
  int finalizeFrame() {
    const size_t grid_size = static_cast<size_t>(width_ * height_);
    
    // Step 1: Multi-band detection - only mark cells detected in >= min_bands_required_ bands
    std::vector<uint8_t> current_frame(grid_size, 0);
    for (size_t i = 0; i < grid_size; ++i) {
      int band_count = band_low_[i] + band_mid_[i] + band_high_[i];
      if (band_count >= min_bands_required_) {
        current_frame[i] = 1;
      }
    }
    
    // Step 2: Store current frame in history buffer
    history_buffer_[history_index_] = current_frame;
    history_index_ = (history_index_ + 1) % history_size_;
    
    // Step 3: Temporal smoothing - mark cell occupied if seen in >= min_history_votes_ frames
    int occupied_count = 0;
    for (size_t i = 0; i < grid_size; ++i) {
      int vote_count = 0;
      for (const auto& frame : history_buffer_) {
        vote_count += frame[i];
      }
      if (vote_count >= min_history_votes_) {
        grid_[i] = 1;
        ++occupied_count;
      } else {
        grid_[i] = 0;
      }
    }
    
    return occupied_count;
  }

  // Legacy method for backward compatibility - clears the entire grid.
  inline void clear() {
    std::fill(grid_.begin(), grid_.end(), 0);
  }

  // Legacy method - marks the cell at world coordinates (x, y) as occupied (1).
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
  
  // Multi-band detection: 3 height bands
  std::vector<uint8_t> band_low_;   // Low height band
  std::vector<uint8_t> band_mid_;   // Mid height band  
  std::vector<uint8_t> band_high_;  // High height band
  
  // Temporal smoothing: circular history buffer
  std::vector<std::vector<uint8_t>> history_buffer_;
  int history_size_;
  int history_index_;
  int min_bands_required_;
  int min_history_votes_;
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
    declare_parameter<double>("map.min_x", -2.0);
    declare_parameter<double>("map.max_x", 8.0);
    declare_parameter<double>("map.min_y", -4.0);
    declare_parameter<double>("map.max_y", 4.0);
    declare_parameter<double>("map.resolution", 0.05);
    declare_parameter<double>("map.z_min", 0.15);  // Filter ground returns - matches local planner
    declare_parameter<double>("map.z_max", 3.0);   // Capture more of tall cylinders
    // Multi-band detection: Z boundaries between low/mid/high bands
    declare_parameter<double>("map.z_band_low_max", 0.8);   // Low band: z_min to 0.8m
    declare_parameter<double>("map.z_band_mid_max", 1.5);   // Mid band: 0.8m to 1.5m, High band: 1.5m to z_max
    declare_parameter<int>("map.min_bands_required", 1);    // Require detection in at least 1 band (less aggressive)
    // Temporal smoothing parameters
    declare_parameter<int>("map.history_size", 5);          // Number of frames to keep in history
    declare_parameter<int>("map.min_history_votes", 2);     // Require detection in at least 2 of 5 frames (less aggressive)
    declare_parameter<double>("goal.x", 2.0);
    declare_parameter<double>("goal.y", 2.0);
    declare_parameter<double>("path.z", 0.8);
    declare_parameter<double>("planner.solve_time", 1.0);
    declare_parameter<double>("planner.point_spacing", 0.5);
    // Robot outer radius (meters). Derived from model.sdf.jinja: sqrt(2)*(74.25 mm) ≈ 0.105 m
    declare_parameter<double>("robot.radius", 0.2); //was 0.105    // Self-collision filtering radius to prevent robot position being marked occupied
    declare_parameter<double>("robot.self_collision_radius", 0.15);
    // Load parameter values
    get_parameter("map.min_x", map_min_x_);
    get_parameter("map.max_x", map_max_x_);
    get_parameter("map.min_y", map_min_y_);
    get_parameter("map.max_y", map_max_y_);
    get_parameter("map.resolution", map_resolution_);
    get_parameter("map.z_min", z_min_);
    get_parameter("map.z_max", z_max_);
    get_parameter("map.z_band_low_max", z_band_low_max_);
    get_parameter("map.z_band_mid_max", z_band_mid_max_);
    get_parameter("map.min_bands_required", min_bands_required_);
    get_parameter("map.history_size", history_size_);
    get_parameter("map.min_history_votes", min_history_votes_);
    get_parameter("goal.x", goal_x_);
    get_parameter("goal.y", goal_y_);
    get_parameter("path.z", path_z_);
    get_parameter("planner.solve_time", solve_time_);
    get_parameter("planner.point_spacing", point_spacing_);
    get_parameter("robot.radius", robot_radius_);
    get_parameter("robot.self_collision_radius", self_collision_radius_);

    grid_ = std::make_unique<GridMap>(map_min_x_, map_max_x_, map_min_y_, map_max_y_, map_resolution_,
                                       history_size_, min_bands_required_, min_history_votes_);

    // TF2 setup for coordinate transforms
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>("/ompl_rrt_star_trajectory", 10);
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/rrt_star_grid", 10);
    traj_start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/trajectory_start_position", 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cf_0/lidar/points", rclcpp::SensorDataQoS(),
      std::bind(&RRTStarPlannerNode::cloudCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&RRTStarPlannerNode::goalCallback, this, std::placeholders::_1));

      // Subscribe to robot pose to plan from the current position instead of a fixed origin
      start_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cf_1/pose", rclcpp::SensorDataQoS(),
        std::bind(&RRTStarPlannerNode::startPoseCallback, this, std::placeholders::_1));

    plan_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), //was 1000 ms
      std::bind(&RRTStarPlannerNode::tryPlanAndPublish, this));

    RCLCPP_INFO(get_logger(), "RRT* planner node initialized.");
  }

private:
  // Callback for LiDAR PointCloud2 messages: Transforms points to world frame, filters by z-band, and marks occupied cells.
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
      grid_->clearBands();  // Clear band grids for new frame (not the final grid)

      // Check if transform is available from sensor frame to world frame
      if (!tf_buffer_->canTransform("world", msg->header.frame_id, tf2::TimePointZero, 
                                   std::chrono::milliseconds(100))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Cannot transform from %s to world", msg->header.frame_id.c_str());
        return;
      }

      // Get transform from sensor frame to world frame
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform("world", msg->header.frame_id, tf2::TimePointZero);

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        // Transform point from sensor frame to world frame
        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.header = msg->header;
        point_in.point.x = x;
        point_in.point.y = y;
        point_in.point.z = z;
        
        tf2::doTransform(point_in, point_out, transform);
        
        // Self-collision filtering: Skip points too close to robot's current position
        if (have_current_start_) {
          double dx = point_out.point.x - current_start_x_;
          double dy = point_out.point.y - current_start_y_;
          double distance_to_robot = std::sqrt(dx*dx + dy*dy);
          
          if (distance_to_robot < self_collision_radius_) {
            continue;  // Skip this point - too close to robot
          }
        }
        
        // Filter by z-band and mark occupied in appropriate height band
        if (point_out.point.z >= z_min_ && point_out.point.z <= z_max_) {
          grid_->markOccupiedInBand(point_out.point.x, point_out.point.y, 
                                    point_out.point.z, z_band_low_max_, z_band_mid_max_);
        }
      }

      // Finalize frame: apply multi-band filtering and temporal smoothing
      int occupied_cells = grid_->finalizeFrame();
      RCLCPP_DEBUG(get_logger(), "Frame finalized with %d occupied cells after multi-band + temporal filtering", occupied_cells);

      // Inflate obstacles by robot outer radius to enforce clearance
      grid_->dilateOccupied(robot_radius_);

      have_grid_ = true;
      latest_stamp_ = msg->header.stamp;
      RCLCPP_DEBUG(get_logger(), "Updated occupancy grid from transformed LiDAR points.");

      // Publish debug OccupancyGrid for visualization in RViz
      publishOccupancyGrid();
      
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF2 transform failed: %s", ex.what());
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

  // Callback for current robot pose: Stores the latest x,y to use as the planning start.
  void startPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    try {
      current_start_x_ = static_cast<double>(msg->pose.position.x);
      current_start_y_ = static_cast<double>(msg->pose.position.y);
      have_current_start_ = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error in startPoseCallback: %s", e.what());
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
    start_msg.header.frame_id = "world";
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
    del_marker.header.frame_id = "world";
    del_marker.header.stamp = now();
    del_marker.action = visualization_msgs::msg::Marker::DELETE;
    del_marker.id = PATH_MARKER_ID;
    traj_pub_->publish(del_marker);

    // ADD new marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
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
    grid_msg.header.frame_id = "world";
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
  double z_min_{};
  double z_max_{};
  double z_band_low_max_{};   // Z boundary between low and mid bands
  double z_band_mid_max_{};   // Z boundary between mid and high bands
  int min_bands_required_{};  // Minimum bands for multi-band detection
  int history_size_{};        // Number of frames for temporal smoothing
  int min_history_votes_{};   // Minimum votes for temporal smoothing
  double start_x_{};
  double start_y_{};
  double goal_x_{};
  double goal_y_{};
  double path_z_{};
  double solve_time_{};
  double point_spacing_{};
  double robot_radius_{};
  double self_collision_radius_{};
  // Live start pose
  bool have_current_start_{false};
  double current_start_x_{};
  double current_start_y_{};

  // State
  bool have_grid_{false};
  bool goal_received_{false};
  rclcpp::Time latest_stamp_{};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
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

