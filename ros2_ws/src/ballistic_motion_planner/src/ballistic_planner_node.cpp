// ROS 2 node wrapping the ballistic hopping A* core.
//
// Subscribes:  /elevation_map  (grid_map_msgs/GridMap, transient_local)
//              /cf_1/pose      (geometry_msgs/PoseStamped, live start)
//              /goal_pose      (geometry_msgs/PoseStamped, one-shot trigger)
// Publishes:   /ballistic_trajectory       (visualization_msgs/Marker,
//                DELETE then ADD SPHERE_LIST, id=400 — the hopcopter contract;
//                point z = terrain elevation + output.hover_offset)
//              /trajectory_start_position  (geometry_msgs/PoseStamped)
// Plans exactly once per /goal_pose message; there is no replan timer.
// CSVs (path + per-hop energy chain) go to data/data_ballistic_planner/.

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include "ballistic_motion_planner/hopping_astar.hpp"
#include "ballistic_motion_planner/map2d5.hpp"

namespace
{
constexpr int kPathMarkerId = 400;  // hopcopter accepts only this id
}

class BallisticPlannerNode : public rclcpp::Node
{
public:
  BallisticPlannerNode()
  : Node("ballistic_planner")
  {
    // Physics / search parameters (defaults = config.py's shipped values;
    // v_g_max and V_max are derived from formulas so parity stays exact).
    p_.g = declare_parameter("physics.g", 9.81);
    p_.mass = declare_parameter("physics.mass", 0.8);
    // 0.8 = ~80% of apex PE retained per hop (sim-measured); 20% loss.
    p_.eta = declare_parameter("physics.eta", 0.8);
    p_.h_initial = declare_parameter("physics.h_initial", 1.0);
    p_.min_apex = declare_parameter("physics.min_apex", 0.3);
    p_.e_inject_max = declare_parameter("physics.e_inject_max", p_.mass * p_.g * 1.0);
    p_.mu = declare_parameter("physics.mu", 1.2);
    p_.V_g_max = declare_parameter("physics.v_g_max", std::sqrt(2.0 * p_.g * 2.0));
    p_.V_max = declare_parameter(
      "physics.v_max",
      std::sqrt(p_.eta * p_.V_g_max * p_.V_g_max + 2.0 * p_.e_inject_max / p_.mass));
    p_.speed_bin = declare_parameter("search.speed_bin", 0.25);
    p_.w_energy = declare_parameter("search.w_energy", 0.84);
    p_.hop_scan_step = declare_parameter("search.hop_scan_step", 0.1);
    p_.hop_scan_step_ref_radius = declare_parameter("search.hop_scan_step_ref_radius", 1.0);
    p_.min_hop_radius = declare_parameter("search.min_hop_radius", 0.0);
    p_.robot_radius = declare_parameter("robot.radius", 0.03);
    p_.leg_length = declare_parameter("robot.leg_length", 0.4);
    p_.min_clearance_gate = declare_parameter("robot.min_clearance", 0.05);
    p_.steep_grade = declare_parameter("clearance.steep_grade", std::tan(60.0 * M_PI / 180.0));
    p_.arc_max_step = declare_parameter("clearance.arc_max_step", 0.05);
    hover_offset_ = declare_parameter("output.hover_offset", 0.8);
    const auto map_topic = declare_parameter("topics.map", std::string("/elevation_map"));
    const auto traj_topic = declare_parameter("topics.trajectory", std::string("/ballistic_trajectory"));

    // Each check guards a silent plan-returns-nothing failure.
    ballistic::validate_params(p_);

    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(traj_topic, 10);
    start_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/trajectory_start_position", 10);

    map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic, rclcpp::QoS(1).transient_local(),
      [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) {
        latest_map_msg_ = msg;
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf_1/pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        robot_x_ = msg->pose.position.x;
        robot_y_ = msg->pose.position.y;
        have_pose_ = true;
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goalCallback(*msg);
      });

    csv_dir_ = "/home/zweminhtetaung/CrazySim/data/data_ballistic_planner";
    std::error_code ec;
    std::filesystem::create_directories(csv_dir_, ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Could not create CSV dir %s: %s",
        csv_dir_.c_str(), ec.message().c_str());
    }

    RCLCPP_INFO(get_logger(),
      "Ballistic planner ready. Waiting for %s and /cf_1/pose; "
      "plans once per /goal_pose.", map_topic.c_str());
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped & msg)
  {
    try {
      if (!latest_map_msg_) {
        RCLCPP_WARN(get_logger(), "Goal received but no elevation map yet — ignoring. "
          "Re-publish the goal once the map is up.");
        return;
      }
      if (!have_pose_) {
        RCLCPP_WARN(get_logger(), "Goal received but no /cf_1/pose yet — ignoring. "
          "Re-publish the goal once the pose is up.");
        return;
      }

      const auto t0 = std::chrono::steady_clock::now();
      const auto map = convertMap(*latest_map_msg_);

      const double gx = msg.pose.position.x;
      const double gy = msg.pose.position.y;
      if (!map->is_within_bounds(robot_x_, robot_y_)) {
        RCLCPP_ERROR(get_logger(),
          "Robot position (%.2f, %.2f) is outside the map "
          "[%.2f, %.2f] x [%.2f, %.2f] — refusing to plan.",
          robot_x_, robot_y_, map->origin_x(), map->origin_x() + map->size_x(),
          map->origin_y(), map->origin_y() + map->size_y());
        return;
      }
      if (!map->is_within_bounds(gx, gy)) {
        RCLCPP_ERROR(get_logger(),
          "Goal (%.2f, %.2f) is outside the map — refusing to plan.", gx, gy);
        return;
      }

      ballistic::HoppingAStarPlanner planner(*map, {robot_x_, robot_y_}, {gx, gy}, p_);
      const auto t1 = std::chrono::steady_clock::now();
      auto path = planner.plan();
      const auto t2 = std::chrono::steady_clock::now();

      const double prep_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      const double plan_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

      if (!path) {
        RCLCPP_ERROR(get_logger(),
          "No path found from (%.2f, %.2f) to (%.2f, %.2f) "
          "[prep %.1f ms, search %.1f ms, %ld expansions].",
          robot_x_, robot_y_, gx, gy, prep_ms, plan_ms, planner.n_expansions());
        return;
      }

      RCLCPP_INFO(get_logger(),
        "[TIMING] Path with %zu waypoints: map+fields %.1f ms, A* %.1f ms "
        "(%ld expansions, %ld edge checks, %ld accepted)",
        path->size(), prep_ms, plan_ms, planner.n_expansions(),
        planner.n_edge_checks(), planner.n_edges_accepted());
      logHopTable(planner);

      publishTrajectory(*map, *path);
      publishStartPosition(*map);
      writeCsvs(*map, *path, planner.path_hops());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Error in goalCallback: %s", e.what());
    }
  }

  // GridMap -> Map2D5. Iterate OUR indices and query grid_map by position:
  // the two cell-center lattices coincide, so the lookup is exact, and
  // grid_map's inverted internal index ordering never enters the picture.
  std::unique_ptr<ballistic::Map2D5> convertMap(const grid_map_msgs::msg::GridMap & msg)
  {
    grid_map::GridMap gm;
    grid_map::GridMapRosConverter::fromMessage(msg, gm);

    const double res = gm.getResolution();
    const double size_x = gm.getLength().x();
    const double size_y = gm.getLength().y();
    const double origin_x = gm.getPosition().x() - size_x / 2.0;
    const double origin_y = gm.getPosition().y() - size_y / 2.0;

    auto map = std::make_unique<ballistic::Map2D5>(size_x, size_y, res, origin_x, origin_y);
    const auto & layer = gm.get("elevation");
    (void)layer;
    for (int r = 0; r < map->rows(); ++r) {
      for (int c = 0; c < map->cols(); ++c) {
        const auto [x, y] = map->grid_to_world(r, c);
        const float v = gm.atPosition("elevation", grid_map::Position(x, y));
        // NaN on the wire = OBSTACLE (infinitely tall column / unknown cell).
        map->at(r, c) = std::isnan(v) ? ballistic::Map2D5::OBSTACLE
                                      : static_cast<double>(v);
      }
    }
    return map;
  }

  void logHopTable(const ballistic::HoppingAStarPlanner & planner)
  {
    std::ostringstream oss;
    oss << "\nhop      X      Z   alpha    v_s   v_g   drop   E_inj\n";
    const auto & hops = planner.path_hops();
    for (size_t i = 0; i < hops.size(); ++i) {
      const auto & h = hops[i];
      char line[160];
      std::snprintf(line, sizeof(line),
        "%3zu  %5.2f %+6.2f  %5.1f  %5.2f %5.2f  %5.2f  %6.2f J\n",
        i + 1, h.X, h.Z, h.alpha_s * 180.0 / M_PI, h.v_s, h.v_g,
        h.apex_drop, h.e_inject);
      oss << line;
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void publishTrajectory(
    const ballistic::Map2D5 & map,
    const std::vector<std::pair<double, double>> & path)
  {
    if (path.size() < 2) {
      RCLCPP_WARN(get_logger(),
        "Path has %zu point(s); hopcopter ignores <2-point trajectories — not publishing.",
        path.size());
      return;
    }

    // DELETE first: puts hopcopter into "awaiting new path" so it does not
    // fall back to the previous goal. The pair is load-bearing.
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.header.stamp = now();
    del.id = kPathMarkerId;
    del.action = visualization_msgs::msg::Marker::DELETE;
    traj_pub_->publish(del);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = now();
    m.ns = "ballistic_path";
    m.id = kPathMarkerId;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.1;
    m.color.b = 1.0;  // blue, to distinguish from RRT*'s red
    m.color.a = 1.0;
    for (const auto & [x, y] : path) {
      const auto [r, c] = map.world_to_grid(x, y);
      geometry_msgs::msg::Point pt;
      pt.x = x;
      pt.y = y;
      // Control target: terrain elevation + hover offset. On flat ground this
      // is exactly today's hardcoded 0.8, preserving hopcopter's behavior.
      pt.z = map.at(r, c) + hover_offset_;
      m.points.push_back(pt);
    }
    traj_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "Published trajectory marker (id=%d, %zu points)",
      kPathMarkerId, m.points.size());
  }

  void publishStartPosition(const ballistic::Map2D5 & map)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.header.stamp = now();
    ps.pose.position.x = robot_x_;
    ps.pose.position.y = robot_y_;
    const auto [r, c] = map.world_to_grid(robot_x_, robot_y_);
    ps.pose.position.z = map.at(r, c) + hover_offset_;
    ps.pose.orientation.w = 1.0;
    start_pub_->publish(ps);
  }

  void writeCsvs(
    const ballistic::Map2D5 & map,
    const std::vector<std::pair<double, double>> & path,
    const std::vector<ballistic::Hop> & hops)
  {
    const auto t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream stamp;
    stamp << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

    {
      std::ofstream f(csv_dir_ + "/ballistic_path_" + stamp.str() + ".csv");
      if (f) {
        f << "x,y,z\n" << std::fixed << std::setprecision(6);
        for (const auto & [x, y] : path) {
          const auto [r, c] = map.world_to_grid(x, y);
          f << x << "," << y << "," << map.at(r, c) + hover_offset_ << "\n";
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open path CSV in %s", csv_dir_.c_str());
      }
    }
    {
      std::ofstream f(csv_dir_ + "/ballistic_hops_" + stamp.str() + ".csv");
      if (f) {
        f << "hop,X,Z,alpha_deg,v_s,v_g,v_g_in,e_inject,apex_drop,hop_radius\n"
          << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < hops.size(); ++i) {
          const auto & h = hops[i];
          f << i + 1 << "," << h.X << "," << h.Z << ","
            << h.alpha_s * 180.0 / M_PI << "," << h.v_s << "," << h.v_g << ","
            << h.v_g_in << "," << h.e_inject << "," << h.apex_drop << ","
            << h.hop_radius << "\n";
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Failed to open hops CSV in %s", csv_dir_.c_str());
      }
    }
  }

  ballistic::PlannerParams p_;
  double hover_offset_;
  std::string csv_dir_;

  grid_map_msgs::msg::GridMap::SharedPtr latest_map_msg_;
  double robot_x_ = 0.0, robot_y_ = 0.0;
  bool have_pose_ = false;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallisticPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
