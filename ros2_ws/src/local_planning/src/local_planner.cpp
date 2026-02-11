/**
 * @file local_planner.cpp
 * @brief Local planner with potential fields for hopping robot obstacle avoidance.
 * 
 * This node adjusts the next waypoint in the trajectory queue when obstacles are
 * detected near it. It runs once per jump cycle at the start of jumping state 3
 * (takeoff/climbing), computing a waypoint adjustment that will be ready when
 * the robot reaches the apex and begins falling.
 * 
 * Subscriptions:
 *   - /cf_0/lidar/points (sensor_msgs/PointCloud2): LiDAR point cloud
 *   - /jumping_state (std_msgs/Int8): Current jumping state (1=falling, 2=stance, 3=climbing)
 *   - /trajectory_queue_state (geometry_msgs/PoseArray): Current goal and next waypoint
 * 
 * Publications:
 *   - /local_planner/adjusted_waypoint (geometry_msgs/PointStamped): Adjusted waypoint
 *   - /local_planner/markers (visualization_msgs/MarkerArray): Debug visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <vector>
#include <memory>
#include <chrono>
#include <limits>

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("local_planner")
    {
        // Declare parameters with defaults
        this->declare_parameter("obstacle_detection_radius", 0.5);  // meters
        this->declare_parameter("max_waypoint_displacement", 0.3);  // meters
        this->declare_parameter("min_obstacle_distance", 0.1);      // meters - clamp for force calculation
        this->declare_parameter("force_gain", 0.5);                 // scaling factor for repulsive force
        this->declare_parameter("obstacle_z_min", 0.15);            // meters - ignore points below this (ground filter)
        this->declare_parameter("obstacle_z_max", 1.5);             // meters - ignore points above this
        
        // Get parameters
        obstacle_detection_radius_ = this->get_parameter("obstacle_detection_radius").as_double();
        max_waypoint_displacement_ = this->get_parameter("max_waypoint_displacement").as_double();
        min_obstacle_distance_ = this->get_parameter("min_obstacle_distance").as_double();
        force_gain_ = this->get_parameter("force_gain").as_double();
        obstacle_z_min_ = this->get_parameter("obstacle_z_min").as_double();
        obstacle_z_max_ = this->get_parameter("obstacle_z_max").as_double();
        
        RCLCPP_INFO(this->get_logger(), 
            "Local planner initialized with: detection_radius=%.2f, max_displacement=%.2f, z_filter=[%.2f, %.2f]",
            obstacle_detection_radius_, max_waypoint_displacement_, obstacle_z_min_, obstacle_z_max_);
        
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscribers
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cf_0/lidar/points", 
            rclcpp::SensorDataQoS(),
            std::bind(&LocalPlanner::lidarCallback, this, std::placeholders::_1));
        
        jumping_state_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/jumping_state",
            10,
            std::bind(&LocalPlanner::jumpingStateCallback, this, std::placeholders::_1));
        
        queue_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/trajectory_queue_state",
            10,
            std::bind(&LocalPlanner::queueStateCallback, this, std::placeholders::_1));
        
        // Publishers
        adjusted_waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/local_planner/adjusted_waypoint", 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/local_planner/markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "Local planner node started");
    }

private:
    // Parameters
    double obstacle_detection_radius_;
    double max_waypoint_displacement_;
    double min_obstacle_distance_;
    double force_gain_;
    double obstacle_z_min_;  // Ground filter: ignore points below this height
    double obstacle_z_max_;  // Ceiling filter: ignore points above this height
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // State tracking
    int8_t current_jumping_state_ = 0;
    int8_t prev_jumping_state_ = 0;
    bool processed_this_cycle_ = false;
    
    // Cumulative drift prevention: track which waypoint we've already adjusted
    // We store the original waypoint position before adjustment, so we can detect
    // when a NEW waypoint appears (different position = robot moved to next goal)
    geometry_msgs::msg::Point last_adjusted_waypoint_original_;
    bool has_adjusted_waypoint_ = false;
    const double waypoint_change_threshold_ = 0.05;  // meters - threshold to detect new waypoint
    
    // Timing measurement
    std::chrono::steady_clock::time_point queue_received_time_;
    
    // Cached data
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_world_;
    bool cloud_valid_ = false;
    
    // Queue state from hopcopter
    geometry_msgs::msg::Point current_goal_;
    geometry_msgs::msg::Point next_waypoint_;
    bool current_goal_valid_ = false;
    bool next_waypoint_valid_ = false;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr jumping_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr queue_state_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr adjusted_waypoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    /**
     * @brief Cache the latest LiDAR scan, transformed to world frame.
     */
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // Transform point cloud to world frame
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
                "world",
                msg->header.frame_id,
                tf2::TimePointZero);  // Use latest available transform
            
            sensor_msgs::msg::PointCloud2 cloud_world;
            tf2::doTransform(*msg, cloud_world, transform);
            
            // Convert to PCL
            latest_cloud_world_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::fromROSMsg(cloud_world, *latest_cloud_world_);
            cloud_valid_ = true;
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "LiDAR transform failed: %s", ex.what());
            cloud_valid_ = false;
        }
    }
    
    /**
     * @brief Handle jumping state updates. Trigger planning on state 3 entry.
     */
    void jumpingStateCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        current_jumping_state_ = msg->data;
        
        // Detect transition TO state 3 (takeoff)
        if (prev_jumping_state_ != 3 && current_jumping_state_ == 3) {
            processed_this_cycle_ = false;  // Reset for new jump cycle
            RCLCPP_DEBUG(this->get_logger(), "Detected transition to jumping state 3");
        }
        
        // When we leave state 3, reset for next cycle
        if (prev_jumping_state_ == 3 && current_jumping_state_ != 3) {
            processed_this_cycle_ = false;
        }
        
        prev_jumping_state_ = current_jumping_state_;
    }
    
    /**
     * @brief Handle queue state from hopcopter. This triggers the actual planning.
     */
    void queueStateCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Queue state has fewer than 2 poses");
            return;
        }
        
        // Extract current goal (pose 0)
        current_goal_valid_ = (msg->poses[0].orientation.w > 0.5);
        if (current_goal_valid_) {
            current_goal_ = msg->poses[0].position;
        }
        
        // Extract next waypoint (pose 1)
        next_waypoint_valid_ = (msg->poses[1].orientation.w > 0.5);
        if (next_waypoint_valid_) {
            next_waypoint_ = msg->poses[1].position;
        }
        
        RCLCPP_DEBUG(this->get_logger(), 
            "Queue state received: current_goal_valid=%d, next_waypoint_valid=%d",
            current_goal_valid_, next_waypoint_valid_);
        
        // Only process once per jump cycle, during state 3
        if (current_jumping_state_ == 3 && !processed_this_cycle_) {
            queue_received_time_ = std::chrono::steady_clock::now();
            RCLCPP_INFO(this->get_logger(), "[TIMING] Queue state received, starting planning...");
            runPotentialFieldPlanning();
            processed_this_cycle_ = true;
        }
    }
    
    /**
     * @brief Main planning function: compute potential field forces and adjust waypoint.
     */
    void runPotentialFieldPlanning()
    {
        RCLCPP_INFO(this->get_logger(), "Running potential field planning...");
        
        // Check prerequisites
        if (!next_waypoint_valid_) {
            RCLCPP_INFO(this->get_logger(), "No next waypoint to adjust");
            publishVisualization({}, next_waypoint_, next_waypoint_, 0.0, 0.0);
            return;
        }
        
        // Cumulative drift prevention: check if this is the SAME waypoint we already adjusted
        if (has_adjusted_waypoint_) {
            double dx = next_waypoint_.x - last_adjusted_waypoint_original_.x;
            double dy = next_waypoint_.y - last_adjusted_waypoint_original_.y;
            double dist_to_last = std::sqrt(dx * dx + dy * dy);
            
            if (dist_to_last < waypoint_change_threshold_) {
                // Same waypoint as before - don't adjust again to prevent cumulative drift
                RCLCPP_INFO(this->get_logger(), 
                    "Skipping adjustment - waypoint (%.2f, %.2f) already adjusted this goal period",
                    next_waypoint_.x, next_waypoint_.y);
                return;
            } else {
                // New waypoint detected - robot must have moved to next goal
                RCLCPP_INFO(this->get_logger(), 
                    "New next waypoint detected (%.2f, %.2f) - allowing adjustment",
                    next_waypoint_.x, next_waypoint_.y);
                has_adjusted_waypoint_ = false;
            }
        }
        
        if (!cloud_valid_ || !latest_cloud_world_) {
            RCLCPP_WARN(this->get_logger(), "No valid LiDAR data available");
            publishVisualization({}, next_waypoint_, next_waypoint_, 0.0, 0.0);
            return;
        }
        
        // Find obstacles within detection radius of next waypoint
        // Use CLOSEST obstacle only - obstacle size should NOT affect repulsion strength!
        // "A bigger obstacle doesn't mean we should step farther away from it.
        //  As long as we're at a 'safe' distance, that's fine!"
        std::vector<pcl::PointXYZ> nearby_obstacles;
        double closest_dist = std::numeric_limits<double>::max();
        double closest_dx = 0.0;
        double closest_dy = 0.0;
        
        for (const auto& pt : latest_cloud_world_->points) {
            // Skip invalid points
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            
            // Z-filter: ignore ground returns and ceiling points
            // Only consider obstacles at heights where robot could collide
            if (pt.z < obstacle_z_min_ || pt.z > obstacle_z_max_) {
                continue;
            }
            
            // Calculate 2D distance to next waypoint (we only adjust x, y)
            double dx = pt.x - next_waypoint_.x;
            double dy = pt.y - next_waypoint_.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < obstacle_detection_radius_) {
                nearby_obstacles.push_back(pt);
                
                // Track the CLOSEST obstacle point only
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_dx = dx;
                    closest_dy = dy;
                }
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Found %zu obstacles near next waypoint (%.2f, %.2f)",
            nearby_obstacles.size(), next_waypoint_.x, next_waypoint_.y);
        
        // Compute adjusted waypoint
        geometry_msgs::msg::Point adjusted_waypoint = next_waypoint_;
        double force_x = 0.0;
        double force_y = 0.0;
        
        if (!nearby_obstacles.empty()) {
            // Calculate repulsion based on CLOSEST obstacle only
            // This ensures obstacle SIZE doesn't affect repulsion strength
            double clamped_dist = std::max(closest_dist, min_obstacle_distance_);
            
            // Linear falloff repulsive force: (radius - dist) / radius
            double force_magnitude = (obstacle_detection_radius_ - closest_dist) / obstacle_detection_radius_;
            force_magnitude *= force_gain_;
            
            // Force direction: from closest obstacle toward waypoint (pushes waypoint away)
            if (clamped_dist > 1e-6) {
                force_x = -closest_dx / clamped_dist * force_magnitude;
                force_y = -closest_dy / clamped_dist * force_magnitude;
            }
            
            // Calculate displacement
            double displacement_x = force_x;
            double displacement_y = force_y;
            
            // Clamp to maximum displacement
            double displacement_mag = std::sqrt(displacement_x * displacement_x + 
                                                displacement_y * displacement_y);
            
            if (displacement_mag > max_waypoint_displacement_) {
                double scale = max_waypoint_displacement_ / displacement_mag;
                displacement_x *= scale;
                displacement_y *= scale;
                displacement_mag = max_waypoint_displacement_;
            }
            
            adjusted_waypoint.x = next_waypoint_.x + displacement_x;
            adjusted_waypoint.y = next_waypoint_.y + displacement_y;
            
            RCLCPP_INFO(this->get_logger(),
                "Adjusting waypoint: (%.2f, %.2f) -> (%.2f, %.2f), displacement=%.3f, closest_dist=%.3f",
                next_waypoint_.x, next_waypoint_.y,
                adjusted_waypoint.x, adjusted_waypoint.y,
                displacement_mag, closest_dist);
            
            // Publish adjusted waypoint
            // Use header.stamp.sec to encode waypoint index (0 = first item in waypoint_list)
            geometry_msgs::msg::PointStamped adjusted_msg;
            adjusted_msg.header.stamp.sec = 0;  // Index 0 in hopcopter's waypoint_list
            adjusted_msg.header.stamp.nanosec = 0;
            adjusted_msg.header.frame_id = "world";
            adjusted_msg.point = adjusted_waypoint;
            adjusted_waypoint_pub_->publish(adjusted_msg);
            
            // DEBUG: timing measurement
            auto publish_time = std::chrono::steady_clock::now();
            auto planning_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                publish_time - queue_received_time_).count();
            RCLCPP_INFO(this->get_logger(), 
                "[TIMING] Adjusted waypoint published - planning took %.2f ms",
                planning_duration / 1000.0);
            
            // Record that we've adjusted this waypoint (store ORIGINAL position for comparison)
            last_adjusted_waypoint_original_ = next_waypoint_;
            has_adjusted_waypoint_ = true;
            
            publishVisualization(nearby_obstacles, next_waypoint_, adjusted_waypoint,
                                force_x, force_y);
        } else {
            RCLCPP_INFO(this->get_logger(), "No obstacles detected - waypoint unchanged");
            publishVisualization({}, next_waypoint_, next_waypoint_, 0.0, 0.0);
        }
    }
    
    /**
     * @brief Publish visualization markers for debugging.
     */
    void publishVisualization(const std::vector<pcl::PointXYZ>& obstacles,
                              const geometry_msgs::msg::Point& original_wp,
                              const geometry_msgs::msg::Point& adjusted_wp,
                              double force_x, double force_y)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        auto now = this->get_clock()->now();
        
        // Clear previous markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "world";
        delete_marker.header.stamp = now;
        delete_marker.ns = "local_planner";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        
        // Marker 1: Original waypoint (blue sphere)
        visualization_msgs::msg::Marker original_marker;
        original_marker.header.frame_id = "world";
        original_marker.header.stamp = now;
        original_marker.ns = "local_planner";
        original_marker.id = 1;
        original_marker.type = visualization_msgs::msg::Marker::SPHERE;
        original_marker.action = visualization_msgs::msg::Marker::ADD;
        original_marker.pose.position = original_wp;
        original_marker.pose.orientation.w = 1.0;
        original_marker.scale.x = 0.15;
        original_marker.scale.y = 0.15;
        original_marker.scale.z = 0.15;
        original_marker.color.r = 0.0;
        original_marker.color.g = 0.0;
        original_marker.color.b = 1.0;
        original_marker.color.a = 0.8;
        original_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(original_marker);
        
        // Marker 2: Adjusted waypoint (green sphere)
        visualization_msgs::msg::Marker adjusted_marker;
        adjusted_marker.header.frame_id = "world";
        adjusted_marker.header.stamp = now;
        adjusted_marker.ns = "local_planner";
        adjusted_marker.id = 2;
        adjusted_marker.type = visualization_msgs::msg::Marker::SPHERE;
        adjusted_marker.action = visualization_msgs::msg::Marker::ADD;
        adjusted_marker.pose.position = adjusted_wp;
        adjusted_marker.pose.orientation.w = 1.0;
        adjusted_marker.scale.x = 0.15;
        adjusted_marker.scale.y = 0.15;
        adjusted_marker.scale.z = 0.15;
        adjusted_marker.color.r = 0.0;
        adjusted_marker.color.g = 1.0;
        adjusted_marker.color.b = 0.0;
        adjusted_marker.color.a = 0.8;
        adjusted_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(adjusted_marker);
        
        // Marker 3: Detected obstacles (red spheres)
        if (!obstacles.empty()) {
            visualization_msgs::msg::Marker obstacles_marker;
            obstacles_marker.header.frame_id = "world";
            obstacles_marker.header.stamp = now;
            obstacles_marker.ns = "local_planner";
            obstacles_marker.id = 3;
            obstacles_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            obstacles_marker.action = visualization_msgs::msg::Marker::ADD;
            obstacles_marker.pose.orientation.w = 1.0;
            obstacles_marker.scale.x = 0.05;
            obstacles_marker.scale.y = 0.05;
            obstacles_marker.scale.z = 0.05;
            obstacles_marker.color.r = 1.0;
            obstacles_marker.color.g = 0.0;
            obstacles_marker.color.b = 0.0;
            obstacles_marker.color.a = 0.8;
            obstacles_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            
            for (const auto& obs : obstacles) {
                geometry_msgs::msg::Point p;
                p.x = obs.x;
                p.y = obs.y;
                p.z = obs.z;
                obstacles_marker.points.push_back(p);
            }
            marker_array.markers.push_back(obstacles_marker);
        }
        
        // Marker 4: Force vector (yellow arrow)
        double force_mag = std::sqrt(force_x * force_x + force_y * force_y);
        if (force_mag > 0.01) {
            visualization_msgs::msg::Marker force_marker;
            force_marker.header.frame_id = "world";
            force_marker.header.stamp = now;
            force_marker.ns = "local_planner";
            force_marker.id = 4;
            force_marker.type = visualization_msgs::msg::Marker::ARROW;
            force_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Arrow from original waypoint in force direction
            geometry_msgs::msg::Point start, end;
            start = original_wp;
            end.x = original_wp.x + force_x;
            end.y = original_wp.y + force_y;
            end.z = original_wp.z;
            force_marker.points.push_back(start);
            force_marker.points.push_back(end);
            
            force_marker.scale.x = 0.03;  // Shaft diameter
            force_marker.scale.y = 0.06;  // Head diameter
            force_marker.scale.z = 0.0;
            force_marker.color.r = 1.0;
            force_marker.color.g = 1.0;
            force_marker.color.b = 0.0;
            force_marker.color.a = 1.0;
            force_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(force_marker);
        }
        
        // Marker 5: Detection radius circle (cyan)
        visualization_msgs::msg::Marker radius_marker;
        radius_marker.header.frame_id = "world";
        radius_marker.header.stamp = now;
        radius_marker.ns = "local_planner";
        radius_marker.id = 5;
        radius_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        radius_marker.action = visualization_msgs::msg::Marker::ADD;
        radius_marker.pose.position = original_wp;
        radius_marker.pose.position.z = 0.01;  // Slightly above ground
        radius_marker.pose.orientation.w = 1.0;
        radius_marker.scale.x = obstacle_detection_radius_ * 2.0;
        radius_marker.scale.y = obstacle_detection_radius_ * 2.0;
        radius_marker.scale.z = 0.01;
        radius_marker.color.r = 0.0;
        radius_marker.color.g = 1.0;
        radius_marker.color.b = 1.0;
        radius_marker.color.a = 0.3;
        radius_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(radius_marker);
        
        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}