/**
 * @file local_planner.cpp
 * @brief Local planner with scoring-based landing point selection for hopping robot.
 * 
 * This node selects the best landing point from a discrete grid of candidates around
 * the next waypoint. Each candidate is scored based on:
 *   1. Surface slope (penalize steep terrain)
 *   2. Obstacle proximity (penalize points near obstacles)
 *   3. Hop distance (penalize points far from takeoff)
 *   4. Edge proximity (penalize points near terrain drop-offs)
 * 
 * Runs once per jump cycle at the start of jumping state 3 (takeoff/climbing).
 * 
 * Subscriptions:
 *   - /cf_0/lidar/points (sensor_msgs/PointCloud2): LiDAR point cloud
 *   - /jumping_state (std_msgs/Int8): Current jumping state (1=falling, 2=stance, 3=climbing)
 *   - /trajectory_queue_state (geometry_msgs/PoseArray): Current goal and next waypoint
 *   - /visited_waypoint (geometry_msgs/PointStamped): Every waypoint the robot targets (from hopcopter)
 * 
 * Publications:
 *   - /local_planner/adjusted_waypoint (geometry_msgs/PointStamped): Best landing point
 *   - /local_planner/markers (visualization_msgs/MarkerArray): Debug visualization
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen for PCA-based surface normal estimation
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <vector>
#include <memory>
#include <chrono>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iomanip>

// Candidate landing point with computed scores
struct CandidatePoint {
    double x, y, z;
    double score_slope;
    double score_obstacle;
    double score_distance;
    double score_edge;
    double total_score;
};

class LocalPlanner : public rclcpp::Node
{
public:
    LocalPlanner() : Node("local_planner")
    {
        // Declare parameters with defaults
        // Grid discretization
        this->declare_parameter("candidate_grid_radius", 0.3);      // meters - radius of search circle
        this->declare_parameter("candidate_grid_step", 0.1);        // meters - spacing between candidates
        
        // Analysis radii
        this->declare_parameter("obstacle_detection_radius", 0.5);  // meters - for obstacle scoring
        this->declare_parameter("slope_analysis_radius", 0.1);      // meters - for normal estimation
        this->declare_parameter("edge_detection_radius", 0.15);     // meters - for edge search
        this->declare_parameter("edge_height_threshold", 0.08);     // meters - Z drop to classify as edge
        
        // Scoring weights
        this->declare_parameter("weight_slope", 0.0);
        this->declare_parameter("weight_obstacle", 1.0);
        this->declare_parameter("weight_distance", 1.0);
        this->declare_parameter("weight_edge", 0.0);
        
        // Robot parameters
        this->declare_parameter("robot_leg_height", 0.25);          // meters - for ground filtering
        this->declare_parameter("max_slope_angle", 0.5);            // radians (~30 degrees)
        this->declare_parameter("downsample_voxel_size", 0.05);     // meters - voxel size for downsampling
        
        // Get parameters
        candidate_grid_radius_ = this->get_parameter("candidate_grid_radius").as_double();
        candidate_grid_step_ = this->get_parameter("candidate_grid_step").as_double();
        obstacle_detection_radius_ = this->get_parameter("obstacle_detection_radius").as_double();
        slope_analysis_radius_ = this->get_parameter("slope_analysis_radius").as_double();
        edge_detection_radius_ = this->get_parameter("edge_detection_radius").as_double();
        edge_height_threshold_ = this->get_parameter("edge_height_threshold").as_double();
        weight_slope_ = this->get_parameter("weight_slope").as_double();
        weight_obstacle_ = this->get_parameter("weight_obstacle").as_double();
        weight_distance_ = this->get_parameter("weight_distance").as_double();
        weight_edge_ = this->get_parameter("weight_edge").as_double();
        robot_leg_height_ = this->get_parameter("robot_leg_height").as_double();
        max_slope_angle_ = this->get_parameter("max_slope_angle").as_double();
        downsample_voxel_size_ = this->get_parameter("downsample_voxel_size").as_double();
        
        RCLCPP_INFO(this->get_logger(), 
            "Local planner initialized: grid_radius=%.2f, grid_step=%.2f, voxel=%.3f, weights=[%.1f,%.1f,%.1f,%.1f]",
            candidate_grid_radius_, candidate_grid_step_, downsample_voxel_size_,
            weight_slope_, weight_obstacle_, weight_distance_, weight_edge_);
        
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
        
        visited_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/visited_waypoint",
            10,
            std::bind(&LocalPlanner::visitedWaypointCallback, this, std::placeholders::_1));
        
        // Publishers
        adjusted_waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/local_planner/adjusted_waypoint", 10);
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/local_planner/markers", 10);
        
        // Initialize CSV logging file with timestamped filename
        initCSVFile();
        
        RCLCPP_INFO(this->get_logger(), "Local planner node started");
    }

private:
    // Parameters - Grid discretization
    double candidate_grid_radius_;
    double candidate_grid_step_;
    
    // Parameters - Analysis radii
    double obstacle_detection_radius_;
    double slope_analysis_radius_;
    double edge_detection_radius_;
    double edge_height_threshold_;
    
    // Parameters - Scoring weights
    double weight_slope_;
    double weight_obstacle_;
    double weight_distance_;
    double weight_edge_;
    
    // Parameters - Robot
    double robot_leg_height_;
    double max_slope_angle_;
    double downsample_voxel_size_;
    
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
    
    // CSV logging
    std::ofstream csv_file_;
    
    // CSV dedup: track last waypoint written to CSV to avoid duplicate rows
    geometry_msgs::msg::Point last_csv_logged_waypoint_;
    bool has_logged_current_waypoint_ = false;
    
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
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr visited_waypoint_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr adjusted_waypoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    /**
     * @brief Initialize timestamped CSV file for logging planning decisions.
     */
    void initCSVFile()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now;
        localtime_r(&time_t_now, &tm_now);
        
        std::ostringstream filename;
        filename << "/home/zweminhtetaung/CrazySim/data/local_planner_output_"
                 << std::put_time(&tm_now, "%Y-%m-%d_%H-%M-%S") << ".csv";
        
        csv_file_.open(filename.str(), std::ios::out | std::ios::app);
        if (csv_file_.is_open()) {
            csv_file_ << "timestamp,original_x,original_y,original_z,"
                      << "selected_x,selected_y,selected_z,"
                      << "total_score,score_slope,score_obstacle,score_distance,score_edge"
                      << std::endl;
            RCLCPP_INFO(this->get_logger(), "CSV logging to: %s", filename.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", filename.str().c_str());
        }
    }
    
    /**
     * @brief Check if a waypoint is different from the last one logged to CSV.
     */
    bool isNewWaypoint(const geometry_msgs::msg::Point& wp)
    {
        if (!has_logged_current_waypoint_) return true;
        double dx = wp.x - last_csv_logged_waypoint_.x;
        double dy = wp.y - last_csv_logged_waypoint_.y;
        return std::sqrt(dx * dx + dy * dy) >= waypoint_change_threshold_;
    }
    
    /**
     * @brief Mark a waypoint as logged to CSV (for dedup).
     */
    void markWaypointLogged(const geometry_msgs::msg::Point& wp)
    {
        last_csv_logged_waypoint_ = wp;
        has_logged_current_waypoint_ = true;
    }
    
    /**
     * @brief Log a planning decision (original waypoint + selected best point) to CSV.
     */
    void logToCSV(const geometry_msgs::msg::Point& original, const CandidatePoint& best)
    {
        if (!csv_file_.is_open()) return;
        
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        std::tm tm_now;
        localtime_r(&time_t_now, &tm_now);
        
        csv_file_ << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S") << "." 
                  << std::setfill('0') << std::setw(3) << ms.count() << ","
                  << std::fixed << std::setprecision(4)
                  << original.x << "," << original.y << "," << original.z << ","
                  << best.x << "," << best.y << "," << best.z << ","
                  << std::setprecision(4)
                  << best.total_score << "," << best.score_slope << ","
                  << best.score_obstacle << "," << best.score_distance << ","
                  << best.score_edge << std::endl;
        csv_file_.flush();
        markWaypointLogged(original);
    }
    
    /**
     * @brief Log an RRT* waypoint where the local planner did NOT adjust it.
     * Selected coordinates and scores are recorded as "-".
     */
    void logSkippedWaypoint(const geometry_msgs::msg::Point& original)
    {
        if (!csv_file_.is_open()) return;
        if (!isNewWaypoint(original)) return;  // Already logged this waypoint
        
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        std::tm tm_now;
        localtime_r(&time_t_now, &tm_now);
        
        csv_file_ << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S") << "." 
                  << std::setfill('0') << std::setw(3) << ms.count() << ","
                  << std::fixed << std::setprecision(4)
                  << original.x << "," << original.y << "," << original.z << ","
                  << "-,-,-,-,-,-,-" << std::endl;
        csv_file_.flush();
        markWaypointLogged(original);
    }
    
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
            runScoringBasedPlanning();
            processed_this_cycle_ = true;
        }
    }
    
    /**
     * @brief Handle visited waypoint from hopcopter.
     * Every waypoint the robot targets is published here. If the local planner
     * already logged it with scores (via logToCSV), the dedup in logSkippedWaypoint
     * will skip it. Otherwise, it logs the RRT* point with dashes for scores.
     */
    void visitedWaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), 
            "Visited waypoint received: (%.3f, %.3f, %.3f)",
            msg->point.x, msg->point.y, msg->point.z);
        
        // Log this waypoint if the local planner didn't already log it with scores
        logSkippedWaypoint(msg->point);
    }
    
    /**
     * @brief Downsample points using voxel grid approach.
     * Keeps one point per voxel to reduce density while preserving structure.
     */
    std::vector<pcl::PointXYZ> downsamplePoints(const std::vector<pcl::PointXYZ>& points)
    {
        if (downsample_voxel_size_ <= 0.0) {
            return points;  // No downsampling if voxel size is 0 or negative
        }
        
        // Hash function for voxel coordinates
        struct VoxelHash {
            size_t operator()(const std::tuple<int, int, int>& v) const {
                return std::hash<int>()(std::get<0>(v)) ^ 
                       (std::hash<int>()(std::get<1>(v)) << 1) ^
                       (std::hash<int>()(std::get<2>(v)) << 2);
            }
        };
        
        std::unordered_map<std::tuple<int, int, int>, pcl::PointXYZ, VoxelHash> voxel_map;
        const double inv_voxel = 1.0 / downsample_voxel_size_;
        
        for (const auto& pt : points) {
            int vx = static_cast<int>(std::floor(pt.x * inv_voxel));
            int vy = static_cast<int>(std::floor(pt.y * inv_voxel));
            int vz = static_cast<int>(std::floor(pt.z * inv_voxel));
            
            auto key = std::make_tuple(vx, vy, vz);
            if (voxel_map.find(key) == voxel_map.end()) {
                voxel_map[key] = pt;  // Keep first point in each voxel
            }
        }
        
        std::vector<pcl::PointXYZ> downsampled;
        downsampled.reserve(voxel_map.size());
        for (const auto& kv : voxel_map) {
            downsampled.push_back(kv.second);
        }
        
        return downsampled;
    }
    
    /**
     * @brief Pre-filter point cloud to only include points within a region of interest.
     * This dramatically reduces computation for scoring functions.
     */
    std::vector<pcl::PointXYZ> filterPointsInRegion(double center_x, double center_y, double center_z, double radius)
    {
        (void)center_z;  // Not used for 2D filtering
        std::vector<pcl::PointXYZ> filtered;
        const double radius_sq = radius * radius;
        
        for (const auto& pt : latest_cloud_world_->points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            
            double dx = pt.x - center_x;
            double dy = pt.y - center_y;
            double dist_sq = dx * dx + dy * dy;
            
            if (dist_sq <= radius_sq) {
                filtered.push_back(pt);
            }
        }
        
        return filtered;
    }
    
    /**
     * @brief Generate candidate landing points in a Cartesian grid around the waypoint.
     */
    std::vector<CandidatePoint> generateCandidatePoints(double center_x, double center_y, double center_z)
    {
        std::vector<CandidatePoint> candidates;
        const double radius_sq = candidate_grid_radius_ * candidate_grid_radius_;
        
        for (double dx = -candidate_grid_radius_; dx <= candidate_grid_radius_; dx += candidate_grid_step_) {
            for (double dy = -candidate_grid_radius_; dy <= candidate_grid_radius_; dy += candidate_grid_step_) {
                // Only include points within circular boundary
                if (dx * dx + dy * dy <= radius_sq) {
                    CandidatePoint cp;
                    cp.x = center_x + dx;
                    cp.y = center_y + dy;
                    cp.z = center_z;
                    cp.score_slope = 0.0;
                    cp.score_obstacle = 0.0;
                    cp.score_distance = 0.0;
                    cp.score_edge = 0.0;
                    cp.total_score = 0.0;
                    candidates.push_back(cp);
                }
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Generated %zu candidate points", candidates.size());
        return candidates;
    }
    
    /**
     * @brief Score based on surface slope at candidate point using PCA normal estimation.
     * @return Score in [0, 1] where 1.0 = flat surface, 0.0 = steep slope
     */
    double scoreSlopeAt(double x, double y, double z, const std::vector<pcl::PointXYZ>& region_points)
    {
        // Gather nearby points for normal estimation from pre-filtered region
        std::vector<Eigen::Vector3d> nearby_points;
        const double radius_sq = slope_analysis_radius_ * slope_analysis_radius_;
        
        for (const auto& pt : region_points) {
            double dx = pt.x - x;
            double dy = pt.y - y;
            double dz = pt.z - z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            
            if (dist_sq <= radius_sq) {
                nearby_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));
            }
        }
        
        // Need at least 3 points for plane fitting
        if (nearby_points.size() < 3) {
            return 0.5;  // Neutral score when insufficient data
        }
        
        // Compute centroid
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (const auto& p : nearby_points) {
            centroid += p;
        }
        centroid /= static_cast<double>(nearby_points.size());
        
        // Build covariance matrix
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& p : nearby_points) {
            Eigen::Vector3d d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<double>(nearby_points.size());
        
        // Eigenvalue decomposition - smallest eigenvector is surface normal
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        Eigen::Vector3d normal = solver.eigenvectors().col(0);  // Smallest eigenvalue
        
        // Ensure normal points upward
        if (normal.z() < 0) {
            normal = -normal;
        }
        
        // Slope angle from vertical (0 = flat, pi/2 = vertical wall)
        double slope_angle = std::acos(std::abs(normal.z()));
        
        // Score: 1.0 for flat, decreasing as slope increases
        double score = 1.0 - std::min(slope_angle / max_slope_angle_, 1.0);
        return std::max(0.0, std::min(1.0, score));
    }
    
    /**
     * @brief Score based on distance to nearest obstacle.
     * @return Score in [0, 1] where 1.0 = no obstacles nearby, 0.0 = obstacle at candidate
     */
    double scoreObstacleAt(double x, double y, double z, const std::vector<pcl::PointXYZ>& region_points)
    {
        double min_dist = std::numeric_limits<double>::max();
        const double ground_z_threshold = z - robot_leg_height_ + 0.05;
        
        for (const auto& pt : region_points) {
            // Filter ground points
            if (pt.z < ground_z_threshold) {
                continue;
            }
            
            double dx = pt.x - x;
            double dy = pt.y - y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < min_dist) {
                min_dist = dist;
            }
        }
        
        // If no obstacles within detection radius, full score
        if (min_dist >= obstacle_detection_radius_) {
            return 1.0;
        }
        
        // Score proportional to distance (farther = better)
        double score = min_dist / obstacle_detection_radius_;
        return std::max(0.0, std::min(1.0, score));
    }
    
    /**
     * @brief Score based on hop distance from takeoff point (current_goal_).
     * @return Score in [0, 1] where 1.0 = at takeoff point, 0.0 = at max grid radius
     */
    double scoreDistanceAt(double x, double y)
    {
        if (!current_goal_valid_) {
            return 0.5;  // Neutral score when no takeoff point
        }
        
        double dx = x - current_goal_.x;
        double dy = y - current_goal_.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        // Max expected distance: original waypoint dist + grid radius
        double orig_dx = next_waypoint_.x - current_goal_.x;
        double orig_dy = next_waypoint_.y - current_goal_.y;
        double orig_dist = std::sqrt(orig_dx * orig_dx + orig_dy * orig_dy);
        double max_dist = orig_dist + candidate_grid_radius_;
        
        if (max_dist < 1e-6) {
            return 1.0;
        }
        
        // Score: closer to takeoff = higher score
        double score = 1.0 - (dist / max_dist);
        return std::max(0.0, std::min(1.0, score));
    }
    
    /**
     * @brief Pre-compute edge points by scanning region once for height discontinuities.
     * This is O(n²) but only done once, not per-candidate.
     * @return Vector of 2D positions where edges were detected
     */
    std::vector<std::pair<double, double>> findEdgePoints(const std::vector<pcl::PointXYZ>& region_points)
    {
        std::vector<std::pair<double, double>> edge_points;
        
        // For each point, check if any neighbor drops significantly
        for (const auto& pt : region_points) {
            bool is_edge = false;
            
            for (const auto& neighbor : region_points) {
                if (is_edge) break;  // Already found this is an edge point
                
                double dx = neighbor.x - pt.x;
                double dy = neighbor.y - pt.y;
                double neighbor_dist_sq = dx * dx + dy * dy;
                
                // Look at nearby neighbors (within 0.1m horizontal)
                if (neighbor_dist_sq < 0.01 && neighbor_dist_sq > 0.001) {
                    double z_drop = pt.z - neighbor.z;
                    
                    // Significant drop detected = edge
                    if (z_drop > edge_height_threshold_) {
                        edge_points.emplace_back(pt.x, pt.y);
                        is_edge = true;
                    }
                }
            }
        }
        
        return edge_points;
    }
    
    /**
     * @brief Score based on proximity to pre-computed edge points.
     * @return Score in [0, 1] where 1.0 = no edges nearby, 0.0 = at edge
     */
    double scoreEdgeAt(double x, double y, const std::vector<std::pair<double, double>>& edge_points)
    {
        if (edge_points.empty()) {
            return 1.0;  // No edges detected, full score
        }
        
        double min_edge_dist = std::numeric_limits<double>::max();
        
        // Find distance to nearest edge point
        for (const auto& edge : edge_points) {
            double dx = x - edge.first;
            double dy = y - edge.second;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < min_edge_dist) {
                min_edge_dist = dist;
            }
        }
        
        // If no edges within detection radius, full score
        if (min_edge_dist >= edge_detection_radius_) {
            return 1.0;
        }
        
        // Score proportional to distance from edge
        double score = min_edge_dist / edge_detection_radius_;
        return std::max(0.0, std::min(1.0, score));
    }
    
    /**
     * @brief Compute total weighted score for a candidate point.
     */
    double computeTotalScore(double slope, double obstacle, double distance, double edge)
    {
        double total_weight = weight_slope_ + weight_obstacle_ + weight_distance_ + weight_edge_;
        if (total_weight < 1e-6) {
            return 0.0;
        }
        
        double weighted_sum = weight_slope_ * slope + 
                              weight_obstacle_ * obstacle + 
                              weight_distance_ * distance + 
                              weight_edge_ * edge;
        
        return weighted_sum / total_weight;
    }
    
    /**
     * @brief Main planning function: generate candidates, score them, select best.
     * @return true if planning ran successfully and logged to CSV, false otherwise.
     */
    bool runScoringBasedPlanning()
    {
        // Check prerequisites - silently return if no waypoint
        if (!next_waypoint_valid_) {
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Running scoring-based landing point selection...");
        
        // Cumulative drift prevention: check if this is the SAME waypoint we already adjusted
        if (has_adjusted_waypoint_) {
            double dx = next_waypoint_.x - last_adjusted_waypoint_original_.x;
            double dy = next_waypoint_.y - last_adjusted_waypoint_original_.y;
            double dist_to_last = std::sqrt(dx * dx + dy * dy);
            
            if (dist_to_last < waypoint_change_threshold_) {
                RCLCPP_INFO(this->get_logger(), 
                    "Skipping - waypoint (%.2f, %.2f) already adjusted this goal period",
                    next_waypoint_.x, next_waypoint_.y);
                return false;
            } else {
                RCLCPP_INFO(this->get_logger(), 
                    "New next waypoint detected (%.2f, %.2f) - allowing adjustment",
                    next_waypoint_.x, next_waypoint_.y);
                has_adjusted_waypoint_ = false;
            }
        }
        
        if (!cloud_valid_ || !latest_cloud_world_) {
            RCLCPP_WARN(this->get_logger(), "No valid LiDAR data available");
            logSkippedWaypoint(next_waypoint_);
            publishVisualization({}, next_waypoint_, next_waypoint_);
            return false;
        }
        
        // Pre-filter point cloud to region of interest
        // Radius includes grid + obstacle detection + edge detection margins
        double filter_radius = candidate_grid_radius_ + 
                               std::max(obstacle_detection_radius_, edge_detection_radius_);
        std::vector<pcl::PointXYZ> region_points = filterPointsInRegion(
            next_waypoint_.x, next_waypoint_.y, next_waypoint_.z, filter_radius);
        
        // Downsample to reduce computational load for edge detection
        size_t pre_downsample = region_points.size();
        region_points = downsamplePoints(region_points);
        
        RCLCPP_INFO(this->get_logger(), 
            "Filtered cloud: %zu -> %zu -> %zu points (voxel=%.3fm)",
            latest_cloud_world_->points.size(), pre_downsample, region_points.size(),
            downsample_voxel_size_);
        
        // Pre-compute edge points ONCE for the entire region (O(n²) done once)
        std::vector<std::pair<double, double>> edge_points = findEdgePoints(region_points);
        RCLCPP_DEBUG(this->get_logger(), "Found %zu edge points", edge_points.size());
        
        // Generate candidate grid
        std::vector<CandidatePoint> candidates = generateCandidatePoints(
            next_waypoint_.x, next_waypoint_.y, next_waypoint_.z);
        
        if (candidates.empty()) {
            RCLCPP_WARN(this->get_logger(), "No candidate points generated");
            logSkippedWaypoint(next_waypoint_);
            publishVisualization({}, next_waypoint_, next_waypoint_);
            return false;
        }
        
        // Score each candidate using pre-filtered region points and pre-computed edges
        for (auto& cp : candidates) {
            cp.score_slope = scoreSlopeAt(cp.x, cp.y, cp.z, region_points);
            cp.score_obstacle = scoreObstacleAt(cp.x, cp.y, cp.z, region_points);
            cp.score_distance = scoreDistanceAt(cp.x, cp.y);
            cp.score_edge = scoreEdgeAt(cp.x, cp.y, edge_points);
            cp.total_score = computeTotalScore(
                cp.score_slope, cp.score_obstacle, cp.score_distance, cp.score_edge);
        }
        
        // Find best candidate (highest score)
        auto best_it = std::max_element(candidates.begin(), candidates.end(),
            [](const CandidatePoint& a, const CandidatePoint& b) {
                return a.total_score < b.total_score;
            });
        
        const CandidatePoint& best = *best_it;
        
        RCLCPP_INFO(this->get_logger(),
            "Best landing point: (%.2f, %.2f) score=%.3f [slope=%.2f, obs=%.2f, dist=%.2f, edge=%.2f]",
            best.x, best.y, best.total_score,
            best.score_slope, best.score_obstacle, best.score_distance, best.score_edge);
        
        // Create adjusted waypoint
        geometry_msgs::msg::Point adjusted_waypoint;
        adjusted_waypoint.x = best.x;
        adjusted_waypoint.y = best.y;
        adjusted_waypoint.z = next_waypoint_.z;  // Keep original z
        
        // Publish adjusted waypoint
        geometry_msgs::msg::PointStamped adjusted_msg;
        adjusted_msg.header.stamp.sec = 0;  // Index 0 in hopcopter's waypoint_list
        adjusted_msg.header.stamp.nanosec = 0;
        adjusted_msg.header.frame_id = "world";
        adjusted_msg.point = adjusted_waypoint;
        adjusted_waypoint_pub_->publish(adjusted_msg);
        
        // Timing measurement
        auto publish_time = std::chrono::steady_clock::now();
        auto planning_duration = std::chrono::duration_cast<std::chrono::microseconds>(
            publish_time - queue_received_time_).count();
        RCLCPP_INFO(this->get_logger(), 
            "[TIMING] Best landing point published - planning took %.2f ms",
            planning_duration / 1000.0);
        
        // Log to CSV: original RRT* waypoint and selected best point
        logToCSV(next_waypoint_, best);
        
        // Record adjustment
        last_adjusted_waypoint_original_ = next_waypoint_;
        has_adjusted_waypoint_ = true;
        
        publishVisualization(candidates, next_waypoint_, adjusted_waypoint);
        return true;
    }
    
    /**
     * @brief Publish visualization markers for debugging.
     */
    void publishVisualization(const std::vector<CandidatePoint>& candidates,
                              const geometry_msgs::msg::Point& original_wp,
                              const geometry_msgs::msg::Point& selected_wp)
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
        original_marker.scale.x = 0.12;
        original_marker.scale.y = 0.12;
        original_marker.scale.z = 0.12;
        original_marker.color.r = 0.0;
        original_marker.color.g = 0.0;
        original_marker.color.b = 1.0;
        original_marker.color.a = 0.8;
        original_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(original_marker);
        
        // Marker 2: Selected landing point (larger green sphere)
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.header.frame_id = "world";
        selected_marker.header.stamp = now;
        selected_marker.ns = "local_planner";
        selected_marker.id = 2;
        selected_marker.type = visualization_msgs::msg::Marker::SPHERE;
        selected_marker.action = visualization_msgs::msg::Marker::ADD;
        selected_marker.pose.position = selected_wp;
        selected_marker.pose.orientation.w = 1.0;
        selected_marker.scale.x = 0.15;
        selected_marker.scale.y = 0.15;
        selected_marker.scale.z = 0.15;
        selected_marker.color.r = 0.0;
        selected_marker.color.g = 1.0;
        selected_marker.color.b = 0.0;
        selected_marker.color.a = 1.0;
        selected_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(selected_marker);
        
        // Marker 3: Candidate points (color gradient based on score)
        if (!candidates.empty()) {
            visualization_msgs::msg::Marker candidates_marker;
            candidates_marker.header.frame_id = "world";
            candidates_marker.header.stamp = now;
            candidates_marker.ns = "local_planner";
            candidates_marker.id = 3;
            candidates_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            candidates_marker.action = visualization_msgs::msg::Marker::ADD;
            candidates_marker.pose.orientation.w = 1.0;
            candidates_marker.scale.x = 0.06;
            candidates_marker.scale.y = 0.06;
            candidates_marker.scale.z = 0.06;
            candidates_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            
            for (const auto& cp : candidates) {
                geometry_msgs::msg::Point p;
                p.x = cp.x;
                p.y = cp.y;
                p.z = cp.z;
                candidates_marker.points.push_back(p);
                
                // Color: red (low score) to green (high score)
                std_msgs::msg::ColorRGBA color;
                color.r = static_cast<float>(1.0 - cp.total_score);
                color.g = static_cast<float>(cp.total_score);
                color.b = 0.0f;
                color.a = 0.7f;
                candidates_marker.colors.push_back(color);
            }
            marker_array.markers.push_back(candidates_marker);
        }
        
        // Marker 4: Search radius circle (cyan cylinder)
        visualization_msgs::msg::Marker radius_marker;
        radius_marker.header.frame_id = "world";
        radius_marker.header.stamp = now;
        radius_marker.ns = "local_planner";
        radius_marker.id = 4;
        radius_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        radius_marker.action = visualization_msgs::msg::Marker::ADD;
        radius_marker.pose.position = original_wp;
        radius_marker.pose.position.z = 0.01;
        radius_marker.pose.orientation.w = 1.0;
        radius_marker.scale.x = candidate_grid_radius_ * 2.0;
        radius_marker.scale.y = candidate_grid_radius_ * 2.0;
        radius_marker.scale.z = 0.01;
        radius_marker.color.r = 0.0;
        radius_marker.color.g = 1.0;
        radius_marker.color.b = 1.0;
        radius_marker.color.a = 0.3;
        radius_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
        marker_array.markers.push_back(radius_marker);
        
        // Marker 5: Takeoff point (yellow sphere) - current_goal_
        if (current_goal_valid_) {
            visualization_msgs::msg::Marker takeoff_marker;
            takeoff_marker.header.frame_id = "world";
            takeoff_marker.header.stamp = now;
            takeoff_marker.ns = "local_planner";
            takeoff_marker.id = 5;
            takeoff_marker.type = visualization_msgs::msg::Marker::SPHERE;
            takeoff_marker.action = visualization_msgs::msg::Marker::ADD;
            takeoff_marker.pose.position = current_goal_;
            takeoff_marker.pose.orientation.w = 1.0;
            takeoff_marker.scale.x = 0.1;
            takeoff_marker.scale.y = 0.1;
            takeoff_marker.scale.z = 0.1;
            takeoff_marker.color.r = 1.0;
            takeoff_marker.color.g = 1.0;
            takeoff_marker.color.b = 0.0;
            takeoff_marker.color.a = 0.8;
            takeoff_marker.lifetime = rclcpp::Duration::from_seconds(2.0);
            marker_array.markers.push_back(takeoff_marker);
        }
        
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