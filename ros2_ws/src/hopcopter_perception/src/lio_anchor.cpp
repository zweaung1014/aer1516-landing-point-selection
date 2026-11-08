// lio_anchor: start FAST-LIO on a held robot, anchor its frame to the world,
// release the robot, and score FAST-LIO against Gazebo (stage 2 of
// docs/lidar_mapping_plan.md).
//
// FAST-LIO estimates gravity from its first IMU samples, so it cannot start on
// a falling robot. The robot spawns held by a DetachableJoint
// (sitl_singleagent.sh -H). Once FAST-LIO has published odometry for
// settle_time and min_scans scans, this node
//  1. publishes the static TF world -> camera_init, taken from one pose during
//     the hold:  T(world <- imu, Gazebo) * T(camera_init <- imu, FAST-LIO)^-1;
//     after that, mapping uses only FAST-LIO (a mocap lab would do the same with
//     the mocap pose at the start);
//  2. publishes the release (std_msgs/Empty on release_topic, bridged to the
//     joint's detach topic), and the robot drops as it did at Play.
// mode:=gazebo releases after settle_time without waiting for FAST-LIO (the
// mapper on Gazebo's pose, stage 3a). If FAST-LIO has not started by
// release_timeout, the robot is released anyway, with an error.
//
// While the robot flies it logs FAST-LIO's pose against Gazebo's for every
// odometry message, to data/data_lio/lio_<stamp>.csv (scripts/lio_report.py).
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{
std::string wall_stamp()
{
  const auto t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream s;
  s << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return s.str();
}

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
    p.orientation.z).normalized().toRotationMatrix();
  return T;
}

// roll, pitch, yaw [deg] of R (world frame, ZYX)
Eigen::Vector3d rpy_deg(const Eigen::Matrix3d & R)
{
  const Eigen::Vector3d ypr = R.eulerAngles(2, 1, 0);
  return Eigen::Vector3d(ypr[2], ypr[1], ypr[0]) * 180.0 / M_PI;
}
}  // namespace

class LioAnchor : public rclcpp::Node
{
public:
  LioAnchor()
  : Node("lio_anchor")
  {
    mode_ = declare_parameter("mode", std::string("lio"));  // lio | gazebo
    world_ = declare_parameter("world_frame", std::string("world"));
    lio_frame_ = declare_parameter("lio_frame", std::string("camera_init"));
    truth_frame_ = declare_parameter("truth_imu_frame", std::string("cf_0/lidar"));
    const auto odom_topic = declare_parameter("odom_topic", std::string("/Odometry"));
    const auto release_topic = declare_parameter("release_topic", std::string("/cf_0/release"));
    settle_time_ = declare_parameter("settle_time", 1.0);      // [s, sim] after the first odometry
    min_scans_ = declare_parameter("min_scans", 5);
    release_timeout_ = declare_parameter("release_timeout", 30.0);  // [s, sim] after the first truth pose
    data_dir_ = declare_parameter(
      "data_dir", std::string("/home/zweminhtetaung/CrazySim/data/data_lio"));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    release_pub_ = create_publisher<std_msgs::msg::Empty>(release_topic, rclcpp::QoS(5));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        if (n_odom_++ == 0) {
          t_first_odom_ = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
          RCLCPP_INFO(get_logger(), "First FAST-LIO odometry at %.3f s", t_first_odom_.seconds());
        }
        pending_.push_back(*msg);
        while (pending_.size() > 200) {
          pending_.pop_front();
        }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {step();});
    RCLCPP_INFO(get_logger(), "Holding until %s; release on %s",
      mode_ == "gazebo" ? "the sim has run settle_time" : "FAST-LIO has settled",
      release_topic.c_str());
  }

private:
  bool truth_at(const rclcpp::Time & t, Eigen::Isometry3d & T)
  {
    try {
      T = tf2::transformToEigen(tf_buffer_->lookupTransform(world_, truth_frame_, t));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void release(const char * why)
  {
    for (int i = 0; i < 3; ++i) {
      release_pub_->publish(std_msgs::msg::Empty());
    }
    released_ = true;
    release_wall_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "Released the robot (%s) at %.3f s", why, now().seconds());
  }

  void step()
  {
    const rclcpp::Time t_now = now();
    if (t_now.nanoseconds() == 0) {
      return;  // no /clock yet
    }
    if (t_start_.nanoseconds() == 0) {
      Eigen::Isometry3d T;
      if (!truth_at(rclcpp::Time(0, 0, RCL_ROS_TIME), T)) {
        return;  // no Gazebo pose yet (Play not pressed)
      }
      t_start_ = t_now;
    }
    // Re-send the release for a second: the bridge may not have matched yet.
    if (released_ && std::chrono::steady_clock::now() - release_wall_ < std::chrono::seconds(1)) {
      release_pub_->publish(std_msgs::msg::Empty());
    }
    if (mode_ == "gazebo") {
      if (!released_ && (t_now - t_start_).seconds() >= settle_time_) {
        release("gazebo mode");
      }
      return;
    }
    if (!anchored_) {
      if (n_odom_ > 0 && !pending_.empty()) {
        const auto & last = pending_.back();
        const rclcpp::Time t_last(last.header.stamp, RCL_ROS_TIME);
        Eigen::Isometry3d T_w_imu;
        if ((t_last - t_first_odom_).seconds() >= settle_time_ && n_odom_ >= min_scans_ &&
          truth_at(t_last, T_w_imu))
        {
          anchor(T_w_imu, to_eigen(last.pose.pose), t_last);
          release("FAST-LIO is up");
        }
      }
      if (!anchored_ && !released_ && (t_now - t_start_).seconds() > release_timeout_) {
        RCLCPP_ERROR(get_logger(), "FAST-LIO has not started after %.0f s: releasing anyway; "
          "there will be no LiDAR map until it does", release_timeout_);
        release("timeout");
      }
      // Late start (released on the timeout): anchor on the fly, from Gazebo.
      if (!anchored_ && released_ && !pending_.empty()) {
        const auto & last = pending_.back();
        const rclcpp::Time t_last(last.header.stamp, RCL_ROS_TIME);
        Eigen::Isometry3d T_w_imu;
        if (n_odom_ >= min_scans_ && truth_at(t_last, T_w_imu)) {
          anchor(T_w_imu, to_eigen(last.pose.pose), t_last);
        }
      }
      if (!anchored_) {
        return;
      }
    }
    // Score every odometry message whose Gazebo pose is in the buffer.
    while (!pending_.empty()) {
      const auto & o = pending_.front();
      const rclcpp::Time t(o.header.stamp, RCL_ROS_TIME);
      Eigen::Isometry3d T_truth;
      if (!truth_at(t, T_truth)) {
        if ((t_now - t).seconds() > 0.5) {
          pending_.pop_front();
          continue;
        }
        break;
      }
      log(t, T_w_ci_ * to_eigen(o.pose.pose), T_truth);
      pending_.pop_front();
    }
  }

  void anchor(const Eigen::Isometry3d & T_w_imu, const Eigen::Isometry3d & T_ci_imu, const rclcpp::Time & t)
  {
    T_w_ci_ = T_w_imu * T_ci_imu.inverse();
    geometry_msgs::msg::TransformStamped s = tf2::eigenToTransform(T_w_ci_);
    s.header.stamp = t;
    s.header.frame_id = world_;
    s.child_frame_id = lio_frame_;
    static_tf_->sendTransform(s);
    anchored_ = true;
    const Eigen::Vector3d p = T_w_ci_.translation(), a = rpy_deg(T_w_ci_.linear());
    RCLCPP_INFO(get_logger(),
      "Anchored %s -> %s at %.3f s: (%.3f, %.3f, %.3f) m, rpy (%.2f, %.2f, %.2f) deg; scoring "
      "FAST-LIO against Gazebo", world_.c_str(), lio_frame_.c_str(), t.seconds(), p.x(), p.y(),
      p.z(), a.x(), a.y(), a.z());
    // Everything before the anchor is the hold: drop it from the log.
    pending_.clear();
  }

  void log(const rclcpp::Time & t, const Eigen::Isometry3d & lio, const Eigen::Isometry3d & truth)
  {
    if (!csv_.is_open()) {
      std::error_code ec;
      std::filesystem::create_directories(data_dir_, ec);
      const std::string file = data_dir_ + "/lio_" + wall_stamp() + ".csv";
      csv_.open(file);
      csv_ << "stamp,released,lio_x,lio_y,lio_z,truth_x,truth_y,truth_z,err_x,err_y,err_z,err,"
        "att_err_deg,lio_roll,lio_pitch,lio_yaw,truth_roll,truth_pitch,truth_yaw\n";
      RCLCPP_INFO(get_logger(), "LIO log: %s", file.c_str());
    }
    const Eigen::Vector3d e = lio.translation() - truth.translation();
    const double att = Eigen::AngleAxisd(truth.linear().transpose() * lio.linear()).angle();
    const Eigen::Vector3d a = rpy_deg(lio.linear()), b = rpy_deg(truth.linear());
    csv_ << std::fixed << std::setprecision(5) << t.seconds() << "," << int(released_) << ","
         << lio.translation().x() << "," << lio.translation().y() << "," << lio.translation().z() << ","
         << truth.translation().x() << "," << truth.translation().y() << "," << truth.translation().z()
         << "," << e.x() << "," << e.y() << "," << e.z() << "," << e.norm() << ","
         << att * 180.0 / M_PI << "," << a.x() << "," << a.y() << "," << a.z() << ","
         << b.x() << "," << b.y() << "," << b.z() << "\n";
    if (++n_logged_ % 50 == 0) {
      csv_.flush();
      RCLCPP_INFO(get_logger(), "LIO vs Gazebo at %.1f s: position error %.3f m, attitude %.2f deg",
        t.seconds(), e.norm(), att * 180.0 / M_PI);
    }
  }

  std::string mode_, world_, lio_frame_, truth_frame_, data_dir_;
  double settle_time_, release_timeout_;
  int64_t min_scans_;
  long n_odom_ = 0, n_logged_ = 0;
  bool anchored_ = false, released_ = false;
  rclcpp::Time t_first_odom_{0, 0, RCL_ROS_TIME}, t_start_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point release_wall_;
  Eigen::Isometry3d T_w_ci_ = Eigen::Isometry3d::Identity();
  std::deque<nav_msgs::msg::Odometry> pending_;
  std::ofstream csv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr release_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioAnchor>());
  rclcpp::shutdown();
  return 0;
}
