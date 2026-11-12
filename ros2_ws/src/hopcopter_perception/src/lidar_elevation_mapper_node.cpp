// lidar_elevation_mapper: builds /elevation_map from LiDAR scans (stage 3 of
// docs/lidar_mapping_plan.md). The map core is elevation_mapper.cpp.
//
// Subscribes:  cloud_topic (sensor_msgs/PointCloud2, any frame). Each cloud's
//                transform to map_frame, and the sensor origin, are looked up
//                in TF at the cloud's stamp, so one node serves both inputs:
//                 - the raw scan with Gazebo's pose (stage 3a; a mocap lab):
//                   /lidar/points in cf_0/lidar, TF from gz_truth_tf;
//                 - FAST-LIO (stage 3b; the real robot): the same scan read as
//                   FAST-LIO's IMU frame (cloud_frame:=body), TF world ->
//                   camera_init (lio_anchor) -> body (FAST-LIO); or FAST-LIO's
//                   dense /cloud_registered in camera_init with sensor_frame:=body.
//              /hopcopter/foot_contact (geometry_msgs/PointStamped, map frame):
//                every touchdown's foot, a near-certain height; the first one
//                also fills the start disk.
//              known_map_topic (grid_map_msgs/GridMap, transient_local): the
//                scenario's known map. Its geometry is adopted when map.size_x
//                is 0, and it is what the map is scored against.
// Publishes:   /elevation_map (grid_map_msgs/GridMap, transient_local) at
//                publish_rate: layers elevation (NaN = no usable height),
//                upper_bound (NaN = no ray has bounded it), variance, hits.
// Writes:      data/data_elevation_map/score_<stamp>.csv (one row per publish,
//                with a known map) and snapshot_<stamp>_<n>.json every
//                snapshot_period, plus one at shutdown, for
//                scripts/plot_elevation_map.py.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hopcopter_perception/elevation_mapper.hpp"

using hopcopter_perception::ElevationMapper;
using hopcopter_perception::MapperParams;

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

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return std::nan("");
  }
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<size_t>(q * (v.size() - 1) + 0.5))];
}
}  // namespace

class LidarElevationMapper : public rclcpp::Node
{
public:
  LidarElevationMapper()
  : Node("lidar_elevation_mapper")
  {
    cloud_topic_ = declare_parameter("cloud_topic", std::string("/lidar/points"));
    map_frame_ = declare_parameter("map_frame", std::string("world"));
    cloud_frame_ = declare_parameter("cloud_frame", std::string(""));    // "" = the header's
    sensor_frame_ = declare_parameter("sensor_frame", std::string(""));  // "" = the cloud's
    known_topic_ = declare_parameter("known_map_topic", std::string("/elevation_map_known"));
    out_topic_ = declare_parameter("map_topic", std::string("/elevation_map"));
    contact_topic_ = declare_parameter("foot_contact_topic", std::string("/hopcopter/foot_contact"));
    publish_rate_ = declare_parameter("publish_rate", 2.0);
    max_wait_ = declare_parameter("tf_max_wait", 1.0);  // [s] a cloud waits this long for its TF
    snapshot_period_ = declare_parameter("snapshot_period", 10.0);
    data_dir_ = declare_parameter(
      "data_dir", std::string("/home/zweminhtetaung/CrazySim/data/data_elevation_map"));
    missed_obstacle_height_ = declare_parameter("score.missed_obstacle_height", 1.0);
    // The LiDAR runs at 10 Hz. More scans than this per second of scan time, or a
    // scan not newer than the last, means a second publisher on the cloud topic
    // (a node left over from an earlier launch): warn (docs/lidar_map_quality_plan.md).
    max_scan_rate_ = declare_parameter("max_scan_rate", 11.0);

    MapperParams d;
    p_.resolution = declare_parameter("map.resolution", d.resolution);
    p_.size_x = declare_parameter("map.size_x", 0.0);  // 0: the known map's geometry
    p_.size_y = declare_parameter("map.size_y", 0.0);
    p_.origin_x = declare_parameter("map.origin_x", d.origin_x);
    p_.origin_y = declare_parameter("map.origin_y", d.origin_y);
    p_.min_range = declare_parameter("height.min_range", d.min_range);
    p_.max_update_range = declare_parameter("height.max_update_range", d.max_update_range);
    p_.range_sigma = declare_parameter("height.range_sigma", d.range_sigma);
    p_.attitude_sigma = declare_parameter("height.attitude_sigma_deg", d.attitude_sigma * 180.0 / M_PI) *
      M_PI / 180.0;
    p_.min_sigma = declare_parameter("height.min_sigma", d.min_sigma);
    p_.min_cell_sigma = declare_parameter("height.min_cell_sigma", d.min_cell_sigma);
    p_.mahalanobis_threshold = declare_parameter("height.mahalanobis_threshold", d.mahalanobis_threshold);
    p_.multi_height_noise = declare_parameter("height.multi_height_noise", d.multi_height_noise);
    p_.max_inflated_sigma = declare_parameter("height.max_inflated_sigma", d.max_inflated_sigma);
    p_.contact_sigma = declare_parameter("height.contact_sigma", d.contact_sigma);
    p_.min_hits = static_cast<int>(declare_parameter("height.min_hits", static_cast<int64_t>(d.min_hits)));
    p_.ray_carving = declare_parameter("bound.ray_carving", d.ray_carving);
    p_.carve_max_range = declare_parameter("bound.max_range", d.carve_max_range);
    p_.carve_margin = declare_parameter("bound.margin", d.carve_margin);
    p_.carve_end_skip = declare_parameter("bound.end_skip", d.carve_end_skip);
    p_.carve_clear_margin = declare_parameter("bound.clear_margin", d.carve_clear_margin);
    p_.carve_clear_rays = static_cast<int>(
      declare_parameter("bound.clear_rays", static_cast<int64_t>(d.carve_clear_rays)));
    p_.trusted_hits = static_cast<int>(
      declare_parameter("bound.trusted_hits", static_cast<int64_t>(d.trusted_hits)));
    p_.carve_center_tolerance = declare_parameter("bound.center_tolerance", d.carve_center_tolerance);
    p_.require_top_confirmation = declare_parameter("confirm.require", d.require_top_confirmation);
    p_.confirm_margin = declare_parameter("confirm.margin", d.confirm_margin);
    p_.start_disk_radius = declare_parameter("start_disk_radius", d.start_disk_radius);
    // Face points and edge attribution (docs/lidar_map_quality_plan.md, Fix 2).
    p_.face_filter = declare_parameter("face.filter", d.face_filter);
    p_.face_min_dz = declare_parameter("face.min_dz", d.face_min_dz);
    p_.face_max_dz = declare_parameter("face.max_dz", d.face_max_dz);
    p_.face_radius = declare_parameter("face.radius", d.face_radius);
    p_.face_min_slope = declare_parameter("face.min_slope", d.face_min_slope);
    p_.edge_attribution = declare_parameter("edge.attribution", d.edge_attribution);
    p_.edge_sigmas = declare_parameter("edge.sigmas", d.edge_sigmas);
    p_.edge_min_support = static_cast<int>(
      declare_parameter("edge.min_support", static_cast<int64_t>(d.edge_min_support)));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto latched = rclcpp::QoS(1).transient_local();
    map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(out_topic_, latched);
    known_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
      known_topic_, latched, [this](grid_map_msgs::msg::GridMap::SharedPtr msg) {onKnown(*msg);});
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS().keep_last(10),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        clouds_.push_back(msg);
        while (clouds_.size() > 30) {
          clouds_.pop_front();
          ++n_dropped_;
        }
      });
    contact_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      contact_topic_, rclcpp::QoS(50),
      [this](geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!mapper_) {
          pending_contacts_.push_back(*msg);
          return;
        }
        mapper_->add_foot_contact(msg->point.x, msg->point.y, msg->point.z);
        ++n_contacts_;
      });

    if (p_.size_x > 0.0 && p_.size_y > 0.0) {
      makeMapper();
    }
    process_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {processClouds();});
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_)),
      [this]() {publish();});

    std::error_code ec;
    std::filesystem::create_directories(data_dir_, ec);
    run_stamp_ = wall_stamp();
    RCLCPP_INFO(get_logger(),
      "LiDAR elevation mapper: %s -> %s (%s), heights from hits within %.1f m; %s",
      cloud_topic_.c_str(), out_topic_.c_str(), map_frame_.c_str(), p_.max_update_range,
      mapper_ ? "map geometry from parameters" : ("waiting for " + known_topic_ +
      " for the map geometry").c_str());
  }

  ~LidarElevationMapper() override
  {
    if (mapper_) {
      writeSnapshot();
    }
  }

private:
  void makeMapper()
  {
    mapper_ = std::make_unique<ElevationMapper>(p_);
    for (const auto & c : pending_contacts_) {
      mapper_->add_foot_contact(c.point.x, c.point.y, c.point.z);
      ++n_contacts_;
    }
    pending_contacts_.clear();
    RCLCPP_INFO(get_logger(), "Map %.1f x %.1f m at %.2f m, origin (%.2f, %.2f): %d x %d cells",
      p_.size_x, p_.size_y, p_.resolution, p_.origin_x, p_.origin_y, mapper_->cols(), mapper_->rows());
  }

  void onKnown(const grid_map_msgs::msg::GridMap & msg)
  {
    grid_map::GridMap gm;
    grid_map::GridMapRosConverter::fromMessage(msg, gm);
    if (!gm.exists("elevation")) {
      return;
    }
    if (!mapper_) {
      p_.resolution = gm.getResolution();
      p_.size_x = gm.getLength().x();
      p_.size_y = gm.getLength().y();
      p_.origin_x = gm.getPosition().x() - p_.size_x / 2.0;
      p_.origin_y = gm.getPosition().y() - p_.size_y / 2.0;
      makeMapper();
    }
    known_.assign(static_cast<size_t>(mapper_->rows()) * mapper_->cols(), std::nan(""));
    for (int r = 0; r < mapper_->rows(); ++r) {
      for (int c = 0; c < mapper_->cols(); ++c) {
        const grid_map::Position pos(mapper_->cell_x(c), mapper_->cell_y(r));
        if (gm.isInside(pos)) {
          known_[mapper_->index(r, c)] = gm.atPosition("elevation", pos);
        }
      }
    }
    have_known_ = true;
  }

  bool lookup(const std::string & frame, const rclcpp::Time & stamp, Eigen::Isometry3d & T)
  {
    try {
      T = tf2::transformToEigen(tf_buffer_->lookupTransform(map_frame_, frame, stamp));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void checkScanRate(const rclcpp::Time & stamp)
  {
    const double t = stamp.seconds();
    if (!scan_times_.empty() && t <= scan_times_.back()) {
      if (++n_out_of_order_ % 20 == 1) {
        RCLCPP_WARN(get_logger(), "Scan at %.3f s is not newer than the last (%.3f s): %ld so far. "
          "Is a second node publishing %s?", t, scan_times_.back(), n_out_of_order_,
          cloud_topic_.c_str());
      }
    }
    scan_times_.push_back(t);
    while (!scan_times_.empty() && scan_times_.front() < t - 2.0) {
      scan_times_.pop_front();
    }
    const double span = t - scan_times_.front();
    if (span >= 1.5) {
      const double rate = (scan_times_.size() - 1) / span;
      const auto wall = std::chrono::steady_clock::now();
      if (rate > max_scan_rate_ && wall - last_rate_warn_ > std::chrono::seconds(5)) {
        last_rate_warn_ = wall;
        RCLCPP_WARN(get_logger(), "%.1f scans/s on %s (expected ~10): is a second node publishing "
          "it? (pgrep -af hopcopter_perception)", rate, cloud_topic_.c_str());
      }
    }
  }

  void processClouds()
  {
    if (!mapper_) {
      return;
    }
    while (!clouds_.empty()) {
      const auto & msg = clouds_.front();
      const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
      const std::string frame = cloud_frame_.empty() ? msg->header.frame_id : cloud_frame_;
      const std::string sensor = sensor_frame_.empty() ? frame : sensor_frame_;
      Eigen::Isometry3d T_cloud, T_sensor;
      if (!lookup(frame, stamp, T_cloud) || !lookup(sensor, stamp, T_sensor)) {
        const rclcpp::Time newest(clouds_.back()->header.stamp, RCL_ROS_TIME);
        if ((newest - stamp).seconds() > max_wait_) {
          clouds_.pop_front();
          if (++n_no_tf_ % 50 == 1) {
            RCLCPP_WARN(get_logger(), "No transform %s <- %s (or %s) at %.3f: dropped %ld scan(s)",
              map_frame_.c_str(), frame.c_str(), sensor.c_str(), stamp.seconds(), n_no_tf_);
          }
          continue;
        }
        return;  // wait for its TF
      }
      const auto t0 = std::chrono::steady_clock::now();
      pts_.clear();
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz)) {
          pts_.push_back(T_cloud * Eigen::Vector3d(*ix, *iy, *iz));
        }
      }
      checkScanRate(stamp);
      mapper_->integrate_scan(T_sensor.translation(), pts_);
      const double ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      scan_ms_sum_ += ms;
      scan_ms_max_ = std::max(scan_ms_max_, ms);
      last_stamp_ = stamp;
      clouds_.pop_front();
    }
  }

  void publish()
  {
    if (!mapper_ || mapper_->scans() == 0) {
      return;
    }
    const auto & m = *mapper_;
    grid_map::GridMap gm({"elevation", "upper_bound", "variance", "hits"});
    gm.setFrameId(map_frame_);
    gm.setGeometry(grid_map::Length(p_.size_x, p_.size_y), p_.resolution,
      grid_map::Position(p_.origin_x + p_.size_x / 2.0, p_.origin_y + p_.size_y / 2.0));
    auto & el = gm["elevation"];
    auto & ub = gm["upper_bound"];
    auto & var = gm["variance"];
    auto & hits = gm["hits"];
    for (int r = 0; r < m.rows(); ++r) {
      for (int c = 0; c < m.cols(); ++c) {
        grid_map::Index idx;
        if (!gm.getIndex(grid_map::Position(m.cell_x(c), m.cell_y(r)), idx)) {
          continue;
        }
        const size_t i = m.index(r, c);
        el(idx(0), idx(1)) = static_cast<float>(m.elevation(i));
        ub(idx(0), idx(1)) = static_cast<float>(m.upper_bound(i));
        const double v = m.variance(i);
        var(idx(0), idx(1)) = std::isfinite(v) && m.measured(i) ? static_cast<float>(v) : NAN;
        hits(idx(0), idx(1)) = static_cast<float>(m.hits(i));
      }
    }
    gm.setTimestamp(last_stamp_.nanoseconds());
    map_pub_->publish(*grid_map::GridMapRosConverter::toMessage(gm));
    ++n_published_;
    if (have_known_) {
      score();
    }
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_snapshot_).count() >= snapshot_period_) {
      last_snapshot_ = now;
      writeSnapshot();
    }
    if (n_published_ % static_cast<long>(std::max(1.0, 10.0 * publish_rate_)) == 0) {
      RCLCPP_INFO(get_logger(),
        "%ld scans (%.1f ms mean, %.1f max), %ld contacts, %ld dropped without TF, %ld heights "
        "cleared by rays, %ld rays through surfaces dropped; face points %.1f%% (%ld hits ignored), "
        "%ld raises credited to a neighbour", m.scans(),
        scan_ms_sum_ / std::max(1L, m.scans()), scan_ms_max_, n_contacts_, n_no_tf_,
        m.heights_cleared(), m.rays_through_surfaces(),
        100.0 * m.face_points() / std::max(1L, m.points_classified()), m.face_hits_ignored(),
        m.hits_attributed());
    }
  }

  // Coverage and error against the known map, one CSV row per publish.
  void score()
  {
    const auto & m = *mapper_;
    const int R = m.rows(), C = m.cols();
    auto known = [&](int r, int c) {return known_[m.index(r, c)];};
    auto near_edge = [&](int r, int c) {
        const double z = known(r, c);
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int rr = r + dr, cc = c + dc;
            if (rr < 0 || cc < 0 || rr >= R || cc >= C) {continue;}
            const double zz = known(rr, cc);
            if (std::isnan(z) != std::isnan(zz) || std::abs(zz - z) > 0.05) {return true;}
          }
        }
        return false;
      };
    std::vector<double> err;
    double signed_sum = 0.0;
    int finite = 0, usable = 0, bounded = 0, unbounded = 0, false_flat = 0, false_edge = 0;
    int missed = 0, bound_viol = 0;
    for (int r = 0; r < R; ++r) {
      for (int c = 0; c < C; ++c) {
        const size_t i = m.index(r, c);
        const double k = known(r, c);
        finite += std::isfinite(k);
        const double e = m.elevation(i);
        if (!std::isnan(e)) {
          if (m.flags(i) & (ElevationMapper::kAssumed | ElevationMapper::kContact)) {
            continue;  // assumed or a contact, not a LiDAR measurement
          }
          ++usable;
          if (std::isnan(k)) {
            missed += e < missed_obstacle_height_;
            continue;
          }
          const double d = e - k;
          if (!near_edge(r, c)) {
            err.push_back(std::abs(d));
            signed_sum += d;
          }
          if (d > 0.10) {(near_edge(r, c) ? false_edge : false_flat)++;}
          if (d < -0.10) {++missed;}
          continue;
        }
        const double b = m.upper_bound(i);
        if (std::isnan(b)) {
          ++unbounded;
        } else {
          ++bounded;
          bound_viol += std::isfinite(k) && b < k - 0.05;
        }
      }
    }
    if (!score_csv_.is_open()) {
      score_csv_.open(data_dir_ + "/score_" + run_stamp_ + ".csv");
      score_csv_ << "stamp,scans,contacts,usable,known_finite,coverage,bounded_unseen,unbounded,"
        "err_median,err_p95,err_max,err_mean,false_flat,false_edge,missed,bound_violations,"
        "heights_cleared,scan_ms_mean,scan_ms_max,face_points_pct,face_hits_ignored,"
        "raises_credited\n";
      RCLCPP_INFO(get_logger(), "Scoring against %s: %s/score_%s.csv", known_topic_.c_str(),
        data_dir_.c_str(), run_stamp_.c_str());
    }
    score_csv_ << std::fixed << std::setprecision(4) << last_stamp_.seconds() << "," << m.scans() << ","
               << n_contacts_ << "," << usable << "," << finite << ","
               << static_cast<double>(usable) / std::max(1, finite) << "," << bounded << ","
               << unbounded << "," << percentile(err, 0.5) << "," << percentile(err, 0.95) << ","
               << percentile(err, 1.0) << "," << (err.empty() ? NAN : signed_sum / err.size()) << ","
               << false_flat << "," << false_edge << "," << missed << "," << bound_viol << ","
               << m.heights_cleared() << "," << scan_ms_sum_ / std::max(1L, m.scans()) << ","
               << scan_ms_max_ << ","
               << 100.0 * m.face_points() / std::max(1L, m.points_classified()) << ","
               << m.face_hits_ignored() << "," << m.hits_attributed() << "\n";
    score_csv_.flush();
  }

  void writeSnapshot()
  {
    const auto & m = *mapper_;
    const std::string file = data_dir_ + "/snapshot_" + run_stamp_ + "_" +
      std::to_string(n_snapshots_++) + ".json";
    std::ofstream f(file);
    if (!f) {
      return;
    }
    auto arr = [&](const char * key, auto getter) {
        f << "\"" << key << "\": [";
        for (int i = 0; i < m.rows() * m.cols(); ++i) {
          const double v = getter(static_cast<size_t>(i));
          f << (i ? "," : "");
          if (std::isfinite(v)) {f << v;} else {f << "null";}
        }
        f << "],\n";
      };
    f << std::setprecision(6) << "{\n\"stamp\": " << last_stamp_.seconds() << ",\n\"scans\": "
      << m.scans() << ",\n\"origin_x\": " << p_.origin_x << ",\n\"origin_y\": " << p_.origin_y
      << ",\n\"resolution\": " << p_.resolution << ",\n\"rows\": " << m.rows() << ",\n\"cols\": "
      << m.cols() << ",\n";
    arr("elevation", [&](size_t i) {return m.elevation(i);});
    arr("upper_bound", [&](size_t i) {return m.upper_bound(i);});
    arr("raw", [&](size_t i) {return m.raw_height(i);});
    arr("hits", [&](size_t i) {return static_cast<double>(m.hits(i));});
    arr("flags", [&](size_t i) {return static_cast<double>(m.flags(i));});
    if (have_known_) {
      arr("known", [&](size_t i) {return known_[i];});
    }
    f << "\"format\": 1\n}\n";
  }

  std::string cloud_topic_, map_frame_, cloud_frame_, sensor_frame_, known_topic_, out_topic_;
  std::string contact_topic_, data_dir_, run_stamp_;
  double publish_rate_, max_wait_, snapshot_period_, missed_obstacle_height_;
  MapperParams p_;
  std::unique_ptr<ElevationMapper> mapper_;
  std::vector<double> known_;
  bool have_known_ = false;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> clouds_;
  std::vector<geometry_msgs::msg::PointStamped> pending_contacts_;
  std::vector<Eigen::Vector3d> pts_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  long n_no_tf_ = 0, n_dropped_ = 0, n_contacts_ = 0, n_published_ = 0, n_snapshots_ = 0;
  double scan_ms_sum_ = 0.0, scan_ms_max_ = 0.0, max_scan_rate_ = 11.0;
  std::deque<double> scan_times_;  // [s] stamps of the scans of the last 2 s
  long n_out_of_order_ = 0;
  std::chrono::steady_clock::time_point last_rate_warn_{};
  std::chrono::steady_clock::time_point last_snapshot_ = std::chrono::steady_clock::now();
  std::ofstream score_csv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr map_pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr known_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr contact_sub_;
  rclcpp::TimerBase::SharedPtr process_timer_, publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarElevationMapper>();
  rclcpp::spin(node);
  node.reset();  // writes the last snapshot
  rclcpp::shutdown();
  return 0;
}
