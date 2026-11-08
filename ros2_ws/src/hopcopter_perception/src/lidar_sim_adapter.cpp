// lidar_sim_adapter: makes Gazebo's LiDAR look like the real one to FAST-LIO
// (sim only; stage 1 of docs/lidar_mapping_plan.md).
//
// Points: /cf_0/lidar/points -> /lidar/points (every finite point outside the
// blind radius, for the mapper) and /lio/points (every lio_point_stride-th of
// them, for FAST-LIO). Gazebo's gpu_lidar renders each scan at one instant, so
// every point's time offset is 0, which is correct; FAST-LIO's default input
// handler (lidar_type 5 in its config) keeps it. That handler does not drop the
// rays that hit nothing (non-finite), so this does, and it ignores
// point_filter_num, so the stride stands in for it: the Mid-360 config FAST-LIO
// ships keeps every 3rd point. With all 20 k, a scan took up to 51 ms, and
// FAST-LIO's 10-deep IMU queue (50 ms at 200 Hz, on the same thread) can drop
// a touchdown's samples. The outputs keep x, y, z and intensity, the frame and
// the stamp.
// IMU: /cf_0/lidar_imu -> /lio/imu, averaged down to imu.output_rate (200 Hz,
// the Mid-360's): Gazebo samples its IMU instantaneously, which at 200 Hz
// aliases a touchdown (one stance's x velocity came out 6 m/s wrong), where a
// real IMU averages through its anti-aliasing filter. So Gazebo runs it at 1 kHz
// and each output is the mean of a window, stamped at the window's mean time:
// integrating the outputs keeps the velocity change. Then it is optionally
// clipped at the real sensor's range (imu.acc_limit_g, imu.gyro_limit_dps;
// 0 = off), so saturation at touchdown can be switched on and off.
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LidarSimAdapter : public rclcpp::Node
{
public:
  LidarSimAdapter()
  : Node("lidar_sim_adapter")
  {
    const auto in_points = declare_parameter("points_in", std::string("/cf_0/lidar/points"));
    const auto out_points = declare_parameter("points_out", std::string("/lio/points"));
    const auto full_points = declare_parameter("points_full_out", std::string("/lidar/points"));
    stride_ = std::max<int64_t>(1, declare_parameter("lio_point_stride", int64_t{3}));
    const auto in_imu = declare_parameter("imu_in", std::string("/cf_0/lidar_imu"));
    const auto out_imu = declare_parameter("imu_out", std::string("/lio/imu"));
    blind_ = declare_parameter("blind", 0.3);
    max_range_ = declare_parameter("max_range", 40.0);
    frame_ = declare_parameter("frame_id", std::string(""));  // "" keeps the input's
    acc_limit_ = declare_parameter("imu.acc_limit_g", 0.0) * 9.80665;
    gyro_limit_ = declare_parameter("imu.gyro_limit_dps", 0.0) * M_PI / 180.0;
    const double rate = declare_parameter("imu.output_rate", 200.0);  // 0: pass every sample
    period_ = rate > 0.0 ? 1.0 / rate : 0.0;

    pts_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_points, rclcpp::QoS(5));
    full_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(full_points, rclcpp::QoS(5));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(out_imu, rclcpp::QoS(400));
    pts_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_points, rclcpp::SensorDataQoS().keep_last(5),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {onPoints(*msg);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      in_imu, rclcpp::QoS(400), [this](sensor_msgs::msg::Imu::SharedPtr msg) {onImu(*msg);});
    RCLCPP_INFO(get_logger(),
      "%s -> %s (finite, %.2f-%.1f m) and every %ldth point to %s; %s -> %s (averaged to %.0f Hz; "
      "clip: acc %s, gyro %s)", in_points.c_str(), full_points.c_str(), blind_, max_range_,
      static_cast<long>(stride_), out_points.c_str(), in_imu.c_str(), out_imu.c_str(), rate,
      acc_limit_ > 0.0 ? (std::to_string(acc_limit_ / 9.80665) + " g").c_str() : "off",
      gyro_limit_ > 0.0 ? (std::to_string(gyro_limit_ * 180.0 / M_PI) + " dps").c_str() : "off");
  }

private:
  void onPoints(const sensor_msgs::msg::PointCloud2 & in)
  {
    bool has_i = false;
    for (const auto & f : in.fields) {
      has_i = has_i || f.name == "intensity";
    }
    sensor_msgs::msg::PointCloud2 out;
    out.header = in.header;
    if (!frame_.empty()) {
      out.header.frame_id = frame_;
    }
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    mod.resize(static_cast<size_t>(in.width) * in.height);
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z"), oi(out, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> ix(in, "x"), iy(in, "y"), iz(in, "z");
    std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> ii;
    if (has_i) {
      ii = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(in, "intensity");
    }
    size_t n = 0;
    const double b2 = blind_ * blind_, m2 = max_range_ * max_range_;
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      const float i = ii ? **ii : 0.0f;
      if (ii) {
        ++(*ii);
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const double r2 = static_cast<double>(x) * x + static_cast<double>(y) * y +
        static_cast<double>(z) * z;
      if (r2 < b2 || r2 > m2) {
        continue;
      }
      *ox = x;
      *oy = y;
      *oz = z;
      *oi = i;
      ++ox;
      ++oy;
      ++oz;
      ++oi;
      ++n;
    }
    mod.resize(n);
    out.height = 1;
    out.width = static_cast<uint32_t>(n);
    out.is_dense = true;
    if (stride_ > 1) {
      sensor_msgs::msg::PointCloud2 lio;
      lio.header = out.header;
      lio.fields = out.fields;
      lio.is_bigendian = out.is_bigendian;
      lio.point_step = out.point_step;
      lio.height = 1;
      lio.is_dense = true;
      const size_t m = (n + stride_ - 1) / stride_;
      lio.data.resize(m * out.point_step);
      for (size_t k = 0; k < m; ++k) {
        std::copy_n(&out.data[k * stride_ * out.point_step], out.point_step,
          &lio.data[k * out.point_step]);
      }
      lio.width = static_cast<uint32_t>(m);
      lio.row_step = static_cast<uint32_t>(m * out.point_step);
      pts_pub_->publish(lio);
    } else {
      pts_pub_->publish(out);
    }
    full_pub_->publish(out);
  }

  void onImu(sensor_msgs::msg::Imu msg)
  {
    if (period_ > 0.0) {
      // accumulate a window; emit its mean once the window spans one output period
      const double t = rclcpp::Time(msg.header.stamp).seconds();
      if (t_last_ > 0.0 && t > t_last_) {
        dt_in_ = t - t_last_;  // the input's sample interval
      }
      t_last_ = t;
      if (n_ == 0) {
        t_first_ = t;
      }
      sum_t_ += t;
      sum_a_ += Eigen3(msg.linear_acceleration);
      sum_w_ += Eigen3(msg.angular_velocity);
      ++n_;
      // n samples cover n input intervals: at 1 kHz into 200 Hz, five per output
      if (t - t_first_ + dt_in_ < period_ - 1e-6) {
        return;
      }
      const double m = static_cast<double>(n_);
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(sum_t_ / m * 1e9));
      set(msg.linear_acceleration, sum_a_ / m);
      set(msg.angular_velocity, sum_w_ / m);
      n_ = 0;
      sum_t_ = 0.0;
      sum_a_ = sum_w_ = {0.0, 0.0, 0.0};
    }
    if (acc_limit_ > 0.0) {
      auto & a = msg.linear_acceleration;
      a.x = std::clamp(a.x, -acc_limit_, acc_limit_);
      a.y = std::clamp(a.y, -acc_limit_, acc_limit_);
      a.z = std::clamp(a.z, -acc_limit_, acc_limit_);
    }
    if (gyro_limit_ > 0.0) {
      auto & w = msg.angular_velocity;
      w.x = std::clamp(w.x, -gyro_limit_, gyro_limit_);
      w.y = std::clamp(w.y, -gyro_limit_, gyro_limit_);
      w.z = std::clamp(w.z, -gyro_limit_, gyro_limit_);
    }
    imu_pub_->publish(msg);
  }

  struct V3
  {
    double x, y, z;
    V3 operator+(const V3 & o) const {return {x + o.x, y + o.y, z + o.z};}
    V3 & operator+=(const V3 & o) {x += o.x; y += o.y; z += o.z; return *this;}
    V3 operator/(double d) const {return {x / d, y / d, z / d};}
  };
  static V3 Eigen3(const geometry_msgs::msg::Vector3 & v) {return {v.x, v.y, v.z};}
  static void set(geometry_msgs::msg::Vector3 & v, const V3 & a) {v.x = a.x; v.y = a.y; v.z = a.z;}

  double blind_, max_range_, acc_limit_, gyro_limit_, period_ = 0.0;
  // the IMU window being averaged
  int n_ = 0;
  double t_first_ = 0.0, sum_t_ = 0.0, t_last_ = 0.0, dt_in_ = 0.0;
  V3 sum_a_{0.0, 0.0, 0.0}, sum_w_{0.0, 0.0, 0.0};
  std::string frame_;
  int64_t stride_ = 3;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pts_pub_, full_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pts_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSimAdapter>());
  rclcpp::shutdown();
  return 0;
}
