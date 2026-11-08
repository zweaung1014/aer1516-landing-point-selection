// gz_truth_tf: Gazebo's ground truth as TF (sim only).
//
// Broadcasts world -> cf_0/base_link from /cf_0/odom (the OdometryPublisher's
// model pose, 200 Hz, stamped in sim time), and the static cf_0/base_link ->
// cf_0/lidar: the Mid-360's frame in model.sdf.jinja (0.4 m up, rolled 180 deg;
// the LiDAR and its IMU share it). This is the pose a mocap lab would give: the
// mapper uses it in stage 3a, and lio_anchor scores FAST-LIO against it.
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class GzTruthTf : public rclcpp::Node
{
public:
  GzTruthTf()
  : Node("gz_truth_tf")
  {
    world_ = declare_parameter("world_frame", std::string("world"));
    base_ = declare_parameter("base_frame", std::string("cf_0/base_link"));
    lidar_ = declare_parameter("lidar_frame", std::string("cf_0/lidar"));
    const auto odom_topic = declare_parameter("odom_topic", std::string("/cf_0/odom"));
    // base_link -> LiDAR: x y z [m], roll pitch yaw [rad] (model.sdf.jinja's lidar_sensor pose)
    const auto xyzrpy = declare_parameter(
      "lidar_pose", std::vector<double>{0.0, 0.0, 0.4, M_PI, 0.0, 0.0});

    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped s;
    s.header.stamp = now();
    s.header.frame_id = base_;
    s.child_frame_id = lidar_;
    s.transform.translation.x = xyzrpy.at(0);
    s.transform.translation.y = xyzrpy.at(1);
    s.transform.translation.z = xyzrpy.at(2);
    tf2::Quaternion q;
    q.setRPY(xyzrpy.at(3), xyzrpy.at(4), xyzrpy.at(5));
    s.transform.rotation.x = q.x();
    s.transform.rotation.y = q.y();
    s.transform.rotation.z = q.z();
    s.transform.rotation.w = q.w();
    static_tf_->sendTransform(s);

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(50), [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg->header.stamp;
        t.header.frame_id = world_;
        t.child_frame_id = base_;
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;
        const auto & r = t.transform.rotation;
        if (r.x == 0.0 && r.y == 0.0 && r.z == 0.0 && r.w == 0.0) {
          return;  // the first Gazebo message can carry a zero quaternion
        }
        tf_->sendTransform(t);
      });
    RCLCPP_INFO(get_logger(), "Broadcasting %s -> %s from %s, and %s -> %s (static)",
      world_.c_str(), base_.c_str(), odom_topic.c_str(), base_.c_str(), lidar_.c_str());
  }

private:
  std::string world_, base_, lidar_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzTruthTf>());
  rclcpp::shutdown();
  return 0;
}
