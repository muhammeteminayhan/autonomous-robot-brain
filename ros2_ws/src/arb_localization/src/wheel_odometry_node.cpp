// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Integrates the preprocessed wheel encoders into a pose estimate and
// publishes it as nav_msgs/Odometry plus the odom -> base_link transform.
//
// WHY THIS NODE PUBLISHES odom -> base_link AND NOT map -> base_link
// ==================================================================
// REP-105 splits the localisation chain in two on purpose:
//
//   map  --(correction, slow, DISCONTINUOUS)-->  odom
//   odom --(dead reckoning, fast, SMOOTH)-->     base_link
//
// Dead reckoning is continuous and differentiable but drifts without bound.
// A corrected global estimate is bounded but JUMPS whenever a correction
// arrives. Controllers need the first property: a jump in the transform they
// are tracking would appear as a step in the error signal and jerk the robot.
// Planners need the second: a plan drawn in a drifting frame slowly stops
// matching the world.
//
// Keeping the correction in map -> odom lets both hold at once. Writing the
// correction into odom -> base_link instead destroys both, and is the single
// most common way to break a ROS navigation stack.
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "arb_localization/differential_drive_odometry.hpp"

namespace arb_localization
{

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : rclcpp::Node("wheel_odometry")
  {
    OdometryConfig cfg;
    cfg.wheel_radius_m = declare_parameter("wheel_radius_m", 0.06);
    cfg.wheel_separation_m = declare_parameter("wheel_separation_m", 0.35);
    const int counts = static_cast<int>(declare_parameter("counts_per_revolution", 1024));
    const int quad = static_cast<int>(declare_parameter("quadrature_multiplier", 4));
    cfg.encoder_resolution_rad =
      (counts > 0 && quad > 0) ? (2.0 * M_PI / (counts * quad)) : 0.0;
    cfg.k_right = declare_parameter("k_right", 0.002);
    cfg.k_left = declare_parameter("k_left", 0.002);
    cfg.min_variance_per_step = declare_parameter("min_variance_per_step", 1e-9);
    cfg.max_dt_s = declare_parameter("max_dt_s", 0.5);
    cfg.max_wheel_step_rad = declare_parameter("max_wheel_step_rad", 2.0);
    cfg.left_joint = declare_parameter("left_joint", std::string("left_wheel_joint"));
    cfg.right_joint = declare_parameter("right_joint", std::string("right_wheel_joint"));

    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);

    const std::string input = declare_parameter("input_topic", std::string("/arb/wheel_states"));
    const std::string output = declare_parameter("output_topic", std::string("/arb/odom"));

    odometry_ = std::make_unique<DifferentialDriveOdometry>(cfg);
    config_ = cfg;

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_wheels(*msg);});

    if (publish_tf_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    report_timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Wheel odometry: %s -> %s | r=%.4f m, L=%.4f m, v_max=%.3f m/s at the "
      "configured wheel limit | publishing %s -> %s",
      input.c_str(), output.c_str(), cfg.wheel_radius_m, cfg.wheel_separation_m,
      cfg.wheel_radius_m * 12.0, odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  static int index_of(const sensor_msgs::msg::JointState & js, const std::string & name)
  {
    for (std::size_t i = 0; i < js.name.size(); ++i) {
      if (js.name[i] == name) {return static_cast<int>(i);}
    }
    return -1;
  }

  void on_wheels(const sensor_msgs::msg::JointState & msg)
  {
    ++received_;
    const int li = index_of(msg, config_.left_joint);
    const int ri = index_of(msg, config_.right_joint);
    if (li < 0 || ri < 0 ||
        msg.position.size() <= static_cast<std::size_t>(std::max(li, ri)))
    {
      ++missing_joint_;
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    const auto step = odometry_->update(
      msg.position[li], msg.position[ri], stamp.seconds());

    if (!step.valid) {
      switch (step.reason) {
        case OdometryReject::kFirstSample: break;
        case OdometryReject::kNonFinite: ++rejected_nonfinite_; break;
        case OdometryReject::kNonMonotonicTime: ++rejected_time_; break;
        case OdometryReject::kImplausibleStep:
          ++rejected_step_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "implausible wheel step rejected (>%.2f rad in one sample); "
            "a dropped or corrupted encoder reading",
            config_.max_wheel_step_rad);
          break;
        default: break;
      }
      return;
    }
    ++integrated_;

    const auto & pose = odometry_->pose();
    const auto & P = odometry_->covariance();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg.header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // nav_msgs/Odometry carries a 6x6 covariance over (x, y, z, roll, pitch,
    // yaw). Only the planar block is estimated. The unobserved rows are given
    // a large but finite variance rather than zero: zero would claim perfect
    // knowledge of z, roll and pitch, and any consumer that inverts the matrix
    // would be inverting a singular one.
    odom.pose.covariance.fill(0.0);
    odom.pose.covariance[0] = P(0, 0);   // xx
    odom.pose.covariance[1] = P(0, 1);   // xy
    odom.pose.covariance[5] = P(0, 2);   // x-yaw
    odom.pose.covariance[6] = P(1, 0);
    odom.pose.covariance[7] = P(1, 1);
    odom.pose.covariance[11] = P(1, 2);
    odom.pose.covariance[30] = P(2, 0);
    odom.pose.covariance[31] = P(2, 1);
    odom.pose.covariance[35] = P(2, 2);  // yaw-yaw
    odom.pose.covariance[14] = kUnobservedVariance;   // zz
    odom.pose.covariance[21] = kUnobservedVariance;   // roll
    odom.pose.covariance[28] = kUnobservedVariance;   // pitch

    odom.twist.twist.linear.x = step.linear_velocity;
    odom.twist.twist.angular.z = step.angular_velocity;
    odom.twist.covariance.fill(0.0);
    // Velocity uncertainty is the per-step position uncertainty divided by the
    // interval, since velocity here is a difference over dt and nothing else.
    const double dt = step.dt_s;
    const double quant_m = config_.encoder_resolution_rad * config_.wheel_radius_m;
    const double v_var = 2.0 * quant_m * quant_m / 12.0 / (dt * dt);
    odom.twist.covariance[0] = v_var;
    odom.twist.covariance[35] =
      4.0 * v_var / (config_.wheel_separation_m * config_.wheel_separation_m);
    odom.twist.covariance[7] = kUnobservedVariance;
    odom.twist.covariance[14] = kUnobservedVariance;
    odom.twist.covariance[21] = kUnobservedVariance;
    odom.twist.covariance[28] = kUnobservedVariance;

    publisher_->publish(odom);

    if (broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg.header.stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose.x;
      tf.transform.translation.y = pose.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom.pose.pose.orientation;
      broadcaster_->sendTransform(tf);
    }
  }

  void report()
  {
    const auto & pose = odometry_->pose();
    const auto & P = odometry_->covariance();
    RCLCPP_INFO(
      get_logger(),
      "odom: pose=(%.3f, %.3f, %.1f deg) travelled=%.2f m | 1-sigma: "
      "pos=%.3f m heading=%.2f deg | received=%lu integrated=%lu "
      "rejected: time=%lu step=%lu non-finite=%lu missing=%lu",
      pose.x, pose.y, pose.theta * 180.0 / M_PI, odometry_->distance_travelled_m(),
      std::sqrt(std::max(0.0, P(0, 0) + P(1, 1))),
      std::sqrt(std::max(0.0, P(2, 2))) * 180.0 / M_PI,
      received_, integrated_, rejected_time_, rejected_step_,
      rejected_nonfinite_, missing_joint_);
  }

  static constexpr double kUnobservedVariance = 1e6;

  OdometryConfig config_;
  std::unique_ptr<DifferentialDriveOdometry> odometry_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};

  unsigned long received_{0}, integrated_{0};
  unsigned long rejected_time_{0}, rejected_step_{0};
  unsigned long rejected_nonfinite_{0}, missing_joint_{0};
};

}  // namespace arb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_localization::WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
