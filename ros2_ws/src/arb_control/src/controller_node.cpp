// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// PID + pure-pursuit path follower. The alternative to the DWA local planner:
// it tracks the global path precisely and has NO obstacle avoidance, which
// makes it the honest baseline for measuring what DWA's avoidance costs in
// tracking accuracy.
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arb_control/path_follower.hpp"
#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"

namespace arb_control
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : rclcpp::Node("controller")
  {
    FollowerConfig cfg;
    cfg.linear.kp = declare_parameter("linear.kp", 1.6);
    cfg.linear.ki = declare_parameter("linear.ki", 0.35);
    cfg.linear.kd = declare_parameter("linear.kd", 0.02);
    cfg.linear.integral_limit = declare_parameter("linear.integral_limit", 0.25);
    cfg.linear.output_limit = declare_parameter("linear.output_limit", 1.0);
    cfg.linear.derivative_filter_tau =
      declare_parameter("linear.derivative_filter_tau", 0.05);

    cfg.angular.kp = declare_parameter("angular.kp", 2.2);
    cfg.angular.ki = declare_parameter("angular.ki", 0.10);
    cfg.angular.kd = declare_parameter("angular.kd", 0.12);
    cfg.angular.integral_limit = declare_parameter("angular.integral_limit", 0.4);
    cfg.angular.output_limit = declare_parameter("angular.output_limit", 2.0);
    cfg.angular.derivative_filter_tau =
      declare_parameter("angular.derivative_filter_tau", 0.05);

    cfg.lookahead_min_m = declare_parameter("lookahead_min_m", 0.35);
    cfg.lookahead_gain_s = declare_parameter("lookahead_gain_s", 0.9);
    cfg.lookahead_max_m = declare_parameter("lookahead_max_m", 1.2);
    cfg.max_linear_velocity = declare_parameter("max_linear_velocity", 0.70);
    cfg.min_linear_velocity = declare_parameter("min_linear_velocity", 0.0);
    cfg.max_angular_velocity = declare_parameter("max_angular_velocity", 2.0);
    cfg.max_linear_acceleration =
      declare_parameter("max_linear_acceleration", 1.0);
    cfg.max_angular_acceleration =
      declare_parameter("max_angular_acceleration", 3.0);
    cfg.heading_slowdown_gain = declare_parameter("heading_slowdown_gain", 1.4);
    cfg.in_place_turn_threshold_rad =
      declare_parameter("in_place_turn_threshold_rad", 1.0);
    cfg.goal_tolerance_m = declare_parameter("goal_tolerance_m", 0.20);

    follower_ = std::make_unique<PathFollower>(cfg);
    control_rate_ = declare_parameter("control_rate_hz", 20.0);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      declare_parameter("path_topic", std::string("/arb/global_path")), 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {
        path_ = m;
        // Progress along the OLD path says nothing about the new one; the PID
        // integrators, which hold the accumulated bias correction, are kept.
        follower_->onNewPath();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/arb/odom")),
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        current_v_ = m->twist.twist.linear.x;
        current_w_ = m->twist.twist.angular.z;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      declare_parameter("cmd_topic", std::string("/cmd_vel")), 10);
    // Published so the Phase 9 evaluation can measure tracking error,
    // overshoot and control smoothness without re-deriving them from the pose.
    diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      declare_parameter("diagnostics_topic", std::string("/arb/control_diagnostics")),
      10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_), [this]() {step();});
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Controller (pure pursuit + PID): linear %.2f/%.2f/%.2f, "
      "angular %.2f/%.2f/%.2f, lookahead %.2f-%.2f m, %.0f Hz",
      cfg.linear.kp, cfg.linear.ki, cfg.linear.kd,
      cfg.angular.kp, cfg.angular.ki, cfg.angular.kd,
      cfg.lookahead_min_m, cfg.lookahead_max_m, control_rate_);
  }

private:
  void publish(double v, double w)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    cmd_pub_->publish(t);
    // Control smoothness is the magnitude of the command CHANGE per step. A
    // controller that reaches the goal by chattering between extremes has a
    // fine tracking error and destroys its gearboxes.
    if (have_last_) {
      jerk_sum_ += std::abs(v - last_v_) + std::abs(w - last_w_);
      ++jerk_samples_;
    }
    last_v_ = v;
    last_w_ = w;
    have_last_ = true;
  }

  void step()
  {
    const double dt = 1.0 / control_rate_;
    if (!path_ || path_->poses.empty()) {
      publish(0.0, 0.0);
      follower_->reset();
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, map_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      ++no_tf_;
      publish(0.0, 0.0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "no transform: %s", ex.what());
      return;
    }
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const arb_core::Pose2D map_to_base(
      tf.transform.translation.x, tf.transform.translation.y, yaw);

    std::vector<arb_core::Pose2D> local;
    local.reserve(path_->poses.size());
    for (const auto & p : path_->poses) {
      local.push_back(map_to_base.compose(
          arb_core::Pose2D(p.pose.position.x, p.pose.position.y, 0.0)));
    }

    const FollowerCommand cmd = follower_->follow(local, current_v_, dt);
    ++cycles_;
    if (!cmd.at_goal) {
      cross_track_sum_ += std::abs(cmd.cross_track_error_m);
      cross_track_peak_ = std::max(cross_track_peak_,
                                   std::abs(cmd.cross_track_error_m));
      heading_sum_ += std::abs(cmd.heading_error_rad);
      ++error_samples_;
    }
    last_cmd_ = cmd;
    publish(cmd.v, cmd.omega);

    std_msgs::msg::Float64MultiArray diag;
    diag.data = {cmd.cross_track_error_m, cmd.heading_error_rad, cmd.v,
      cmd.omega, cmd.lookahead_m, cmd.at_goal ? 1.0 : 0.0,
      cmd.progress_m, cmd.distance_to_goal_m};
    diag_pub_->publish(diag);
  }

  void report()
  {
    const double n = std::max(1.0, static_cast<double>(error_samples_));
    RCLCPP_INFO(
      get_logger(),
      "control: %lu cycles | cross-track mean %.4f m peak %.4f m | "
      "heading mean %.2f deg | smoothness %.4f per step | no tf %lu",
      cycles_, cross_track_sum_ / n, cross_track_peak_,
      arb_core::radToDeg(heading_sum_ / n),
      jerk_samples_ ? jerk_sum_ / jerk_samples_ : 0.0, no_tf_);
  }

  std::unique_ptr<PathFollower> follower_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  nav_msgs::msg::Path::ConstSharedPtr path_;
  double current_v_{0.0}, current_w_{0.0};
  double last_v_{0.0}, last_w_{0.0};
  bool have_last_{false};
  double control_rate_{20.0};
  std::string map_frame_{"map"}, base_frame_{"base_link"};
  FollowerCommand last_cmd_;

  unsigned long cycles_{0}, error_samples_{0}, jerk_samples_{0}, no_tf_{0};
  double cross_track_sum_{0.0}, cross_track_peak_{0.0};
  double heading_sum_{0.0}, jerk_sum_{0.0};
};

}  // namespace arb_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_control::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
