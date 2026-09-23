// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Follows the global path while avoiding whatever the LiDAR sees, using the
// Dynamic Window Approach. Publishes /cmd_vel directly: in DWA the local
// planner IS the controller, because it searches the space of commands rather
// than the space of positions.
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"
#include "arb_planning/dynamic_window.hpp"

namespace arb_planning
{

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : rclcpp::Node("local_planner")
  {
    DwaConfig cfg;
    cfg.max_linear_velocity = declare_parameter("max_linear_velocity", 0.70);
    cfg.min_linear_velocity = declare_parameter("min_linear_velocity", -0.30);
    cfg.max_angular_velocity = declare_parameter("max_angular_velocity", 2.00);
    cfg.max_linear_acceleration =
      declare_parameter("max_linear_acceleration", 1.00);
    cfg.max_angular_acceleration =
      declare_parameter("max_angular_acceleration", 3.00);
    cfg.linear_samples = static_cast<int>(declare_parameter("linear_samples", 9));
    cfg.angular_samples = static_cast<int>(declare_parameter("angular_samples", 21));
    cfg.simulation_time = declare_parameter("simulation_time", 2.0);
    cfg.simulation_dt = declare_parameter("simulation_dt", 0.1);
    cfg.heading_weight = declare_parameter("heading_weight", 1.0);
    cfg.clearance_weight = declare_parameter("clearance_weight", 0.7);
    cfg.velocity_weight = declare_parameter("velocity_weight", 0.25);
    cfg.path_weight = declare_parameter("path_weight", 1.2);
    cfg.path_lookahead_m = declare_parameter("path_lookahead_m", 1.0);
    cfg.robot_radius = declare_parameter("robot_radius", 0.25);
    cfg.max_useful_clearance = declare_parameter("max_useful_clearance", 1.2);
    cfg.collision_margin = declare_parameter("collision_margin", 0.02);
    cfg.enforce_stopping_distance =
      declare_parameter("enforce_stopping_distance", true);
    cfg.collision_check_step_m = declare_parameter("collision_check_step_m", 0.02);
    cfg.in_place_rotation_speed =
      declare_parameter("in_place_rotation_speed", 0.05);

    planner_ = std::make_unique<DynamicWindowPlanner>(cfg);

    control_rate_ = declare_parameter("control_rate_hz", 20.0);
    goal_tolerance_ = declare_parameter("goal_tolerance_m", 0.20);
    obstacle_range_ = declare_parameter("obstacle_range_m", 3.0);
    obstacle_stride_ = static_cast<int>(declare_parameter("obstacle_stride", 3));
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      declare_parameter("path_topic", std::string("/arb/global_path")), 10,
      [this](nav_msgs::msg::Path::ConstSharedPtr m) {path_ = m;});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter("scan_topic", std::string("/arb/scan")),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {scan_ = m;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/arb/odom")),
      rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        current_v_ = m->twist.twist.linear.x;
        current_w_ = m->twist.twist.angular.z;
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      declare_parameter("cmd_topic", std::string("/cmd_vel")), 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_), [this]() {step();});
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Local planner (DWA): %d x %d velocity samples, %.1f s horizon, "
      "weights heading %.2f clearance %.2f velocity %.2f path %.2f, "
      "carrot %.2f m, %.0f Hz",
      cfg.linear_samples, cfg.angular_samples, cfg.simulation_time,
      cfg.heading_weight, cfg.clearance_weight, cfg.velocity_weight,
      cfg.path_weight, cfg.path_lookahead_m, control_rate_);
  }

private:
  void publish(double v, double w)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    cmd_pub_->publish(t);
    last_v_ = v;
    last_w_ = w;
  }

  void step()
  {
    const double dt = 1.0 / control_rate_;
    if (!path_ || path_->poses.empty()) {
      // No path means no goal, not "drive anyway".
      publish(0.0, 0.0);
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
    // map -> base_link, used to bring the path into the robot frame. DWA works
    // entirely in the robot frame because that is where the velocity model and
    // the LiDAR both live.
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const arb_core::Pose2D map_to_base(
      tf.transform.translation.x, tf.transform.translation.y, yaw);

    std::vector<arb_core::Pose2D> local_path;
    local_path.reserve(path_->poses.size());
    for (const auto & p : path_->poses) {
      local_path.push_back(
        map_to_base.compose(arb_core::Pose2D(
          p.pose.position.x, p.pose.position.y, 0.0)));
    }
    const arb_core::Pose2D goal = local_path.back();

    if (std::hypot(goal.x, goal.y) < goal_tolerance_) {
      publish(0.0, 0.0);
      if (!reached_) {
        RCLCPP_INFO(get_logger(), "local goal reached");
        reached_ = true;
      }
      return;
    }
    reached_ = false;

    // Obstacles straight from the scan, in the robot frame. Not from the map:
    // the map is two seconds old at best and cannot contain anything that has
    // just walked in front of the robot, which is the entire point of having a
    // local planner.
    std::vector<ObstaclePoint> obstacles;
    if (scan_) {
      obstacles.reserve(scan_->ranges.size() / obstacle_stride_ + 1);
      for (std::size_t i = 0; i < scan_->ranges.size();
           i += static_cast<std::size_t>(obstacle_stride_))
      {
        const float r = scan_->ranges[i];
        if (!std::isfinite(r) || r < scan_->range_min || r > obstacle_range_) {
          continue;
        }
        const double a = scan_->angle_min +
          static_cast<double>(i) * scan_->angle_increment;
        // The scan is in lidar_link, which is 0.14 m ahead of base_link.
        obstacles.push_back({lidar_offset_x_ + r * std::cos(a), r * std::sin(a)});
      }
    }

    const DwaResult r = planner_->plan(
      current_v_, current_w_, goal, local_path, obstacles, dt);
    ++cycles_;
    if (!r.success) {++failed_cycles_;}
    last_result_ = r;
    publish(r.v, r.omega);
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(),
      "DWA: %lu cycles (%lu with no admissible command) | last: v=%.3f w=%.3f, "
      "%d/%d candidates admissible, %.2f ms | no tf %lu",
      cycles_, failed_cycles_, last_v_, last_w_,
      last_result_.candidates_admissible, last_result_.candidates_evaluated,
      last_result_.planning_time_ms, no_tf_);
  }

  std::unique_ptr<DynamicWindowPlanner> planner_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  nav_msgs::msg::Path::ConstSharedPtr path_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  double current_v_{0.0}, current_w_{0.0};
  double last_v_{0.0}, last_w_{0.0};
  double control_rate_{20.0}, goal_tolerance_{0.20};
  double obstacle_range_{3.0};
  double lidar_offset_x_{0.14};
  int obstacle_stride_{3};
  std::string map_frame_{"map"}, base_frame_{"base_link"};
  bool reached_{false};
  DwaResult last_result_;
  unsigned long cycles_{0}, failed_cycles_{0}, no_tf_{0};
};

}  // namespace arb_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_planning::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
