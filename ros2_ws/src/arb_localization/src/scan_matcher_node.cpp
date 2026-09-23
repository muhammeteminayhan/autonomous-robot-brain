// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Matches each LiDAR scan against the occupancy grid the robot has built, and
// publishes the corrected pose for the EKF to fuse as an ABSOLUTE measurement.
//
// WHY THIS CLOSES THE LOOP
// ========================
// Everything before this point measures CHANGE. Wheel encoders measure wheel
// rotation, the gyroscope measures rotation rate; neither says where the robot
// is, only how it moved. Any estimator built from them alone has an error that
// can only grow.
//
// Phase 6 measured what that costs: after 82 m of exploration the map was
// structurally correct and rotated by several degrees, because the heading
// error had nowhere to go. Scan matching gives the filter its first and only
// statement about absolute position, and bounds the error instead of its rate.
//
// WHY IT IS ALSO SELF-REFERENTIAL
// The map was built from the estimate this node is correcting, so a perfect
// match proves the scan agrees with the robot's own beliefs. That is why the
// reported covariance has a floor: the evidence is real but not independent,
// and a filter told otherwise would drive its uncertainty to zero on its own
// opinion. Breaking the circle properly needs loop closure and pose-graph
// optimisation, which is out of scope here and named in the limitations.
#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"
#include "arb_localization/scan_matcher.hpp"

namespace arb_localization
{

class ScanMatcherNode : public rclcpp::Node
{
public:
  ScanMatcherNode()
  : rclcpp::Node("scan_matcher")
  {
    MatcherConfig cfg;
    cfg.coarse_linear_window_m = declare_parameter("coarse.linear_window_m", 0.30);
    cfg.coarse_linear_step_m = declare_parameter("coarse.linear_step_m", 0.05);
    cfg.coarse_angular_window_rad =
      declare_parameter("coarse.angular_window_rad", 0.12);
    cfg.coarse_angular_step_rad = declare_parameter("coarse.angular_step_rad", 0.02);
    cfg.fine_linear_window_m = declare_parameter("fine.linear_window_m", 0.06);
    cfg.fine_linear_step_m = declare_parameter("fine.linear_step_m", 0.01);
    cfg.fine_angular_window_rad = declare_parameter("fine.angular_window_rad", 0.025);
    cfg.fine_angular_step_rad = declare_parameter("fine.angular_step_rad", 0.004);
    cfg.point_stride = static_cast<int>(declare_parameter("point_stride", 2));
    cfg.min_points = static_cast<int>(declare_parameter("min_points", 60));
    cfg.min_score = declare_parameter("min_score", 0.35);
    cfg.min_improvement = declare_parameter("min_improvement", -0.02);
    cfg.position_sigma_m = declare_parameter("position_sigma_m", 0.05);
    cfg.heading_sigma_rad = declare_parameter("heading_sigma_rad", 0.02);
    matcher_ = std::make_unique<ScanMatcher>(cfg);
    stride_ = std::max(1, cfg.point_stride);

    field_sigma_ = declare_parameter("field_sigma_m", 0.15);
    field_max_distance_ = declare_parameter("field_max_distance_m", 0.75);
    occupied_threshold_ = static_cast<int>(declare_parameter("occupied_threshold", 65));
    min_obstacles_ = static_cast<int>(declare_parameter("min_map_obstacles", 400));
    match_period_ = declare_parameter("match_period_s", 0.4);
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    lidar_frame_ = declare_parameter("lidar_frame", std::string("lidar_link"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    tf_timeout_ = declare_parameter("tf_timeout_s", 0.15);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/arb/map")), map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {onMap(m);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter("scan_topic", std::string("/arb/scan")),
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter("output_topic", std::string("/arb/scan_match_pose")),
      rclcpp::SensorDataQoS());

    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Scan matcher: coarse +-%.2f m / +-%.1f deg, fine step %.3f m / %.2f deg, "
      "every %.2f s, stride %d, min score %.2f",
      cfg.coarse_linear_window_m,
      arb_core::radToDeg(cfg.coarse_angular_window_rad),
      cfg.fine_linear_step_m, arb_core::radToDeg(cfg.fine_angular_step_rad),
      match_period_, stride_, cfg.min_score);
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    // Rebuilding the likelihood field is the expensive part, so it is done on
    // map arrival (2 Hz) rather than per scan (10 Hz).
    auto field = std::make_shared<LikelihoodField>(
      msg->data, static_cast<int>(msg->info.width),
      static_cast<int>(msg->info.height), msg->info.resolution,
      msg->info.origin.position.x, msg->info.origin.position.y,
      occupied_threshold_, field_sigma_, field_max_distance_);
    field_ = field;
    ++maps_received_;
  }

  std::optional<arb_core::Pose2D> lookup(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return arb_core::Pose2D(tf.transform.translation.x,
                            tf.transform.translation.y, yaw);
  }

  void onScan(const sensor_msgs::msg::LaserScan & msg)
  {
    ++scans_received_;
    const auto field = field_;
    if (!field || field->obstacleCount() < min_obstacles_) {
      // Matching against a nearly empty map is worse than not matching: the
      // few obstacles it does contain form a weak, ambiguous constraint and
      // the best-scoring pose can be anywhere.
      ++skipped_no_map_;
      return;
    }

    const rclcpp::Time stamp(msg.header.stamp);
    const double now = stamp.seconds();
    if (last_match_time_ > 0.0 && now - last_match_time_ < match_period_) {
      return;
    }

    const auto sensor = lookup(map_frame_, lidar_frame_, stamp);
    const auto base = lookup(map_frame_, base_frame_, stamp);
    if (!sensor || !base) {++skipped_no_tf_; return;}

    // Points in the SENSOR frame. Subsampled: neighbouring beams on the same
    // wall are nearly redundant and the cost is linear in the point count.
    std::vector<ScanPoint> points;
    points.reserve(msg.ranges.size() / stride_ + 1);
    for (std::size_t i = 0; i < msg.ranges.size(); i += stride_) {
      const float r = msg.ranges[i];
      if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) {continue;}
      const double a = msg.angle_min + static_cast<double>(i) * msg.angle_increment;
      points.push_back({r * std::cos(a), r * std::sin(a)});
    }

    const MatchResult result = matcher_->match(*field, *sensor, points);
    last_match_time_ = now;
    ++matches_attempted_;
    last_score_ = result.score;
    if (!result.valid) {++matches_rejected_; return;}
    ++matches_accepted_;

    // The matcher corrected the SENSOR pose; the filter's state is base_link.
    // Applying the same correction means composing the sensor's correction
    // with the fixed sensor-to-base transform, which is exactly
    //   T_map_base_new = T_map_sensor_new * T_sensor_base
    const arb_core::Pose2D sensor_to_base = sensor->inverse().compose(*base);
    const arb_core::Pose2D corrected = result.pose.compose(sensor_to_base);
    last_correction_ = std::hypot(corrected.x - base->x, corrected.y - base->y);
    last_heading_correction_ = arb_core::angleDiff(corrected.theta, base->theta);

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = corrected.x;
    out.pose.pose.position.y = corrected.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, corrected.theta);
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    out.pose.covariance.fill(0.0);
    out.pose.covariance[0] = result.covariance(0, 0);
    out.pose.covariance[7] = result.covariance(1, 1);
    out.pose.covariance[35] = result.covariance(2, 2);
    out.pose.covariance[14] = 1e6;
    out.pose.covariance[21] = 1e6;
    out.pose.covariance[28] = 1e6;
    pose_pub_->publish(out);
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(),
      "scan match: %lu accepted / %lu attempted (%lu rejected) | "
      "last score %.3f, correction %.3f m %.2f deg | scans %lu, maps %lu "
      "(skipped: %lu no map, %lu no tf)",
      matches_accepted_, matches_attempted_, matches_rejected_, last_score_,
      last_correction_, arb_core::radToDeg(last_heading_correction_),
      scans_received_, maps_received_, skipped_no_map_, skipped_no_tf_);
  }

  std::unique_ptr<ScanMatcher> matcher_;
  std::shared_ptr<const LikelihoodField> field_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  int stride_{2};
  double field_sigma_{0.15};
  double field_max_distance_{0.75};
  int occupied_threshold_{65};
  int min_obstacles_{400};
  double match_period_{0.4};
  double tf_timeout_{0.15};
  std::string map_frame_{"map"};
  std::string lidar_frame_{"lidar_link"};
  std::string base_frame_{"base_link"};

  double last_match_time_{0.0};
  double last_score_{0.0};
  double last_correction_{0.0};
  double last_heading_correction_{0.0};
  unsigned long scans_received_{0}, maps_received_{0};
  unsigned long matches_attempted_{0}, matches_accepted_{0}, matches_rejected_{0};
  unsigned long skipped_no_map_{0}, skipped_no_tf_{0};
};

}  // namespace arb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_localization::ScanMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
