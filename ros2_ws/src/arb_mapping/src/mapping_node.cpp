// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Builds a 2D occupancy grid from the preprocessed LiDAR and the EKF pose.
//
// WHICH POSE THE MAP IS BUILT FROM
// ================================
// The scan is placed using the ESTIMATED pose, never ground truth. That makes
// the map self-referential: an error in the estimate smears the map, and a
// smeared map gives worse scan matches, which worsens the estimate. Building
// the map from ground truth would produce a beautiful picture that proves
// nothing about the stack.
#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arb_core/pose2d.hpp"
#include "arb_mapping/occupancy_grid.hpp"

namespace arb_mapping
{

class MappingNode : public rclcpp::Node
{
public:
  MappingNode()
  : rclcpp::Node("mapping")
  {
    GridConfig cfg;
    cfg.resolution_m = declare_parameter("resolution_m", 0.05);
    cfg.width_m = declare_parameter("width_m", 24.0);
    cfg.height_m = declare_parameter("height_m", 18.0);
    cfg.origin_x_m = declare_parameter("origin_x_m", -12.0);
    cfg.origin_y_m = declare_parameter("origin_y_m", -9.0);
    cfg.p_hit = declare_parameter("p_hit", 0.75);
    cfg.p_miss = declare_parameter("p_miss", 0.35);
    cfg.p_prior = declare_parameter("p_prior", 0.5);
    cfg.log_odds_min = declare_parameter("log_odds_min", -2.0);
    cfg.log_odds_max = declare_parameter("log_odds_max", 3.5);
    cfg.occupied_threshold = declare_parameter("occupied_threshold", 0.85);
    cfg.free_threshold = declare_parameter("free_threshold", -0.4);
    cfg.max_usable_range_m = declare_parameter("max_usable_range_m", 10.0);
    cfg.min_usable_range_m = declare_parameter("min_usable_range_m", 0.15);

    map_frame_ = declare_parameter("map_frame", std::string("map"));
    publish_period_ = declare_parameter("publish_period_s", 0.5);

    grid_ = std::make_unique<OccupancyGrid>(cfg);

    const auto qos = rclcpp::SensorDataQoS();
    // The sensor pose comes from TF at the SCAN's timestamp, not from the
    // newest pose message.
    //
    // WHY THAT MATTERS
    // The EKF publishes at 50 Hz and its estimate lags by the measurement
    // buffer, so the newest pose can be 50-80 ms older than the scan. Taking
    // it anyway rotates the whole scan by however far the robot turned in
    // between: at 0.7 rad/s and 0.3 s of tolerance that is 12 degrees, which at
    // 8 m sweeps the beam 1.7 m sideways. The visible symptom is a starburst
    // of free rays radiating out THROUGH walls, because every scan is cast at
    // a slightly wrong angle and their union covers a fan.
    //
    // tf2 interpolates between the transforms bracketing the requested time,
    // which is exactly the problem it exists to solve. It also supplies the
    // map -> lidar_link transform directly, so the sensor mount offset stops
    // being a number this node has to be told and stays where it belongs, in
    // the URDF.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    lidar_frame_ = declare_parameter("lidar_frame", std::string("lidar_link"));
    tf_timeout_ = declare_parameter("tf_timeout_s", 0.15);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      declare_parameter("scan_topic", std::string("/arb/scan")), qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {onScan(*m);});

    // TRANSIENT_LOCAL so a planner that starts later still receives the map.
    // A volatile map means a late subscriber waits a whole publish period
    // seeing nothing, and plans against an empty world if it does not wait.
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      declare_parameter("map_topic", std::string("/arb/map")), map_qos);

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period_), [this]() {publish();});
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Mapping: %d x %d cells at %.3f m (%.1f x %.1f m from %.1f, %.1f) | "
      "log-odds hit %+.3f miss %+.3f, clamped to [%.1f, %.1f]",
      grid_->width(), grid_->height(), cfg.resolution_m, cfg.width_m,
      cfg.height_m, cfg.origin_x_m, cfg.origin_y_m,
      OccupancyGrid::probabilityToLogOdds(cfg.p_hit) -
      OccupancyGrid::probabilityToLogOdds(cfg.p_prior),
      OccupancyGrid::probabilityToLogOdds(cfg.p_miss) -
      OccupancyGrid::probabilityToLogOdds(cfg.p_prior),
      cfg.log_odds_min, cfg.log_odds_max);
  }

private:
  /// Pose of the LiDAR in the map frame at the given time, or nothing.
  std::optional<arb_core::Pose2D> sensorPoseAt(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, lidar_frame_, stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      last_tf_error_ = ex.what();
      return std::nullopt;
    }
    tf2::Quaternion q(
      tf.transform.rotation.x, tf.transform.rotation.y,
      tf.transform.rotation.z, tf.transform.rotation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return arb_core::Pose2D(
      tf.transform.translation.x, tf.transform.translation.y, yaw);
  }

  void onScan(const sensor_msgs::msg::LaserScan & msg)
  {
    ++scans_received_;
    const rclcpp::Time stamp(msg.header.stamp);
    const auto sensor_opt = sensorPoseAt(stamp);
    if (!sensor_opt) {
      ++scans_no_pose_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no %s -> %s transform at the scan time: %s",
        map_frame_.c_str(), lidar_frame_.c_str(), last_tf_error_.c_str());
      return;
    }
    const arb_core::Pose2D sensor = *sensor_opt;

    const std::vector<float> ranges(msg.ranges.begin(), msg.ranges.end());
    const int used = grid_->integrateScan(
      sensor, ranges, msg.angle_min, msg.angle_increment,
      msg.range_min, msg.range_max);
    beams_used_ += static_cast<unsigned long>(used);
    ++scans_integrated_;
  }

  void publish()
  {
    if (scans_integrated_ == 0) {return;}
    const auto & cfg = grid_->config();

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(cfg.resolution_m);
    msg.info.width = static_cast<unsigned int>(grid_->width());
    msg.info.height = static_cast<unsigned int>(grid_->height());
    msg.info.origin.position.x = cfg.origin_x_m;
    msg.info.origin.position.y = cfg.origin_y_m;
    msg.info.origin.orientation.w = 1.0;
    msg.data = grid_->toOccupancyBytes();
    map_pub_->publish(msg);
  }

  void report()
  {
    const auto c = grid_->counts();
    const int total = grid_->width() * grid_->height();
    RCLCPP_INFO(
      get_logger(),
      "map: %d free (%.1f%%), %d occupied (%.1f%%), %d unknown (%.1f%%) | "
      "scans %lu received, %lu integrated (%lu without a transform) | "
      "%lu beams used",
      c.free, 100.0 * c.free / total, c.occupied, 100.0 * c.occupied / total,
      c.unknown, 100.0 * c.unknown / total,
      scans_received_, scans_integrated_, scans_no_pose_, beams_used_);
  }

  std::unique_ptr<OccupancyGrid> grid_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::string map_frame_{"map"};
  std::string lidar_frame_{"lidar_link"};
  double tf_timeout_{0.15};
  double publish_period_{0.5};
  std::string last_tf_error_;

  unsigned long scans_received_{0}, scans_integrated_{0};
  unsigned long scans_no_pose_{0};
  unsigned long beams_used_{0};
};

}  // namespace arb_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_mapping::MappingNode>());
  rclcpp::shutdown();
  return 0;
}
