// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Fuses wheel odometry and the IMU gyroscope into a single pose estimate, and
// publishes the map -> odom correction.
//
// WHY THE CORRECTION GOES IN map -> odom
// ======================================
// The filter's own answer is the pose of base_link in map. But the TF tree
// already contains odom -> base_link, published by the odometry node, and a
// frame may have only one parent. So the EKF publishes
//
//     T_map_odom = T_map_base * T_odom_base^-1
//
// which leaves odom -> base_link smooth and continuous for the controller,
// while map -> base_link (the composition) is globally consistent for the
// planner. Two publishers of the same transform produce a tree that flickers
// between them and is worse than either alone.
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <arb_msgs/msg/ekf_diagnostics.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"
#include "arb_localization/ekf.hpp"

namespace arb_localization
{

using EKF = ExtendedKalmanFilter;

/// One buffered measurement, tagged with the instant it describes.
///
/// WHY A BUFFER IS NOT OPTIONAL
/// ============================
/// The IMU arrives at 200 Hz and the wheel odometry at 50 Hz. Whichever
/// callback fires advances the filter clock to its own timestamp, so at any
/// moment the newest IMU stamp is up to 20 ms AHEAD of the newest odometry
/// stamp. Fusing each measurement the instant it arrives therefore means every
/// odometry message is already in the filter's past and gets discarded as
/// out of order.
///
/// Measured before this buffer existed: 1917 IMU updates and 3 odometry
/// updates in ten seconds, with 485 rejections. The filter was running on the
/// gyroscope alone, which makes the gyro bias unobservable -- the exact state
/// the six-state design exists to estimate. Nothing errored; the numbers were
/// simply wrong.
///
/// The fix is the standard one: hold measurements briefly, then process them
/// in TIMESTAMP order once everything from that instant has had time to
/// arrive. The cost is a fixed lag equal to the buffer window; the benefit is
/// that every sensor is fused against the state at the moment it measured.
struct BufferedMeasurement
{
  enum class Type { kWheelTwist, kGyro };

  double stamp{0.0};
  Type type{Type::kWheelTwist};
  builtin_interfaces::msg::Time ros_stamp;

  // wheel twist
  double v{0.0};
  double omega{0.0};
  double sigma_v{0.0};
  double sigma_omega{0.0};

  // gyro
  double yaw_rate{0.0};
  double sigma{0.0};
};

class EkfNode : public rclcpp::Node
{
public:
  EkfNode()
  : rclcpp::Node("ekf")
  {
    EKF::Config cfg;
    cfg.accel_sigma = declare_parameter("process.accel_sigma", 0.8);
    cfg.angular_accel_sigma = declare_parameter("process.angular_accel_sigma", 2.0);
    cfg.gyro_bias_random_walk =
      declare_parameter("process.gyro_bias_random_walk", 2.0e-5);
    cfg.position_process_sigma =
      declare_parameter("process.position_sigma", 0.002);
    cfg.heading_process_sigma = declare_parameter("process.heading_sigma", 0.002);
    cfg.enable_gating = declare_parameter("gating.enabled", true);
    cfg.nis_gate_1dof = declare_parameter("gating.nis_1dof", 6.63);
    cfg.nis_gate_2dof = declare_parameter("gating.nis_2dof", 9.21);
    cfg.nis_gate_3dof = declare_parameter("gating.nis_3dof", 11.34);
    cfg.max_consecutive_rejections =
      static_cast<int>(declare_parameter("gating.max_consecutive_rejections", 25));
    cfg.max_predict_dt = declare_parameter("max_predict_dt", 0.5);

    initial_bias_sigma_ = declare_parameter("initial.gyro_bias_sigma", 0.01);
    initial_pose_sigma_ = declare_parameter("initial.pose_sigma", 0.05);
    initial_twist_sigma_ = declare_parameter("initial.twist_sigma", 0.10);

    // Floor on the measurement sigma taken from a message. A sensor that
    // reports zero uncertainty would be given infinite weight and would
    // overwrite the state outright, which is exactly what a misconfigured
    // upstream node does.
    min_sigma_ = declare_parameter("min_measurement_sigma", 1.0e-4);

    // How long to hold a measurement before fusing it. Must exceed the spread
    // in arrival times across sensors -- here the 20 ms odometry period -- or
    // the slower sensor is still late. Too long and the published estimate
    // lags reality by the same amount.
    buffer_delay_ = declare_parameter("measurement_buffer_delay_s", 0.06);
    max_buffer_size_ =
      static_cast<std::size_t>(declare_parameter("measurement_buffer_max", 2000));

    map_frame_ = declare_parameter("map_frame", std::string("map"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    publish_rate_hz_ = declare_parameter("publish_rate_hz", 50.0);
    use_imu_ = declare_parameter("use_imu", true);

    filter_ = std::make_unique<EKF>(cfg);

    const auto qos = rclcpp::SensorDataQoS();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      declare_parameter("odom_topic", std::string("/arb/odom")), qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdom(*m);});
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter("imu_topic", std::string("/arb/imu")), qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {onImu(*m);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      declare_parameter("pose_topic", std::string("/arb/pose")), qos);
    diag_pub_ = create_publisher<arb_msgs::msg::EkfDiagnostics>(
      declare_parameter("diagnostics_topic", std::string("/ekf/diagnostics")),
      rclcpp::QoS(50));

    if (publish_tf_) {
      broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {publish();});
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "EKF: state [x y theta v omega b_gyro], fusing wheel odometry%s | "
      "gating %s | publishing %s -> %s at %.0f Hz",
      use_imu_ ? " + IMU gyro" : " ONLY (IMU disabled)",
      cfg.enable_gating ? "on" : "off",
      map_frame_.c_str(), odom_frame_.c_str(), publish_rate_hz_);
  }

private:
  static double yawOf(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  /// Advance the filter to `stamp`, then let the caller apply a measurement.
  ///
  /// Predicting to the MEASUREMENT's timestamp rather than to now is what
  /// keeps a 200 Hz IMU and a 50 Hz encoder stream consistent: each
  /// measurement is fused against the state at the instant it was taken, not
  /// against whatever the state happened to be when the callback ran.
  bool advanceTo(double stamp)
  {
    if (!initialised_) {return false;}

    const double dt = stamp - filter_time_;

    // A measurement AT the current filter time needs no prediction, only an
    // update, and must NOT be discarded.
    //
    // This is not a corner case here, it is the common case. Gazebo publishes
    // on a 1 ms physics grid, so the 200 Hz IMU and the 50 Hz encoders share a
    // timestamp exactly every fourth sample. Treating "not strictly newer" as
    // out of order therefore threw away every odometry measurement that
    // coincided with an IMU sample -- which, at these rates, was all of them.
    //
    // Measured with the strict comparison: 1962 IMU updates against 3 odometry
    // updates in ten seconds. The filter was running on the gyroscope alone,
    // leaving the gyro bias unobservable, and it reported no error at all.
    if (dt == 0.0) {return true;}

    if (dt < 0.0) {
      // Genuinely stale. The buffer drains in timestamp order, so this means
      // something inserted a measurement from before the filter's clock.
      ++out_of_order_;
      return false;
    }

    if (!filter_->predict(dt)) {
      // The gap was too large to integrate in one step. Resynchronise the
      // clock rather than repeatedly failing; the covariance is already wider
      // than it should be, which is the honest outcome.
      ++predict_rejected_;
      filter_time_ = stamp;
      return false;
    }
    filter_time_ = stamp;
    return true;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    last_odom_pose_ = arb_core::Pose2D(
      msg.pose.pose.position.x, msg.pose.pose.position.y,
      yawOf(msg.pose.pose.orientation));
    have_odom_pose_ = true;

    if (!initialised_) {
      // Initialise FROM the first odometry message rather than from zero.
      //
      // A filter that starts far from the truth and is protected by a
      // chi-square gate can lock itself into a wrong state: every correct
      // measurement looks like an outlier and is rejected. Starting at the
      // first measurement makes the gate a guard rather than a trap.
      EKF::StateVector s = EKF::StateVector::Zero();
      s(EKF::X) = last_odom_pose_.x;
      s(EKF::Y) = last_odom_pose_.y;
      s(EKF::THETA) = last_odom_pose_.theta;
      s(EKF::V) = msg.twist.twist.linear.x;
      s(EKF::OMEGA) = msg.twist.twist.angular.z;

      EKF::StateMatrix P = EKF::StateMatrix::Zero();
      P(EKF::X, EKF::X) = initial_pose_sigma_ * initial_pose_sigma_;
      P(EKF::Y, EKF::Y) = P(EKF::X, EKF::X);
      P(EKF::THETA, EKF::THETA) = P(EKF::X, EKF::X);
      P(EKF::V, EKF::V) = initial_twist_sigma_ * initial_twist_sigma_;
      P(EKF::OMEGA, EKF::OMEGA) = P(EKF::V, EKF::V);
      P(EKF::GYRO_BIAS, EKF::GYRO_BIAS) = initial_bias_sigma_ * initial_bias_sigma_;

      filter_->initialise(s, P);
      filter_time_ = stamp;
      initialised_ = true;
      RCLCPP_INFO(
        get_logger(), "EKF initialised at (%.3f, %.3f, %.1f deg) from odometry",
        s(EKF::X), s(EKF::Y), s(EKF::THETA) * 180.0 / M_PI);
      return;
    }

    ++odom_received_;
    BufferedMeasurement m;
    m.stamp = stamp;
    m.type = BufferedMeasurement::Type::kWheelTwist;
    m.ros_stamp = msg.header.stamp;
    m.v = msg.twist.twist.linear.x;
    m.omega = msg.twist.twist.angular.z;
    m.sigma_v = std::max(min_sigma_, std::sqrt(
        std::max(0.0, msg.twist.covariance[0])));
    m.sigma_omega = std::max(min_sigma_, std::sqrt(
        std::max(0.0, msg.twist.covariance[35])));
    enqueue(m);
  }

  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    if (!use_imu_ || !initialised_) {return;}
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const double rate = msg.angular_velocity.z;
    if (!std::isfinite(rate)) {++imu_rejected_; return;}
    ++imu_received_;
    BufferedMeasurement m;
    m.stamp = stamp;
    m.type = BufferedMeasurement::Type::kGyro;
    m.ros_stamp = msg.header.stamp;
    m.yaw_rate = rate;
    // The covariance comes from the message, which arb_sensors fills from the
    // same noise model that generated the data -- including any runtime fault
    // scaling. That is the mechanism by which a degraded sensor is DOWN-WEIGHTED
    // rather than blindly trusted.
    m.sigma = std::max(min_sigma_, std::sqrt(
        std::max(0.0, msg.angular_velocity_covariance[8])));
    enqueue(m);
  }

  /// Insert a measurement into the time-ordered buffer and drain whatever has
  /// become safe to fuse.
  void enqueue(const BufferedMeasurement & m)
  {
    newest_stamp_ = std::max(newest_stamp_, m.stamp);

    if (initialised_ && m.stamp < filter_time_) {
      // STRICTLY older: the filter has already moved past this instant. Equal
      // timestamps are fine and common -- see advanceTo.
      //
      // With an adequate buffer delay this should be rare, and a rising count
      // means the delay is too short for the spread in sensor arrival times.
      ++out_of_order_;
      return;
    }
    buffer_.emplace(m.stamp, m);
    if (buffer_.size() > max_buffer_size_) {
      // A buffer that grows without bound means measurements are arriving and
      // never becoming ripe -- a stalled clock, say. Dropping the oldest keeps
      // memory bounded and the count makes the condition visible.
      buffer_.erase(buffer_.begin());
      ++buffer_overflow_;
    }
    drain();
  }

  /// Fuse every buffered measurement older than the delay window, in
  /// timestamp order.
  void drain()
  {
    if (!initialised_) {return;}
    const double ripe_before = newest_stamp_ - buffer_delay_;
    while (!buffer_.empty() && buffer_.begin()->first <= ripe_before) {
      const BufferedMeasurement m = buffer_.begin()->second;
      buffer_.erase(buffer_.begin());
      apply(m);
    }
  }

  void apply(const BufferedMeasurement & m)
  {
    if (!advanceTo(m.stamp)) {return;}
    EKF::UpdateResult r;
    if (m.type == BufferedMeasurement::Type::kWheelTwist) {
      r = filter_->updateWheelTwist(m.v, m.omega, m.sigma_v, m.sigma_omega);
      ++odom_count_;
      if (!r.accepted) {++odom_rejected_;}
    } else {
      r = filter_->updateGyro(m.yaw_rate, m.sigma);
      ++imu_count_;
      if (!r.accepted) {++imu_rejected_;}
    }
    recordUpdate(r, m.stamp, m.ros_stamp);
  }

  void recordUpdate(const EKF::UpdateResult & r, double stamp,
                    const builtin_interfaces::msg::Time & msg_stamp)
  {
    last_update_ = r;
    last_update_stamp_ = msg_stamp;
    const double now = now_seconds();
    last_measurement_age_ms_ = (now > 0.0) ? (now - stamp) * 1e3 : 0.0;
  }

  double now_seconds()
  {
    const auto t = get_clock()->now();
    return (t.nanoseconds() == 0) ? 0.0 : t.seconds();
  }

  void publish()
  {
    if (!initialised_) {return;}
    // Drain here too: if a sensor stops publishing, the remaining buffered
    // measurements must still be fused rather than waiting for an arrival that
    // never comes.
    drain();
    const auto pose = filter_->pose();
    const auto P = filter_->covariance();

    builtin_interfaces::msg::Time stamp = last_update_stamp_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.theta);

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;
    out.pose.pose.position.x = pose.x;
    out.pose.pose.position.y = pose.y;
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.pose.orientation.w = q.w();
    out.pose.covariance.fill(0.0);
    out.pose.covariance[0] = P(EKF::X, EKF::X);
    out.pose.covariance[1] = P(EKF::X, EKF::Y);
    out.pose.covariance[5] = P(EKF::X, EKF::THETA);
    out.pose.covariance[6] = P(EKF::Y, EKF::X);
    out.pose.covariance[7] = P(EKF::Y, EKF::Y);
    out.pose.covariance[11] = P(EKF::Y, EKF::THETA);
    out.pose.covariance[30] = P(EKF::THETA, EKF::X);
    out.pose.covariance[31] = P(EKF::THETA, EKF::Y);
    out.pose.covariance[35] = P(EKF::THETA, EKF::THETA);
    out.pose.covariance[14] = kUnobserved;
    out.pose.covariance[21] = kUnobserved;
    out.pose.covariance[28] = kUnobserved;
    pose_pub_->publish(out);

    // --- diagnostics -----------------------------------------------------
    arb_msgs::msg::EkfDiagnostics diag;
    diag.header.stamp = stamp;
    diag.header.frame_id = map_frame_;
    diag.state.assign(filter_->state().data(),
                      filter_->state().data() + EKF::kStateDim);
    diag.covariance_diagonal.resize(EKF::kStateDim);
    for (int i = 0; i < EKF::kStateDim; ++i) {
      diag.covariance_diagonal[i] = P(i, i);
    }
    diag.position_std = std::sqrt(std::max(0.0, P(EKF::X, EKF::X) + P(EKF::Y, EKF::Y)));
    diag.heading_std = std::sqrt(std::max(0.0, P(EKF::THETA, EKF::THETA)));
    diag.covariance_trace = P.trace();
    diag.last_update_source = last_update_.source;
    diag.innovation.assign(
      last_update_.innovation.data(),
      last_update_.innovation.data() + last_update_.innovation.size());
    diag.normalized_innovation_squared = last_update_.nis;
    diag.update_accepted = last_update_.accepted;
    diag.measurement_age_ms = last_measurement_age_ms_;
    diag_pub_->publish(diag);

    // --- map -> odom -----------------------------------------------------
    if (broadcaster_ && have_odom_pose_) {
      // T_map_odom = T_map_base * T_odom_base^-1
      const arb_core::Pose2D correction = pose.compose(last_odom_pose_.inverse());
      tf2::Quaternion qc;
      qc.setRPY(0.0, 0.0, correction.theta);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = correction.x;
      tf.transform.translation.y = correction.y;
      tf.transform.rotation.x = qc.x();
      tf.transform.rotation.y = qc.y();
      tf.transform.rotation.z = qc.z();
      tf.transform.rotation.w = qc.w();
      broadcaster_->sendTransform(tf);
    }
  }

  void report()
  {
    if (!initialised_) {
      RCLCPP_WARN(get_logger(), "EKF not initialised: no odometry received yet");
      return;
    }
    const auto P = filter_->covariance();
    RCLCPP_INFO(
      get_logger(),
      "EKF: pose=(%.3f, %.3f, %.1f deg) v=%.3f w=%.3f bias=%.5f rad/s | "
      "1-sigma pos=%.3f m heading=%.2f deg | odom %lu/%lu fused (%lu gated) "
      "imu %lu/%lu fused (%lu gated) | buffered %zu out-of-order %lu "
      "predict-rejected %lu overflow %lu",
      filter_->pose().x, filter_->pose().y, filter_->pose().theta * 180.0 / M_PI,
      filter_->linearVelocity(), filter_->angularVelocity(), filter_->gyroBias(),
      std::sqrt(std::max(0.0, P(EKF::X, EKF::X) + P(EKF::Y, EKF::Y))),
      std::sqrt(std::max(0.0, P(EKF::THETA, EKF::THETA))) * 180.0 / M_PI,
      odom_count_, odom_received_, odom_rejected_,
      imu_count_, imu_received_, imu_rejected_,
      buffer_.size(), out_of_order_, predict_rejected_, buffer_overflow_);
  }

  static constexpr double kUnobserved = 1e6;

  std::unique_ptr<EKF> filter_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<arb_msgs::msg::EkfDiagnostics>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  std::string map_frame_, odom_frame_, base_frame_;
  bool publish_tf_{true};
  bool use_imu_{true};
  double publish_rate_hz_{50.0};
  double min_sigma_{1e-4};
  double initial_bias_sigma_{0.01};
  double initial_pose_sigma_{0.05};
  double initial_twist_sigma_{0.10};

  bool initialised_{false};
  double filter_time_{0.0};
  arb_core::Pose2D last_odom_pose_;
  bool have_odom_pose_{false};
  EKF::UpdateResult last_update_;
  builtin_interfaces::msg::Time last_update_stamp_;
  double last_measurement_age_ms_{0.0};

  std::multimap<double, BufferedMeasurement> buffer_;
  double buffer_delay_{0.06};
  std::size_t max_buffer_size_{2000};
  double newest_stamp_{0.0};
  unsigned long buffer_overflow_{0};

  unsigned long odom_received_{0}, odom_count_{0}, odom_rejected_{0};
  unsigned long imu_received_{0}, imu_count_{0}, imu_rejected_{0};
  unsigned long out_of_order_{0}, predict_rejected_{0};
};

}  // namespace arb_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_localization::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
