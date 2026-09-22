// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_sensors/imu_processor.hpp"

#include <algorithm>
#include <cmath>

namespace arb_sensors
{
namespace
{

bool finite3(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

/// Fill a 3x3 row-major covariance with sigma^2 on the diagonal.
///
/// Gazebo publishes an all-zero covariance, and in ROS 2 an all-zero
/// covariance means "unknown", not "perfect". Leaving it zero forces every
/// downstream consumer to invent its own R matrix, which is how a project ends
/// up with three different opinions about how good its IMU is. Filling it here,
/// from the same noise model that generated the corruption, keeps one answer.
void set_diagonal_covariance(std::array<double, 9> & cov, double sigma)
{
  cov.fill(0.0);
  const double var = sigma * sigma;
  cov[0] = var;
  cov[4] = var;
  cov[8] = var;
}

}  // namespace

ImuProcessor::ImuProcessor(const ImuConfig & config)
: config_(config), rng_(config.seed, "imu")
{
  for (int i = 0; i < 3; ++i) {
    gyro_[i] = ChannelNoise(
      config_.gyro_white_sigma, config_.gyro_bias_sigma,
      config_.gyro_bias_tau_s, config_.gyro_bias_random_walk,
      config_.gyro_scale_error, config_.input_carries_nominal_noise);
    accel_[i] = ChannelNoise(
      config_.accel_white_sigma, config_.accel_bias_sigma,
      config_.accel_bias_tau_s, config_.accel_bias_random_walk,
      config_.accel_scale_error, config_.input_carries_nominal_noise);
  }
  calibrated_ = (config_.calibration_samples <= 0);
}

void ImuProcessor::set_fault(const ImuFaultState & fault)
{
  fault_ = fault;
  for (int i = 0; i < 3; ++i) {
    gyro_[i].set_noise_scale(fault.noise_scale);
    accel_[i].set_noise_scale(fault.noise_scale);
  }
}

double ImuProcessor::gyro_sigma() const {return gyro_[0].effective_sigma();}
double ImuProcessor::accel_sigma() const {return accel_[0].effective_sigma();}

void ImuProcessor::reset()
{
  rng_.reseed(config_.seed, "imu");
  for (int i = 0; i < 3; ++i) {
    gyro_[i].reset();
    accel_[i].reset();
  }
  have_previous_ = false;
  calib_count_ = 0;
  calib_sum_ = {{0.0, 0.0, 0.0}};
  calib_bias_ = {{0.0, 0.0, 0.0}};
  calibrated_ = (config_.calibration_samples <= 0);
  fault_ = ImuFaultState{};
}

ImuResult ImuProcessor::process(const sensor_msgs::msg::Imu & raw, double stamp_s)
{
  ImuResult result;

  if (fault_.outage) {
    // An outage must NOT advance the bias process or the timestamp state.
    // The sensor is gone; when it comes back, the gap is real and the next
    // dt is what tells the filter how much uncertainty accumulated.
    result.reason = ImuReject::kOutage;
    return result;
  }

  const auto & w = raw.angular_velocity;
  const auto & a = raw.linear_acceleration;
  if (!finite3(w.x, w.y, w.z) || !finite3(a.x, a.y, a.z) || !std::isfinite(stamp_s)) {
    result.reason = ImuReject::kNonFinite;
    return result;
  }

  double dt = 1.0 / config_.rate_hz;
  if (have_previous_) {
    const double measured_dt = stamp_s - previous_stamp_s_;
    if (config_.enforce_monotonic_time && measured_dt <= 0.0) {
      // Out-of-order or duplicate sample. Dropping it is the only safe
      // response: integrating a negative dt runs the bias process backwards.
      result.reason = ImuReject::kNonMonotonicTime;
      return result;
    }
    dt = std::clamp(measured_dt, 0.0, config_.max_dt_s);
  }
  previous_stamp_s_ = stamp_s;
  have_previous_ = true;

  const double truth_w[3] = {w.x, w.y, w.z};
  const double truth_a[3] = {a.x, a.y, a.z};
  double out_w[3];
  double out_a[3];
  for (int i = 0; i < 3; ++i) {
    out_w[i] = gyro_[i].apply(truth_w[i], dt, rng_) + fault_.gyro_bias_offset;
    out_a[i] = accel_[i].apply(truth_a[i], dt, rng_);
  }

  // --- startup calibration ------------------------------------------------
  // Average the first N gyro samples while the robot is known to be still and
  // subtract the result. This is exactly what a real robot does on boot, and
  // it removes the turn-on bias -- the largest, most constant part of the
  // error -- without any filter having to estimate it.
  if (!calibrated_) {
    for (int i = 0; i < 3; ++i) {calib_sum_[i] += out_w[i];}
    if (++calib_count_ >= config_.calibration_samples) {
      for (int i = 0; i < 3; ++i) {
        calib_bias_[i] = calib_sum_[i] / static_cast<double>(calib_count_);
      }
      calibrated_ = true;
    }
    result.reason = ImuReject::kCalibrating;
    return result;
  }
  for (int i = 0; i < 3; ++i) {out_w[i] -= calib_bias_[i];}

  sensor_msgs::msg::Imu out = raw;
  out.angular_velocity.x = out_w[0];
  out.angular_velocity.y = out_w[1];
  out.angular_velocity.z = out_w[2];
  out.linear_acceleration.x = out_a[0];
  out.linear_acceleration.y = out_a[1];
  out.linear_acceleration.z = out_a[2];

  set_diagonal_covariance(out.angular_velocity_covariance, gyro_sigma());
  set_diagonal_covariance(out.linear_acceleration_covariance, accel_sigma());
  // This IMU is not an AHRS: it reports rates and accelerations, not a fused
  // orientation. ROS marks that by setting covariance[0] to -1, and a consumer
  // that ignores the flag and uses the identity quaternion gets a robot that
  // believes it is always facing +x.
  out.orientation_covariance.fill(0.0);
  out.orientation_covariance[0] = -1.0;

  result.message = std::move(out);
  return result;
}

}  // namespace arb_sensors
