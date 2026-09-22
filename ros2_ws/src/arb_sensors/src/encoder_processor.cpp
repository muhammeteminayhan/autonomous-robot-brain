// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_sensors/encoder_processor.hpp"

#include <algorithm>
#include <cmath>

#include "arb_core/angles.hpp"

namespace arb_sensors
{
namespace
{

/// Index of `name` in the JointState, or -1.
int index_of(const sensor_msgs::msg::JointState & js, const std::string & name)
{
  for (std::size_t i = 0; i < js.name.size(); ++i) {
    if (js.name[i] == name) {return static_cast<int>(i);}
  }
  return -1;
}

}  // namespace

EncoderProcessor::EncoderProcessor(const EncoderConfig & config)
: config_(config), rng_(config.seed, "encoder")
{
  const double ticks =
    static_cast<double>(config_.counts_per_revolution) *
    static_cast<double>(config_.quadrature_multiplier);
  resolution_rad_ = (ticks > 0.0) ? arb_core::two_pi<double>() / ticks : 0.0;
}

void EncoderProcessor::set_fault(const EncoderFaultState & fault) {fault_ = fault;}

double EncoderProcessor::quantise(double angle_rad) const
{
  if (resolution_rad_ <= 0.0) {return angle_rad;}
  return std::round(angle_rad / resolution_rad_) * resolution_rad_;
}

double EncoderProcessor::velocity_sigma(double dt_s) const
{
  // Two independent contributions:
  //   quantisation of a difference of two rounded positions, whose error is
  //   uniform on +-resolution/2 each, giving a standard deviation of
  //   resolution / sqrt(12) per sample and sqrt(2) of that for the difference;
  //   plus the configured additive noise.
  const double quant = (dt_s > 0.0)
    ? resolution_rad_ * std::sqrt(2.0 / 12.0) / dt_s
    : 0.0;
  const double additive = config_.velocity_noise_sigma * fault_.noise_scale;
  return std::sqrt(quant * quant + additive * additive);
}

void EncoderProcessor::reset()
{
  rng_.reseed(config_.seed, "encoder");
  have_previous_ = false;
  left_accumulator_ = 0.0;
  right_accumulator_ = 0.0;
  left_output_ = 0.0;
  right_output_ = 0.0;
  fault_ = EncoderFaultState{};
}

EncoderResult EncoderProcessor::process(
  const sensor_msgs::msg::JointState & raw, double stamp_s)
{
  EncoderResult result;

  if (fault_.outage) {
    result.reason = EncoderReject::kOutage;
    return result;
  }

  const int li = index_of(raw, config_.left_joint);
  const int ri = index_of(raw, config_.right_joint);
  if (li < 0 || ri < 0 ||
      raw.position.size() <= static_cast<std::size_t>(std::max(li, ri)))
  {
    result.reason = EncoderReject::kMissingJoint;
    return result;
  }

  const double left_truth = raw.position[li];
  const double right_truth = raw.position[ri];
  if (!std::isfinite(left_truth) || !std::isfinite(right_truth) ||
      !std::isfinite(stamp_s))
  {
    result.reason = EncoderReject::kNonFinite;
    return result;
  }

  const double left_q = quantise(left_truth);
  const double right_q = quantise(right_truth);

  if (!have_previous_) {
    have_previous_ = true;
    previous_stamp_s_ = stamp_s;
    previous_left_quantised_ = left_q;
    previous_right_quantised_ = right_q;
    // Reported position starts at zero: an encoder counts from wherever it was
    // switched on, and no consumer may assume an absolute wheel angle.
    left_accumulator_ = 0.0;
    right_accumulator_ = 0.0;
    left_output_ = 0.0;
    right_output_ = 0.0;
    result.reason = EncoderReject::kFirstSample;
    return result;
  }

  const double measured_dt = stamp_s - previous_stamp_s_;
  if (config_.enforce_monotonic_time && measured_dt <= 0.0) {
    result.reason = EncoderReject::kNonMonotonicTime;
    return result;
  }
  const double dt = std::clamp(measured_dt, 1e-6, config_.max_dt_s);

  // Increment measured on the encoder's count grid, which is all a real
  // encoder can report.
  double d_left = left_q - previous_left_quantised_;
  double d_right = right_q - previous_right_quantised_;
  previous_stamp_s_ = stamp_s;
  previous_left_quantised_ = left_q;
  previous_right_quantised_ = right_q;

  // Slip is multiplicative and drawn per wheel: the two wheels do not lose
  // traction together, and the difference is what turns a straight command
  // into a curve.
  const double slip_l = config_.slip_mean +
    rng_.gaussian(0.0, config_.slip_sigma * fault_.noise_scale);
  const double slip_r = config_.slip_mean +
    rng_.gaussian(0.0, config_.slip_sigma * fault_.noise_scale);
  d_left *= (1.0 - slip_l);
  d_right *= (1.0 - slip_r);

  // Injected drift is a RATE bias, so over dt it contributes bias*dt of angle.
  // Adding it to the increment rather than to the velocity keeps position and
  // velocity mutually consistent: a consumer integrating the reported
  // velocities must land on the reported positions.
  d_left += fault_.left_bias_rad_s * dt;
  d_right += fault_.right_bias_rad_s * dt;

  const double noise_sigma = config_.velocity_noise_sigma * fault_.noise_scale;
  d_left += rng_.gaussian(0.0, noise_sigma) * dt;
  d_right += rng_.gaussian(0.0, noise_sigma) * dt;

  left_accumulator_ += d_left;
  right_accumulator_ += d_right;

  // Round the ACCUMULATOR, not the increment. A quadrature encoder can only
  // report whole counts, so the published position must lie on the count grid;
  // but slip, drift and noise are sub-count effects, and rounding them away at
  // every step would delete them entirely. Keeping full precision in the
  // accumulator lets a 0.05 rad/s drift cross a count boundary every 30 ms and
  // appear in the output, exactly as it would on real hardware.
  const double left_out = quantise(left_accumulator_);
  const double right_out = quantise(right_accumulator_);

  // Velocity is differenced from the PUBLISHED positions, so a consumer that
  // integrates the reported velocities lands exactly on the reported
  // positions. Reporting an unquantised velocity beside a quantised position
  // would make the two mutually inconsistent.
  const double v_left = (left_out - left_output_) / dt;
  const double v_right = (right_out - right_output_) / dt;
  left_output_ = left_out;
  right_output_ = right_out;

  sensor_msgs::msg::JointState out;
  out.header = raw.header;
  out.name = {config_.left_joint, config_.right_joint};
  out.position = {left_out, right_out};
  out.velocity = {v_left, v_right};
  // effort carries the per-sample velocity sigma. JointState has no covariance
  // field, and inventing a custom message for one number would force every
  // consumer to depend on it; the field is documented here and in the node.
  const double sigma = velocity_sigma(dt);
  out.effort = {sigma, sigma};

  result.message = std::move(out);
  result.left_position_rad = left_out;
  result.right_position_rad = right_out;
  result.left_velocity_rad_s = v_left;
  result.right_velocity_rad_s = v_right;
  result.dt_s = dt;
  return result;
}

}  // namespace arb_sensors
