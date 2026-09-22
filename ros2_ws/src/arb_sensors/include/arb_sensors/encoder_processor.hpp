// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_SENSORS__ENCODER_PROCESSOR_HPP_
#define ARB_SENSORS__ENCODER_PROCESSOR_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <sensor_msgs/msg/joint_state.hpp>

#include "arb_sensors/noise_model.hpp"

namespace arb_sensors
{

struct EncoderConfig
{
  double rate_hz{50.0};

  std::string left_joint{"left_wheel_joint"};
  std::string right_joint{"right_wheel_joint"};

  /// Optical/magnetic encoder resolution. With quadrature decoding the usable
  /// resolution is 4x the line count.
  int counts_per_revolution{1024};
  int quadrature_multiplier{4};

  /// Extra white noise on the reported wheel rate, on top of quantisation.
  double velocity_noise_sigma{0.01};

  /// Multiplicative slip: the fraction of wheel rotation that does not become
  /// ground motion. Mean plus per-sample spread, applied independently per
  /// wheel because ASYMMETRIC slip is what silently curves a straight run.
  double slip_mean{0.0};
  double slip_sigma{0.01};

  bool enforce_monotonic_time{true};
  double max_dt_s{0.5};

  std::uint32_t seed{1u};
};

struct EncoderFaultState
{
  double left_bias_rad_s{0.0};   ///< additive bias on the left wheel rate
  double right_bias_rad_s{0.0};  ///< additive bias on the right wheel rate
  double noise_scale{1.0};
  bool outage{false};
};

enum class EncoderReject
{
  kNone,
  kOutage,
  kMissingJoint,
  kNonFinite,
  kNonMonotonicTime,
  kFirstSample,
};

struct EncoderResult
{
  std::optional<sensor_msgs::msg::JointState> message;
  EncoderReject reason{EncoderReject::kNone};

  /// Convenience copies of what was published, so the node can log without
  /// re-parsing the message it just built.
  double left_position_rad{0.0};
  double right_position_rad{0.0};
  double left_velocity_rad_s{0.0};
  double right_velocity_rad_s{0.0};
  double dt_s{0.0};
};

/// Turns Gazebo's exact joint states into something an encoder could actually
/// have produced.
///
/// WHAT A REAL ENCODER GIVES YOU, AND WHAT IT DOES NOT
/// ---------------------------------------------------
/// Gazebo hands us a perfect continuous joint angle and a perfect joint
/// velocity. A quadrature encoder gives neither. It gives an integer COUNT.
/// Everything else is derived:
///
///   * Position is quantised: the finest resolvable rotation is
///     2*pi / (counts_per_rev * quadrature) rad. At 1024 x 4 that is 1.53 mrad,
///     which at a 60 mm wheel is 0.09 mm of arc.
///
///   * Velocity is NOT measured. It is computed by differencing counts over a
///     sample interval, so its quantisation error is
///     resolution/dt -- at 50 Hz that is 0.077 rad/s, which is 11 % of a
///     0.7 rad/s wheel rate. Encoder velocity is therefore noisy at LOW speed
///     and clean at high speed, the opposite of most people's intuition, and
///     it is why Phase 3 integrates positions rather than trusting velocities.
///
/// Reproducing that here, instead of passing Gazebo's exact values through,
/// is what makes the Phase 3 odometry and the Phase 4 filter face a realistic
/// problem.
class EncoderProcessor
{
public:
  explicit EncoderProcessor(const EncoderConfig & config);

  EncoderResult process(const sensor_msgs::msg::JointState & raw, double stamp_s);

  void set_fault(const EncoderFaultState & fault);
  const EncoderFaultState & fault() const {return fault_;}

  /// Smallest resolvable wheel rotation, in radians.
  double resolution_rad() const {return resolution_rad_;}

  /// 1-sigma uncertainty of a velocity sample at the given interval, combining
  /// quantisation and additive noise. Published as the measurement covariance
  /// so the EKF does not have to guess.
  double velocity_sigma(double dt_s) const;

  void reset();

private:
  /// Round a wheel angle onto the encoder's count grid.
  double quantise(double angle_rad) const;

  EncoderConfig config_;
  EncoderFaultState fault_;
  RandomSource rng_;

  double resolution_rad_{0.0};

  bool have_previous_{false};
  double previous_stamp_s_{0.0};
  double previous_left_quantised_{0.0};
  double previous_right_quantised_{0.0};

  /// Continuous accumulator of the corrupted wheel angle. Slip, injected drift
  /// and additive noise are all fractions of a count, so they are accumulated
  /// here at full precision and only the PUBLISHED value is rounded. Rounding
  /// each increment instead would discard every sub-count contribution, and a
  /// slow drift of 0.05 rad/s would never accumulate at all.
  double left_accumulator_{0.0};
  double right_accumulator_{0.0};

  /// Last PUBLISHED position: the accumulator rounded onto the count grid.
  /// A quadrature encoder reports an integer count and nothing else, so this
  /// is what velocity is differenced from and what consumers integrate.
  double left_output_{0.0};
  double right_output_{0.0};
};

}  // namespace arb_sensors

#endif  // ARB_SENSORS__ENCODER_PROCESSOR_HPP_
