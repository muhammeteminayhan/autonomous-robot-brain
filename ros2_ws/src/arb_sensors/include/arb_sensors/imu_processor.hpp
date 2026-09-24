// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_SENSORS__IMU_PROCESSOR_HPP_
#define ARB_SENSORS__IMU_PROCESSOR_HPP_

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include <sensor_msgs/msg/imu.hpp>

#include "arb_sensors/noise_model.hpp"

namespace arb_sensors
{

/// Everything the IMU stage can be told to do, all of it from configuration.
struct ImuConfig
{
  double rate_hz{200.0};

  // Nominal specification, converted from datasheet noise density by the node:
  //   sigma_discrete = noise_density * sqrt(rate)
  double gyro_white_sigma{0.005};
  double gyro_bias_sigma{0.0002};
  double gyro_bias_tau_s{300.0};
  double gyro_bias_random_walk{0.00002};
  double gyro_scale_error{0.002};

  double accel_white_sigma{0.05};
  double accel_bias_sigma{0.005};
  double accel_bias_tau_s{300.0};
  double accel_bias_random_walk{0.0004};
  double accel_scale_error{0.003};

  /// True when the incoming stream already contains the datasheet white noise,
  /// as it does when Gazebo models the IMU. The stage then injects only the
  /// EXTRA noise a fault calls for, instead of double-counting the nominal.
  bool input_carries_nominal_noise{true};

  /// Samples averaged at startup to estimate and remove the turn-on bias.
  /// 0 disables calibration. The robot must be stationary for this to be
  /// meaningful, which is why the node refuses to calibrate once it has seen
  /// motion.
  int calibration_samples{0};

  /// Reject a sample whose timestamp is not strictly after the previous one.
  bool enforce_monotonic_time{true};

  /// Largest plausible gap between samples. A longer gap means the stream was
  /// interrupted, and the bias process must not be advanced by that whole
  /// interval in one step as if nothing had happened.
  double max_dt_s{0.5};

  std::uint32_t seed{1u};
};

/// Runtime knobs. These exist so Phase 10 can degrade the sensor without
/// restarting anything, and so the nominal configuration above is never
/// overwritten by a fault.
struct ImuFaultState
{
  /// Multiplies the GYROSCOPE white-noise sigma.
  double noise_scale{1.0};

  /// Multiplies the ACCELEROMETER white-noise sigma, separately.
  ///
  /// Separate because the scenarios distinguish them: Case B inflates both
  /// (severe vibration shakes the whole package) while Case F inflates only the
  /// gyro. One shared knob made `affect_accelerometer: false` a comment rather
  /// than a setting, and a config value that is silently ignored is worse than
  /// no config value.
  ///
  /// It cannot change the pose estimate, and that is the point: the EKF does
  /// not fuse the accelerometer at all (see ekf.hpp -- for a planar robot at
  /// walking pace the signal is buried under gravity leakage from millidegree
  /// tilt). Case B measures that claim instead of asserting it.
  double accel_noise_scale{1.0};

  double gyro_bias_offset{0.0}; ///< rad/s added on top of the modelled bias
  bool outage{false};           ///< drop every sample while true
};

/// Why a sample was not republished. Returned instead of a bare nullopt so the
/// node can count causes and report them, rather than silently swallowing data.
enum class ImuReject
{
  kNone,
  kOutage,
  kNonFinite,
  kNonMonotonicTime,
  kCalibrating,
};

struct ImuResult
{
  std::optional<sensor_msgs::msg::Imu> message;
  ImuReject reason{ImuReject::kNone};
};

/// Stateless-per-message except for the bias processes and the calibration
/// accumulator. Deliberately free of rclcpp so it can be unit tested without
/// spinning a node.
class ImuProcessor
{
public:
  explicit ImuProcessor(const ImuConfig & config);

  /// Process one raw sample. `stamp_s` is the measurement time in seconds,
  /// taken from the message header, never from the wall clock: replaying a
  /// rosbag faster than real time must produce identical output.
  ImuResult process(const sensor_msgs::msg::Imu & raw, double stamp_s);

  void set_fault(const ImuFaultState & fault);
  const ImuFaultState & fault() const {return fault_;}

  /// True once the startup bias estimate is complete (or was not requested).
  bool calibrated() const {return calibrated_;}

  /// The estimated turn-on bias that is being subtracted, for diagnostics.
  const std::array<double, 3> & gyro_calibration_bias() const {return calib_bias_;}

  /// Effective 1-sigma values right now, after any runtime noise scaling.
  double gyro_sigma() const;
  double accel_sigma() const;

  void reset();

private:
  ImuConfig config_;
  ImuFaultState fault_;
  RandomSource rng_;

  ChannelNoise gyro_[3];
  ChannelNoise accel_[3];

  bool have_previous_{false};
  double previous_stamp_s_{0.0};

  bool calibrated_{true};
  int calib_count_{0};
  std::array<double, 3> calib_sum_{{0.0, 0.0, 0.0}};
  std::array<double, 3> calib_bias_{{0.0, 0.0, 0.0}};
};

}  // namespace arb_sensors

#endif  // ARB_SENSORS__IMU_PROCESSOR_HPP_
