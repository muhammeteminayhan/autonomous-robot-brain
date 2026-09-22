// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_SENSORS__LIDAR_PROCESSOR_HPP_
#define ARB_SENSORS__LIDAR_PROCESSOR_HPP_

#include <cstdint>
#include <optional>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "arb_sensors/noise_model.hpp"

namespace arb_sensors
{

struct LidarConfig
{
  double rate_hz{10.0};

  /// The scanner's nominal 1-sigma range noise.
  double range_noise_sigma{0.015};

  /// True when the incoming scan already contains that noise, as it does when
  /// Gazebo models the sensor. The stage then injects only the EXTRA noise a
  /// fault calls for. Independent variances add, so reaching a target total of
  /// `noise_scale * sigma` from an input that already carries `sigma` means
  /// injecting sqrt((noise_scale*sigma)^2 - sigma^2), which is zero at
  /// noise_scale = 1. Adding a full sigma unconditionally would make the
  /// nominal scan 41 % noisier than the part being simulated.
  bool input_carries_nominal_noise{true};

  /// Baseline probability that a beam returns nothing, independent of faults.
  double dropout_probability{0.0};

  /// Reject a return that neither of its angular neighbours can belong to the
  /// same surface as. Isolated points are usually edge effects on a corner;
  /// left alone they become permanent phantom cells in the occupancy grid.
  bool despeckle{true};

  /// How oblique a surface we are willing to believe in, measured from the
  /// surface normal. This is what sets the neighbour threshold, and it MUST be
  /// geometric rather than a fixed distance:
  ///
  ///     delta_r = r * tan(phi) * angle_increment
  ///
  /// At 10 m with 1 deg beams, a surface seen at 75 deg obliquity legitimately
  /// steps 0.65 m between neighbouring beams. A fixed 0.25 m threshold calls
  /// every one of those an outlier. Measured with that threshold on this
  /// world: 12 beams per scan discarded, of which 99.9 % were within 5 cm of a
  /// real wall. The filter was deleting data, not noise.
  double despeckle_max_obliquity_deg{85.0};

  /// Floor under the geometric threshold, so that measurement noise alone can
  /// never trigger despeckling at short range. Roughly 3 sigma of the range
  /// noise.
  double despeckle_floor_m{0.05};

  bool enforce_monotonic_time{true};

  std::uint32_t seed{1u};
};

struct LidarFaultState
{
  double dropout_ratio{0.0};   ///< extra fraction of beams discarded
  double noise_scale{1.0};
  bool burst_mode{false};      ///< dropouts arrive in bursts, not uniformly
  bool burst_active{false};    ///< driven by the fault scheduler in Phase 10
  bool outage{false};
};

enum class LidarReject
{
  kNone,
  kOutage,
  kEmptyScan,
  kNonMonotonicTime,
};

struct LidarResult
{
  std::optional<sensor_msgs::msg::LaserScan> message;
  LidarReject reason{LidarReject::kNone};

  int valid_beams{0};
  int dropped_beams{0};
  int despeckled_beams{0};
};

/// Cleans and optionally degrades a 2D scan.
///
/// THE ONE RULE FOR LASER SCANS
/// ----------------------------
/// A LaserScan range is not always a number. ROS defines:
///   * r > range_max (conventionally +inf) -- the beam hit nothing
///   * r < range_min (conventionally -inf) -- something is too close to measure
///   * NaN                                 -- the reading is invalid
///
/// These are NOT interchangeable, and none of them may be treated as a
/// distance. Code that does `if (r < 0.5) obstacle()` reports an obstacle for
/// every -inf beam; code that averages ranges gets NaN for the whole scan. This
/// stage normalises everything unmeasurable to a single sentinel
/// (+infinity) so downstream code has exactly one case to handle, and says so
/// in the message it publishes.
class LidarProcessor
{
public:
  explicit LidarProcessor(const LidarConfig & config);

  LidarResult process(const sensor_msgs::msg::LaserScan & raw, double stamp_s);

  void set_fault(const LidarFaultState & fault);
  const LidarFaultState & fault() const {return fault_;}

  /// Probability that any given beam is discarded right now.
  double effective_dropout() const;

  /// Noise this stage injects, which is zero at noise_scale = 1 when the input
  /// already carries the nominal noise. See `input_carries_nominal_noise`.
  double added_range_sigma() const;

  void reset();

private:
  LidarConfig config_;
  LidarFaultState fault_;
  RandomSource rng_;

  bool have_previous_{false};
  double previous_stamp_s_{0.0};
};

}  // namespace arb_sensors

#endif  // ARB_SENSORS__LIDAR_PROCESSOR_HPP_
