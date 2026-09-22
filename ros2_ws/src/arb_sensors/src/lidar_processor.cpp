// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_sensors/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace arb_sensors
{
namespace
{
constexpr float kNoReturn = std::numeric_limits<float>::infinity();

bool is_measurable(float r, float range_min, float range_max)
{
  return std::isfinite(r) && r >= range_min && r <= range_max;
}

}  // namespace

LidarProcessor::LidarProcessor(const LidarConfig & config)
: config_(config), rng_(config.seed, "lidar") {}

void LidarProcessor::set_fault(const LidarFaultState & fault) {fault_ = fault;}

double LidarProcessor::added_range_sigma() const
{
  const double target = config_.range_noise_sigma * fault_.noise_scale;
  if (!config_.input_carries_nominal_noise) {return target;}
  const double extra = target * target -
    config_.range_noise_sigma * config_.range_noise_sigma;
  return (extra > 0.0) ? std::sqrt(extra) : 0.0;
}

double LidarProcessor::effective_dropout() const
{
  double p = config_.dropout_probability;
  if (fault_.burst_mode) {
    // In burst mode the fault is all-or-nothing: either the scanner is having
    // a bad moment and loses `dropout_ratio` of its beams, or it is fine.
    // Spreading the same average loss uniformly over time would be far gentler
    // and would not reproduce the failure we want to study, where the estimator
    // must survive several consecutive near-useless scans.
    if (fault_.burst_active) {p += fault_.dropout_ratio;}
  } else {
    p += fault_.dropout_ratio;
  }
  return std::clamp(p, 0.0, 1.0);
}

void LidarProcessor::reset()
{
  rng_.reseed(config_.seed, "lidar");
  have_previous_ = false;
  fault_ = LidarFaultState{};
}

LidarResult LidarProcessor::process(
  const sensor_msgs::msg::LaserScan & raw, double stamp_s)
{
  LidarResult result;

  if (fault_.outage) {
    result.reason = LidarReject::kOutage;
    return result;
  }
  if (raw.ranges.empty()) {
    result.reason = LidarReject::kEmptyScan;
    return result;
  }
  if (have_previous_ && config_.enforce_monotonic_time &&
      stamp_s <= previous_stamp_s_)
  {
    result.reason = LidarReject::kNonMonotonicTime;
    return result;
  }
  previous_stamp_s_ = stamp_s;
  have_previous_ = true;

  sensor_msgs::msg::LaserScan out = raw;
  const float range_min = raw.range_min;
  const float range_max = raw.range_max;
  const double p_drop = effective_dropout();
  const double sigma = added_range_sigma();

  for (std::size_t i = 0; i < out.ranges.size(); ++i) {
    const float r = raw.ranges[i];

    if (!is_measurable(r, range_min, range_max)) {
      // Normalise every unmeasurable case to one sentinel.
      out.ranges[i] = kNoReturn;
      continue;
    }
    float noisy = static_cast<float>(rng_.gaussian(r, sigma));
    // Noise can push a valid return outside the sensor's range. Clamping keeps
    // the message self-consistent; letting it through would make downstream
    // validity checks disagree with each other.
    noisy = std::clamp(noisy, range_min, range_max);
    out.ranges[i] = noisy;
    ++result.valid_beams;
  }

  // --- despeckle ----------------------------------------------------------
  // A return that neither angular neighbour could share a surface with is
  // almost always an edge effect: a beam clipping the corner of an obstacle.
  // Left alone it becomes a permanent phantom cell in the occupancy grid.
  //
  // "Could share a surface with" is a GEOMETRIC test, not a fixed distance.
  // Along a flat surface seen at obliquity phi from its normal, consecutive
  // beams separated by d(theta) differ in range by
  //
  //     delta_r = r * tan(phi) * d(theta)
  //
  // which grows with range. Accepting surfaces up to `max_obliquity` and
  // rejecting anything beyond keeps grazing-incidence wall returns -- the
  // ones a fixed threshold throws away precisely where the scan is most
  // valuable, far down a corridor.
  //
  // The scan is circular, so neighbours wrap: beam 0's predecessor is the last
  // beam, not a missing index.
  if (config_.despeckle && out.ranges.size() >= 3) {
    const std::vector<float> before = out.ranges;
    const std::size_t n = before.size();
    const double tan_max =
      std::tan(std::min(config_.despeckle_max_obliquity_deg, 89.9) * M_PI / 180.0);
    const double step = std::fabs(static_cast<double>(out.angle_increment));

    for (std::size_t i = 0; i < n; ++i) {
      const float r = before[i];
      if (!is_measurable(r, range_min, range_max)) {continue;}

      const double threshold =
        std::max(static_cast<double>(config_.despeckle_floor_m), r * tan_max * step);

      const float prev = before[(i + n - 1) % n];
      const float next = before[(i + 1) % n];
      const bool prev_near = is_measurable(prev, range_min, range_max) &&
        std::fabs(static_cast<double>(prev - r)) < threshold;
      const bool next_near = is_measurable(next, range_min, range_max) &&
        std::fabs(static_cast<double>(next - r)) < threshold;
      if (!prev_near && !next_near) {
        out.ranges[i] = kNoReturn;
        ++result.despeckled_beams;
        --result.valid_beams;
      }
    }
  }

  // --- beam dropout ------------------------------------------------------
  // Applied LAST, after despeckling, and the ordering is deliberate.
  //
  // Running dropout first and despeckling afterwards looks more physical --
  // beams are lost, then the scan is cleaned -- but it destroys the experiment.
  // Once 60 % of beams are gone at random, most survivors have no surviving
  // neighbour, so the despeckle filter removes them too: a commanded 60 % loss
  // measured 75.5 % in practice. The Phase 10 table would then have a row
  // labelled "60 % dropout" describing something else.
  //
  // A fault injector is an instrument. An instrument whose setting is silently
  // amplified by a downstream filter is not calibrated, so the loss is applied
  // where nothing can amplify it.
  if (p_drop > 0.0) {
    for (std::size_t i = 0; i < out.ranges.size(); ++i) {
      if (!is_measurable(out.ranges[i], range_min, range_max)) {continue;}
      if (rng_.bernoulli(p_drop)) {
        out.ranges[i] = kNoReturn;
        ++result.dropped_beams;
        --result.valid_beams;
      }
    }
  }

  result.message = std::move(out);
  return result;
}

}  // namespace arb_sensors
