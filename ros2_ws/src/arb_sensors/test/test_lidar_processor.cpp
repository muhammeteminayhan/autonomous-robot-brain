// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "arb_sensors/lidar_processor.hpp"

using arb_sensors::LidarConfig;
using arb_sensors::LidarFaultState;
using arb_sensors::LidarProcessor;
using arb_sensors::LidarReject;

namespace
{

constexpr float kInf = std::numeric_limits<float>::infinity();

LidarConfig quiet_config()
{
  LidarConfig c;
  c.range_noise_sigma = 0.0;
  c.input_carries_nominal_noise = false;
  c.dropout_probability = 0.0;
  c.despeckle = false;
  return c;
}

sensor_msgs::msg::LaserScan make_scan(std::vector<float> ranges, double stamp_s)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = "lidar_link";
  s.header.stamp.sec = static_cast<int32_t>(stamp_s);
  s.header.stamp.nanosec =
    static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1e9);
  // A realistic 1 degree beam spacing regardless of how many beams the test
  // uses. The despeckle threshold is derived from angle_increment, so a test
  // scan with six beams spread over a full circle would imply a 60 degree
  // spacing and a nonsensically permissive threshold.
  s.angle_min = -3.14159265f;
  s.angle_increment = static_cast<float>(2.0 * 3.14159265 / 360.0);
  s.angle_max = s.angle_min + s.angle_increment * (ranges.size() - 1);
  s.range_min = 0.12f;
  s.range_max = 12.0f;
  s.ranges = std::move(ranges);
  return s;
}

int count_finite(const std::vector<float> & r)
{
  int n = 0;
  for (const float v : r) {
    if (std::isfinite(v)) {++n;}
  }
  return n;
}

}  // namespace

// ------------------------------------------------- invalid-value normalisation

TEST(LidarProcessor, NormalisesEveryUnmeasurableBeamToPositiveInfinity)
{
  // ROS defines three distinct "no reading" cases: r > range_max (hit nothing),
  // r < range_min (too close), and NaN (invalid). They are not interchangeable
  // and none of them is a distance. Code like `if (r < 0.5) obstacle()` fires
  // on every -inf beam; averaging a scan containing NaN gives NaN. Collapsing
  // them to ONE sentinel leaves downstream code a single case to handle.
  LidarProcessor p(quiet_config());
  const auto scan = make_scan(
    {1.0f, kInf, -kInf, std::nanf(""), 0.05f, 50.0f, 3.0f}, 0.0);
  const auto r = p.process(scan, 0.0);
  ASSERT_TRUE(r.message.has_value());

  const auto & out = r.message->ranges;
  EXPECT_FLOAT_EQ(out[0], 1.0f);
  for (const std::size_t i : {1u, 2u, 3u, 4u, 5u}) {
    EXPECT_TRUE(std::isinf(out[i]) && out[i] > 0.0f)
      << "index " << i << " should be +inf, got " << out[i];
  }
  EXPECT_FLOAT_EQ(out[6], 3.0f);
  EXPECT_EQ(r.valid_beams, 2);
}

TEST(LidarProcessor, PreservesScanGeometryAndHeader)
{
  LidarProcessor p(quiet_config());
  const auto scan = make_scan({1.0f, 2.0f, 3.0f, 4.0f}, 5.0);
  const auto r = p.process(scan, 5.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_EQ(r.message->ranges.size(), scan.ranges.size());
  EXPECT_FLOAT_EQ(r.message->angle_min, scan.angle_min);
  EXPECT_FLOAT_EQ(r.message->angle_increment, scan.angle_increment);
  EXPECT_FLOAT_EQ(r.message->range_min, scan.range_min);
  EXPECT_FLOAT_EQ(r.message->range_max, scan.range_max);
  EXPECT_EQ(r.message->header.frame_id, "lidar_link");
}

TEST(LidarProcessor, NoisyRangesAreClampedIntoTheSensorLimits)
{
  // Noise can push a valid return outside [range_min, range_max]. Letting it
  // through makes the message self-inconsistent: downstream validity checks
  // would then disagree with each other.
  LidarConfig c = quiet_config();
  c.range_noise_sigma = 3.0;     // deliberately extreme
  LidarProcessor p(c);
  double t = 0.0;
  for (int i = 0; i < 200; ++i) {
    t += 0.1;
    const auto r = p.process(make_scan({0.2f, 11.9f, 6.0f}, t), t);
    ASSERT_TRUE(r.message.has_value());
    for (const float v : r.message->ranges) {
      if (std::isfinite(v)) {
        EXPECT_GE(v, 0.12f);
        EXPECT_LE(v, 12.0f);
      }
    }
  }
}

// ------------------------------------------------------------------ dropout

TEST(LidarProcessor, DropoutRemovesApproximatelyTheRequestedFraction)
{
  LidarConfig c = quiet_config();
  LidarProcessor p(c);
  LidarFaultState f;
  f.dropout_ratio = 0.6;
  p.set_fault(f);
  EXPECT_NEAR(p.effective_dropout(), 0.6, 1e-12);

  std::vector<float> ranges(360, 5.0f);
  int kept = 0;
  int total = 0;
  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.1;
    const auto r = p.process(make_scan(ranges, t), t);
    ASSERT_TRUE(r.message.has_value());
    kept += count_finite(r.message->ranges);
    total += static_cast<int>(ranges.size());
  }
  EXPECT_NEAR(static_cast<double>(kept) / total, 0.4, 0.02);
}

TEST(LidarProcessor, BurstModeOnlyDropsWhileTheBurstIsActive)
{
  // Spreading the same average loss uniformly is far gentler than losing 60 %
  // of beams for three seconds at a time. The bursty version is what forces the
  // estimator to survive several consecutive near-useless scans.
  LidarConfig c = quiet_config();
  LidarProcessor p(c);
  LidarFaultState f;
  f.dropout_ratio = 0.8;
  f.burst_mode = true;
  f.burst_active = false;
  p.set_fault(f);
  EXPECT_NEAR(p.effective_dropout(), 0.0, 1e-12);

  std::vector<float> ranges(200, 4.0f);
  auto r = p.process(make_scan(ranges, 1.0), 1.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_EQ(count_finite(r.message->ranges), 200);

  f.burst_active = true;
  p.set_fault(f);
  EXPECT_NEAR(p.effective_dropout(), 0.8, 1e-12);
  r = p.process(make_scan(ranges, 2.0), 2.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_LT(count_finite(r.message->ranges), 80);
}

TEST(LidarProcessor, EffectiveDropoutIsClampedToAValidProbability)
{
  LidarConfig c = quiet_config();
  c.dropout_probability = 0.7;
  LidarProcessor p(c);
  LidarFaultState f;
  f.dropout_ratio = 0.9;         // 0.7 + 0.9 would be 1.6
  p.set_fault(f);
  EXPECT_DOUBLE_EQ(p.effective_dropout(), 1.0);
}

// ---------------------------------------------------------------- despeckle

TEST(LidarProcessor, ThresholdGrowsWithRangeSoGrazingSurfacesSurvive)
{
  // The regression this encodes: a fixed threshold deletes grazing-incidence
  // returns far down a corridor, which is exactly where the scan carries the
  // most information. Neighbouring beams on a distant oblique wall step by
  // r * tan(phi) * d(theta); at 10 m and 1 degree that is 0.65 m at 75 degrees
  // obliquity, far beyond any sensible fixed threshold.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  c.despeckle_max_obliquity_deg = 85.0;
  LidarProcessor p(c);

  // A wall receding from 4 m to 9.5 m at roughly 80 degrees obliquity:
  // 0.45 m per beam, which at 4 m is tan(81 deg) * r * d(theta). Every range
  // stays inside [range_min, range_max] so the only way a beam can vanish is
  // despeckling.
  std::vector<float> ranges;
  for (int i = 0; i < 13; ++i) {ranges.push_back(4.0f + 0.45f * i);}
  ASSERT_LT(ranges.back(), 12.0f) << "the ramp must stay inside the sensor range";

  const auto r = p.process(make_scan(ranges, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  // Only the two ends are ambiguous: their outer neighbour wraps to the far
  // end of the ramp, which really is discontinuous. The interior must survive.
  for (std::size_t i = 1; i + 1 < ranges.size(); ++i) {
    EXPECT_TRUE(std::isfinite(r.message->ranges[i]))
      << "beam " << i << " at " << ranges[i]
      << " m is a legitimate grazing return and must not be despeckled";
  }
}

TEST(LidarProcessor, ATooStrictObliquityLimitDestroysTheSameGrazingWall)
{
  // The other half of the previous test: the geometric threshold is what saves
  // that wall, not luck. Allowing only 45 degrees of obliquity shrinks the
  // threshold to r * 1.0 * d(theta), which at 4 m is 0.07 m -- far below the
  // wall's real 0.45 m per-beam step -- and the whole surface is discarded.
  //
  // This is the failure the fixed 0.25 m threshold produced on the real world:
  // 12 beams per scan removed, 99.9 % of them within 5 cm of an actual wall.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  c.despeckle_max_obliquity_deg = 45.0;
  c.despeckle_floor_m = 0.05;
  LidarProcessor p(c);

  std::vector<float> ranges;
  for (int i = 0; i < 13; ++i) {ranges.push_back(4.0f + 0.45f * i);}
  const auto r = p.process(make_scan(ranges, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_GT(r.despeckled_beams, 10)
    << "a too-strict obliquity limit must be visible as mass data loss";
}

TEST(LidarProcessor, RemovesIsolatedReturnsButKeepsSurfaces)
{
  // A lone return whose angular neighbours are far away is almost always a
  // beam clipping a corner. Left alone it becomes a permanent phantom cell in
  // the occupancy grid.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  LidarProcessor p(c);

  //            wall surface (contiguous)        speckle    wall
  std::vector<float> ranges = {5.0f, 5.02f, 4.98f, 5.01f, 1.0f, 5.0f, 5.03f, 4.99f};
  const auto r = p.process(make_scan(ranges, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_TRUE(std::isinf(r.message->ranges[4])) << "isolated point must be removed";
  EXPECT_EQ(r.despeckled_beams, 1);
  for (const std::size_t i : {0u, 1u, 2u, 3u, 5u, 6u, 7u}) {
    EXPECT_TRUE(std::isfinite(r.message->ranges[i]))
      << "surface point " << i << " must survive";
  }
}

TEST(LidarProcessor, DespeckleRemovesASpeckleAtIndexZeroUsingItsWrappedNeighbour)
{
  // Beam 0's neighbours are beam N-1 and beam 1. A linear implementation that
  // has no index -1 might compare beam 0 against ITSELF, which always looks
  // "near" and quietly disables despeckling at the seam. Making beam 0 the
  // speckle is what catches that: it must be removed.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  LidarProcessor p(c);

  std::vector<float> ranges = {1.0f, 5.0f, 5.01f, 4.99f, 5.02f, 5.0f};
  const auto r = p.process(make_scan(ranges, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_TRUE(std::isinf(r.message->ranges[0]))
    << "beam 0 is isolated and must be despeckled";
  EXPECT_EQ(r.despeckled_beams, 1);
  for (std::size_t i = 1; i < ranges.size(); ++i) {
    EXPECT_TRUE(std::isfinite(r.message->ranges[i])) << "surface point " << i;
  }
}

TEST(LidarProcessor, DespeckleWrapsAroundTheScan)
{
  // The scan is circular: beam 0 and beam N-1 are angular neighbours. Treating
  // the array as a line removes two genuine surface points at every scan.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  LidarProcessor p(c);
  std::vector<float> ranges = {5.0f, kInf, kInf, kInf, kInf, 5.01f};
  const auto r = p.process(make_scan(ranges, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_TRUE(std::isfinite(r.message->ranges[0]))
    << "beam 0's neighbour is the last beam, not index 1";
  EXPECT_TRUE(std::isfinite(r.message->ranges[5]));
  EXPECT_EQ(r.despeckled_beams, 0);
}

// ------------------------------------------------------------- robustness

TEST(LidarProcessor, OutageAndEmptyAndBackwardsTimeAreRejectedDistinctly)
{
  LidarProcessor p(quiet_config());
  ASSERT_TRUE(p.process(make_scan({1.0f, 2.0f}, 1.0), 1.0).message.has_value());

  EXPECT_EQ(p.process(make_scan({}, 2.0), 2.0).reason, LidarReject::kEmptyScan);
  EXPECT_EQ(p.process(make_scan({1.0f}, 0.5), 0.5).reason,
            LidarReject::kNonMonotonicTime);

  LidarFaultState f;
  f.outage = true;
  p.set_fault(f);
  EXPECT_EQ(p.process(make_scan({1.0f, 2.0f}, 3.0), 3.0).reason,
            LidarReject::kOutage);

  f.outage = false;
  p.set_fault(f);
  EXPECT_TRUE(p.process(make_scan({1.0f, 2.0f}, 4.0), 4.0).message.has_value());
}

TEST(LidarProcessor, ResetRestoresDeterminism)
{
  LidarConfig c = quiet_config();
  c.range_noise_sigma = 0.05;
  LidarProcessor p(c);
  LidarFaultState f;
  f.dropout_ratio = 0.2;
  p.set_fault(f);

  auto run = [&p]() {
      std::vector<float> out;
      std::vector<float> ranges(180, 3.0f);
      double t = 0.0;
      for (int i = 0; i < 20; ++i) {
        t += 0.1;
        const auto r = p.process(make_scan(ranges, t), t);
        if (r.message) {
          out.insert(out.end(), r.message->ranges.begin(), r.message->ranges.end());
        }
      }
      return out;
    };

  const auto first = run();
  p.reset();
  LidarFaultState f2;
  f2.dropout_ratio = 0.2;
  p.set_fault(f2);
  const auto second = run();
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    if (std::isinf(first[i])) {
      EXPECT_TRUE(std::isinf(second[i])) << "beam " << i;
    } else {
      EXPECT_FLOAT_EQ(first[i], second[i]) << "beam " << i;
    }
  }
}

TEST(LidarProcessor, DespeckleDoesNotAmplifyTheCommandedDropout)
{
  // Ordering regression. With despeckling applied AFTER dropout, a random 60 %
  // loss leaves most survivors without a surviving neighbour, and the filter
  // removes them too: a commanded 60 % measured 75.5 %. The Phase 10 benchmark
  // row labelled "60 % dropout" would then describe something else.
  //
  // A fault injector is an instrument; an instrument whose setting is silently
  // amplified by a downstream filter is not calibrated.
  LidarConfig c = quiet_config();
  c.despeckle = true;
  LidarProcessor p(c);
  LidarFaultState f;
  f.dropout_ratio = 0.6;
  p.set_fault(f);

  std::vector<float> ranges(360, 5.0f);   // a perfectly clean wall everywhere
  int kept = 0;
  int total = 0;
  double t = 0.0;
  for (int i = 0; i < 40; ++i) {
    t += 0.1;
    const auto r = p.process(make_scan(ranges, t), t);
    ASSERT_TRUE(r.message.has_value());
    for (const float v : r.message->ranges) {
      if (std::isfinite(v)) {++kept;}
    }
    total += static_cast<int>(ranges.size());
  }
  EXPECT_NEAR(static_cast<double>(kept) / total, 0.4, 0.03)
    << "commanded 60 % dropout must remove 60 %, not more";
}

TEST(LidarProcessor, AddsNoNoiseAtNominalWhenTheInputAlreadyCarriesIt)
{
  LidarConfig c = quiet_config();
  c.range_noise_sigma = 0.015;
  c.input_carries_nominal_noise = true;
  LidarProcessor p(c);
  EXPECT_NEAR(p.added_range_sigma(), 0.0, 1e-15);

  const auto r = p.process(make_scan({3.0f, 3.01f, 2.99f}, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_FLOAT_EQ(r.message->ranges[0], 3.0f);

  LidarFaultState f;
  f.noise_scale = 5.0;
  p.set_fault(f);
  EXPECT_NEAR(p.added_range_sigma(), 0.015 * std::sqrt(24.0), 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
