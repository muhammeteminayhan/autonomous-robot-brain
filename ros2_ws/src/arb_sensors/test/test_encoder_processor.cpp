// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_sensors/encoder_processor.hpp"

using arb_sensors::EncoderConfig;
using arb_sensors::EncoderFaultState;
using arb_sensors::EncoderProcessor;
using arb_sensors::EncoderReject;

namespace
{

/// A config with every random source switched off, so the deterministic
/// behaviour (quantisation, differencing, injected bias) can be asserted
/// exactly instead of statistically.
EncoderConfig quiet_config()
{
  EncoderConfig c;
  c.rate_hz = 50.0;
  c.velocity_noise_sigma = 0.0;
  c.slip_mean = 0.0;
  c.slip_sigma = 0.0;
  return c;
}

sensor_msgs::msg::JointState make_state(double left, double right, double stamp_s)
{
  sensor_msgs::msg::JointState js;
  js.header.stamp.sec = static_cast<int32_t>(stamp_s);
  js.header.stamp.nanosec =
    static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1e9);
  js.name = {"left_wheel_joint", "right_wheel_joint"};
  js.position = {left, right};
  js.velocity = {0.0, 0.0};
  return js;
}

}  // namespace

// ------------------------------------------------------------- quantisation

TEST(EncoderProcessor, ResolutionMatchesCountsAndQuadrature)
{
  EncoderConfig c = quiet_config();
  c.counts_per_revolution = 1024;
  c.quadrature_multiplier = 4;
  EncoderProcessor p(c);
  EXPECT_NEAR(p.resolution_rad(), arb_core::two_pi<double>() / 4096.0, 1e-15);
  // 1.534 mrad, which at a 60 mm wheel is 0.092 mm of arc.
  EXPECT_NEAR(p.resolution_rad(), 0.00153398, 1e-8);
}

TEST(EncoderProcessor, MotionSmallerThanOneCountIsNotReported)
{
  // The defining property of an encoder: sub-count motion is invisible. Code
  // that assumes it can resolve arbitrarily small increments will believe the
  // robot is stationary when it is creeping.
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  const double res = p.resolution_rad();

  p.process(make_state(0.0, 0.0, 0.0), 0.0);              // first sample primes state
  const auto r = p.process(make_state(res * 0.2, res * 0.2, 0.02), 0.02);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_DOUBLE_EQ(r.left_position_rad, 0.0);
  EXPECT_DOUBLE_EQ(r.left_velocity_rad_s, 0.0);
}

TEST(EncoderProcessor, ReportedIncrementsAreAlwaysWholeCounts)
{
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  const double res = p.resolution_rad();

  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0;
  double truth = 0.0;
  for (int i = 1; i <= 200; ++i) {
    t += 0.02;
    truth += 0.137;                          // an arbitrary non-multiple of res
    const auto r = p.process(make_state(truth, truth, t), t);
    ASSERT_TRUE(r.message.has_value());
    const double counts = r.left_position_rad / res;
    EXPECT_NEAR(counts, std::round(counts), 1e-6)
      << "reported position is not on the count grid at step " << i;
  }
}

TEST(EncoderProcessor, PositionTracksTruthToWithinOneCount)
{
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  const double res = p.resolution_rad();

  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0;
  double truth = 0.0;
  for (int i = 0; i < 500; ++i) {
    t += 0.02;
    truth += 0.2;
    const auto r = p.process(make_state(truth, truth, t), t);
    ASSERT_TRUE(r.message.has_value());
    // Quantisation error must not accumulate: it is bounded by one count
    // forever, because each increment is a difference of rounded positions.
    EXPECT_LE(std::fabs(r.left_position_rad - truth), res)
      << "quantisation error accumulated at step " << i;
  }
}

// ------------------------------------------------------------- velocity

TEST(EncoderProcessor, VelocityEqualsTheDifferencedPositionOverDt)
{
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);

  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0;
  double prev_pos = 0.0;
  double truth = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.02;
    truth += 0.1;
    const auto r = p.process(make_state(truth, truth, t), t);
    ASSERT_TRUE(r.message.has_value());
    EXPECT_NEAR(r.left_velocity_rad_s,
                (r.left_position_rad - prev_pos) / 0.02, 1e-9)
      << "position and velocity must be mutually consistent";
    prev_pos = r.left_position_rad;
  }
}

TEST(EncoderProcessor, VelocityQuantisationNoiseShrinksWithLongerIntervals)
{
  // Encoder velocity is NOT measured, it is differenced. Its quantisation
  // error is resolution/dt, so it is worst at LOW speed and short intervals --
  // the opposite of most people's intuition, and the reason Phase 3 integrates
  // positions rather than trusting reported velocities.
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  const double fast = p.velocity_sigma(0.002);
  const double slow = p.velocity_sigma(0.02);
  EXPECT_GT(fast, slow);
  EXPECT_NEAR(fast / slow, 10.0, 1e-9);
}

TEST(EncoderProcessor, VelocitySigmaCombinesQuantisationAndAdditiveNoise)
{
  EncoderConfig c = quiet_config();
  c.velocity_noise_sigma = 0.01;
  EncoderProcessor p(c);
  const double dt = 0.02;
  const double quant = p.resolution_rad() * std::sqrt(2.0 / 12.0) / dt;
  const double expected = std::sqrt(quant * quant + 0.01 * 0.01);
  EXPECT_NEAR(p.velocity_sigma(dt), expected, 1e-12);
}

// ------------------------------------------------------------- faults

/// Drive a bias on one wheel for 10 s of still-standing wheels and check the
/// accumulated angle, the reported rate, and that the other wheel is untouched.
///
/// Parameterised over BOTH wheels deliberately. Left and right are separate
/// code paths, and a mutation that broke only the left one survived an earlier
/// version of this test that exercised only the right.
class EncoderWheelBias : public ::testing::TestWithParam<bool> {};

TEST_P(EncoderWheelBias, RateBiasAccumulatesAsAngleOverTime)
{
  // A rate bias of b rad/s must add exactly b*t radians after t seconds, and
  // it must appear in the POSITION as well as the velocity -- otherwise a
  // consumer integrating the velocities would not land on the positions and
  // could detect the fault by that inconsistency alone.
  const bool left = GetParam();
  const double bias = 0.05;

  EncoderProcessor p{quiet_config()};
  EncoderFaultState f;
  (left ? f.left_bias_rad_s : f.right_bias_rad_s) = bias;
  p.set_fault(f);

  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0;
  for (int i = 0; i < 500; ++i) {         // 10 s, wheels physically still
    t += 0.02;
    p.process(make_state(0.0, 0.0, t), t);
  }
  const auto r = p.process(make_state(0.0, 0.0, t + 0.02), t + 0.02);
  ASSERT_TRUE(r.message.has_value());

  const double biased_pos = left ? r.left_position_rad : r.right_position_rad;
  const double clean_pos = left ? r.right_position_rad : r.left_position_rad;
  const double biased_vel = left ? r.left_velocity_rad_s : r.right_velocity_rad_s;

  // The published position is on the count grid, so it can differ from the
  // exact accumulated angle by up to half a count. The drift must still be
  // there in full: sub-count contributions accumulate at full precision and
  // only the output is rounded.
  EXPECT_NEAR(biased_pos, bias * (t + 0.02), p.resolution_rad());
  EXPECT_NEAR(biased_vel, bias, 4.0 * p.resolution_rad() / 0.02);
  EXPECT_NEAR(clean_pos, 0.0, 1e-12) << "the other wheel must be untouched";
}

INSTANTIATE_TEST_SUITE_P(
  BothWheels, EncoderWheelBias, ::testing::Values(true, false),
  [](const ::testing::TestParamInfo<bool> & info) {
    return info.param ? "Left" : "Right";
  });

TEST(EncoderProcessor, OutageDropsSamplesAndDoesNotAdvanceState)
{
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  const auto before = p.process(make_state(1.0, 1.0, 0.02), 0.02);
  ASSERT_TRUE(before.message.has_value());

  EncoderFaultState f;
  f.outage = true;
  p.set_fault(f);
  for (int i = 0; i < 10; ++i) {
    const auto r = p.process(make_state(2.0, 2.0, 0.04 + 0.02 * i), 0.04 + 0.02 * i);
    EXPECT_FALSE(r.message.has_value());
    EXPECT_EQ(r.reason, EncoderReject::kOutage);
  }

  // When the sensor returns, the accumulated motion is reported as one large
  // increment. That is what a real encoder does: it counted the whole time,
  // the link was down, and the consumer must cope with the gap.
  f.outage = false;
  p.set_fault(f);
  const auto after = p.process(make_state(2.0, 2.0, 0.30), 0.30);
  ASSERT_TRUE(after.message.has_value());
  EXPECT_NEAR(after.left_position_rad, 2.0, p.resolution_rad() * 2);
}

// ------------------------------------------------------------- robustness

TEST(EncoderProcessor, FirstSampleIsConsumedButNotPublished)
{
  // There is no velocity without two samples, and reporting a velocity
  // computed against an implicit zero would inject a spike at startup.
  EncoderProcessor p(quiet_config());
  const auto r = p.process(make_state(5.0, 5.0, 0.0), 0.0);
  EXPECT_FALSE(r.message.has_value());
  EXPECT_EQ(r.reason, EncoderReject::kFirstSample);
}

TEST(EncoderProcessor, ReportedPositionStartsAtZeroRegardlessOfWheelAngle)
{
  // An encoder counts from wherever it was switched on. No consumer may assume
  // an absolute wheel angle.
  EncoderProcessor p(quiet_config());
  p.process(make_state(123.456, -98.7, 0.0), 0.0);
  const auto r = p.process(make_state(123.456, -98.7, 0.02), 0.02);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.left_position_rad, 0.0, 1e-12);
  EXPECT_NEAR(r.right_position_rad, 0.0, 1e-12);
}

TEST(EncoderProcessor, RejectsMissingJointsNonFiniteAndBackwardsTime)
{
  EncoderProcessor p(quiet_config());
  p.process(make_state(0.0, 0.0, 0.0), 0.0);

  sensor_msgs::msg::JointState wrong;
  wrong.name = {"some_other_joint"};
  wrong.position = {1.0};
  EXPECT_EQ(p.process(wrong, 0.02).reason, EncoderReject::kMissingJoint);

  auto nan_state = make_state(std::nan(""), 0.0, 0.04);
  EXPECT_EQ(p.process(nan_state, 0.04).reason, EncoderReject::kNonFinite);

  ASSERT_TRUE(p.process(make_state(0.5, 0.5, 0.06), 0.06).message.has_value());
  EXPECT_EQ(p.process(make_state(0.6, 0.6, 0.05), 0.05).reason,
            EncoderReject::kNonMonotonicTime);
  EXPECT_EQ(p.process(make_state(0.6, 0.6, 0.06), 0.06).reason,
            EncoderReject::kNonMonotonicTime)
    << "a duplicate timestamp is as unusable as a backwards one";
}

TEST(EncoderProcessor, PublishedPositionsStayOnTheCountGridEvenWithNoiseAndSlip)
{
  // A quadrature encoder reports an integer count and nothing else. Slip,
  // drift and noise are all sub-count effects; they must accumulate at full
  // precision internally and be rounded only on the way out. An earlier
  // version added them after quantisation, producing positions that no real
  // encoder could report -- 598 of 598 live samples were off the grid.
  EncoderConfig c = quiet_config();
  c.velocity_noise_sigma = 0.05;
  c.slip_mean = 0.02;
  c.slip_sigma = 0.01;
  EncoderProcessor p(c);
  EncoderFaultState f;
  f.left_bias_rad_s = 0.031;    // deliberately not a multiple of a count
  p.set_fault(f);

  const double res = p.resolution_rad();
  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0, truth = 0.0;
  for (int i = 0; i < 300; ++i) {
    t += 0.02;
    truth += 0.173;
    const auto r = p.process(make_state(truth, truth, t), t);
    ASSERT_TRUE(r.message.has_value());
    for (const double pos : r.message->position) {
      const double counts = pos / res;
      EXPECT_NEAR(counts, std::round(counts), 1e-6)
        << "published position " << pos << " is off the count grid at step " << i;
    }
  }
}

TEST(EncoderProcessor, SubCountDriftStillAccumulatesDespiteQuantisation)
{
  // The hazard of rounding: a drift of 0.031 rad/s advances only 0.00062 rad
  // per 20 ms step, which is under half a count. Rounding each INCREMENT would
  // round every one of them to zero and the fault would never appear at all.
  EncoderConfig c = quiet_config();
  EncoderProcessor p(c);
  EncoderFaultState f;
  f.left_bias_rad_s = 0.031;
  p.set_fault(f);
  EXPECT_LT(0.031 * 0.02, p.resolution_rad() / 2.0)
    << "precondition: one step of drift is below half a count";

  p.process(make_state(0.0, 0.0, 0.0), 0.0);
  double t = 0.0;
  for (int i = 0; i < 500; ++i) {
    t += 0.02;
    p.process(make_state(0.0, 0.0, t), t);
  }
  const auto r = p.process(make_state(0.0, 0.0, t + 0.02), t + 0.02);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.left_position_rad, 0.031 * (t + 0.02), p.resolution_rad());
}

TEST(EncoderProcessor, ResetRestoresDeterminism)
{
  EncoderConfig c = quiet_config();
  c.velocity_noise_sigma = 0.05;
  c.slip_sigma = 0.02;
  EncoderProcessor p(c);

  auto run = [&p]() {
      std::vector<double> out;
      p.process(make_state(0.0, 0.0, 0.0), 0.0);
      double t = 0.0, truth = 0.0;
      for (int i = 0; i < 100; ++i) {
        t += 0.02;
        truth += 0.15;
        const auto r = p.process(make_state(truth, truth, t), t);
        if (r.message) {out.push_back(r.left_velocity_rad_s);}
      }
      return out;
    };

  const auto first = run();
  p.reset();
  const auto second = run();
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i], second[i]) << "sample " << i;
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
