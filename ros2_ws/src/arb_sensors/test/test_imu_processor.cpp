// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <numeric>
#include <vector>

#include "arb_sensors/imu_processor.hpp"

using arb_sensors::ImuConfig;
using arb_sensors::ImuFaultState;
using arb_sensors::ImuProcessor;
using arb_sensors::ImuReject;

namespace
{

ImuConfig quiet_config()
{
  ImuConfig c;
  c.rate_hz = 200.0;
  c.gyro_white_sigma = 0.0;
  c.gyro_bias_sigma = 0.0;
  c.gyro_bias_random_walk = 0.0;
  c.gyro_scale_error = 0.0;
  c.accel_white_sigma = 0.0;
  c.accel_bias_sigma = 0.0;
  c.accel_bias_random_walk = 0.0;
  c.accel_scale_error = 0.0;
  c.calibration_samples = 0;
  // The tests feed clean synthetic samples, so this stage is the only noise
  // source. In the running system Gazebo supplies the nominal noise and the
  // flag is true; see ChannelNoise::added_sigma.
  c.input_carries_nominal_noise = false;
  return c;
}

sensor_msgs::msg::Imu make_imu(double wz, double ax, double stamp_s)
{
  sensor_msgs::msg::Imu m;
  m.header.frame_id = "imu_link";
  m.header.stamp.sec = static_cast<int32_t>(stamp_s);
  m.header.stamp.nanosec =
    static_cast<uint32_t>((stamp_s - std::floor(stamp_s)) * 1e9);
  m.angular_velocity.z = wz;
  m.linear_acceleration.x = ax;
  m.linear_acceleration.z = 9.81;
  return m;
}

double mean_of(const std::vector<double> & xs)
{
  return std::accumulate(xs.begin(), xs.end(), 0.0) / static_cast<double>(xs.size());
}

double stdev_of(const std::vector<double> & xs)
{
  const double m = mean_of(xs);
  double acc = 0.0;
  for (const double x : xs) {acc += (x - m) * (x - m);}
  return std::sqrt(acc / static_cast<double>(xs.size() - 1));
}

}  // namespace

TEST(ImuProcessor, PassesCleanDataThroughWhenAllNoiseIsDisabled)
{
  ImuProcessor p(quiet_config());
  const auto r = p.process(make_imu(0.5, 0.1, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.message->angular_velocity.z, 0.5, 1e-12);
  EXPECT_NEAR(r.message->linear_acceleration.x, 0.1, 1e-12);
}

TEST(ImuProcessor, FillsCovarianceFromTheNoiseModel)
{
  // Gazebo publishes an all-zero covariance, which in ROS 2 means "unknown",
  // not "perfect". Filling it from the same model that generated the noise is
  // what lets the Phase 4 EKF read R off the message instead of inventing one.
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.01;
  c.accel_white_sigma = 0.2;
  ImuProcessor p(c);
  const auto r = p.process(make_imu(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.message->angular_velocity_covariance[0], 1e-4, 1e-15);
  EXPECT_NEAR(r.message->angular_velocity_covariance[4], 1e-4, 1e-15);
  EXPECT_NEAR(r.message->angular_velocity_covariance[8], 1e-4, 1e-15);
  EXPECT_NEAR(r.message->linear_acceleration_covariance[0], 0.04, 1e-15);
  // Off-diagonal terms must be zero: the axes are modelled as independent.
  EXPECT_DOUBLE_EQ(r.message->angular_velocity_covariance[1], 0.0);
}

TEST(ImuProcessor, MarksOrientationAsUnavailable)
{
  // This is a rate gyro plus accelerometer, not an AHRS. ROS signals "no
  // orientation" with covariance[0] = -1. A consumer that ignores the flag and
  // uses the identity quaternion gets a robot that always believes it faces +x.
  ImuProcessor p(quiet_config());
  const auto r = p.process(make_imu(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_DOUBLE_EQ(r.message->orientation_covariance[0], -1.0);
}

TEST(ImuProcessor, NoiseScaleWidensTheReportedCovarianceToo)
{
  // The critical property for Phase 10 case B: when a sensor degrades, the
  // filter must be TOLD, not merely fed worse numbers. If sigma grows but the
  // reported covariance does not, the EKF keeps trusting the IMU and diverges.
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.01;
  ImuProcessor p(c);

  ImuFaultState f;
  f.noise_scale = 10.0;
  p.set_fault(f);

  const auto r = p.process(make_imu(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(p.gyro_sigma(), 0.1, 1e-12);
  EXPECT_NEAR(r.message->angular_velocity_covariance[0], 0.01, 1e-12);
}

TEST(ImuProcessor, GyroAndAccelerometerNoiseScaleIndependently)
{
  // The scenarios distinguish them: Case B degrades the whole package, Case F
  // only the gyro. With one shared knob, `affect_accelerometer: false` in
  // configs/scenarios/case_f_multi_degradation.yaml was a comment rather than
  // a setting -- the accelerometer was degraded anyway and nothing said so.
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.01;
  c.accel_white_sigma = 0.05;
  ImuProcessor p(c);

  ImuFaultState f;
  f.noise_scale = 10.0;         // gyro only
  f.accel_noise_scale = 1.0;
  p.set_fault(f);
  const auto gyro_only = p.process(make_imu(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(gyro_only.message.has_value());
  EXPECT_NEAR(p.gyro_sigma(), 0.10, 1e-12);
  EXPECT_NEAR(p.accel_sigma(), 0.05, 1e-12) << "accelerometer must be untouched";
  EXPECT_NEAR(gyro_only.message->linear_acceleration_covariance[0],
              0.05 * 0.05, 1e-12);

  f.noise_scale = 1.0;          // accelerometer only
  f.accel_noise_scale = 4.0;
  p.set_fault(f);
  const auto accel_only = p.process(make_imu(0.0, 0.0, 0.0), 0.01);
  ASSERT_TRUE(accel_only.message.has_value());
  EXPECT_NEAR(p.gyro_sigma(), 0.01, 1e-12) << "gyro must be untouched";
  EXPECT_NEAR(p.accel_sigma(), 0.20, 1e-12);
  EXPECT_NEAR(accel_only.message->angular_velocity_covariance[0],
              0.01 * 0.01, 1e-12);
}

TEST(ImuProcessor, InjectedNoiseHasTheConfiguredSpread)
{
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.02;
  ImuProcessor p(c);
  std::vector<double> xs;
  double t = 0.0;
  for (int i = 0; i < 20000; ++i) {
    t += 0.005;
    const auto r = p.process(make_imu(0.0, 0.0, t), t);
    ASSERT_TRUE(r.message.has_value());
    xs.push_back(r.message->angular_velocity.z);
  }
  EXPECT_NEAR(stdev_of(xs), 0.02, 0.001);
  EXPECT_NEAR(mean_of(xs), 0.0, 0.002);
}

TEST(ImuProcessor, CalibrationRemovesTheTurnOnBias)
{
  // Average N stationary samples, subtract the result. This is what a real
  // robot does on boot and why you are told not to move it while it starts.
  ImuConfig c = quiet_config();
  c.calibration_samples = 100;
  ImuProcessor p(c);

  ImuFaultState f;
  f.gyro_bias_offset = 0.03;   // stand in for a turn-on bias
  p.set_fault(f);

  double t = 0.0;
  for (int i = 0; i < 100; ++i) {
    t += 0.005;
    const auto r = p.process(make_imu(0.0, 0.0, t), t);
    EXPECT_FALSE(r.message.has_value());
    EXPECT_EQ(r.reason, ImuReject::kCalibrating);
  }
  EXPECT_TRUE(p.calibrated());
  EXPECT_NEAR(p.gyro_calibration_bias()[2], 0.03, 1e-9);

  t += 0.005;
  const auto r = p.process(make_imu(0.0, 0.0, t), t);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.message->angular_velocity.z, 0.0, 1e-9)
    << "the estimated bias must be subtracted after calibration";
}

TEST(ImuProcessor, CalibrationDoesNotRemoveRealRotation)
{
  // The failure mode of this technique: calibrate while the robot is turning
  // and you permanently subtract a real rate. The processor cannot detect that
  // on its own, which is exactly why the node only calibrates at startup and
  // the launch file keeps the robot still. This test documents the behaviour.
  ImuConfig c = quiet_config();
  c.calibration_samples = 50;
  ImuProcessor p(c);
  double t = 0.0;
  for (int i = 0; i < 50; ++i) {
    t += 0.005;
    p.process(make_imu(0.4, 0.0, t), t);   // robot is genuinely turning
  }
  EXPECT_NEAR(p.gyro_calibration_bias()[2], 0.4, 1e-9);
  t += 0.005;
  const auto r = p.process(make_imu(0.4, 0.0, t), t);
  ASSERT_TRUE(r.message.has_value());
  EXPECT_NEAR(r.message->angular_velocity.z, 0.0, 1e-9)
    << "documents the hazard: a real rate was absorbed into the bias estimate";
}

TEST(ImuProcessor, OutageDropsEverythingAndRecovers)
{
  ImuProcessor p(quiet_config());
  ASSERT_TRUE(p.process(make_imu(0.1, 0.0, 0.0), 0.0).message.has_value());

  ImuFaultState f;
  f.outage = true;
  p.set_fault(f);
  for (int i = 1; i <= 20; ++i) {
    const auto r = p.process(make_imu(0.1, 0.0, 0.005 * i), 0.005 * i);
    EXPECT_FALSE(r.message.has_value());
    EXPECT_EQ(r.reason, ImuReject::kOutage);
  }

  f.outage = false;
  p.set_fault(f);
  const auto back = p.process(make_imu(0.1, 0.0, 1.0), 1.0);
  EXPECT_TRUE(back.message.has_value()) << "the stream must resume cleanly";
}

TEST(ImuProcessor, RejectsNonFiniteAndNonMonotonicSamples)
{
  ImuProcessor p(quiet_config());
  ASSERT_TRUE(p.process(make_imu(0.0, 0.0, 1.0), 1.0).message.has_value());

  EXPECT_EQ(p.process(make_imu(std::nan(""), 0.0, 1.005), 1.005).reason,
            ImuReject::kNonFinite);
  EXPECT_EQ(p.process(make_imu(INFINITY, 0.0, 1.005), 1.005).reason,
            ImuReject::kNonFinite);
  EXPECT_EQ(p.process(make_imu(0.0, 0.0, 0.5), 0.5).reason,
            ImuReject::kNonMonotonicTime);
  EXPECT_EQ(p.process(make_imu(0.0, 0.0, 1.0), 1.0).reason,
            ImuReject::kNonMonotonicTime);
}

TEST(ImuProcessor, ClampsAbsurdTimeGapsInsteadOfJumpingTheBiasProcess)
{
  // After a long stall the measured dt can be huge. Advancing a random-walk
  // bias by that whole interval in one step injects a jump that never happened.
  ImuConfig c = quiet_config();
  c.gyro_bias_sigma = 0.0;
  c.gyro_bias_random_walk = 1.0;   // deliberately enormous
  c.gyro_bias_tau_s = -1.0;        // pure random walk
  c.max_dt_s = 0.05;
  ImuProcessor p(c);

  p.process(make_imu(0.0, 0.0, 0.0), 0.0);
  const auto r = p.process(make_imu(0.0, 0.0, 100.0), 100.0);
  ASSERT_TRUE(r.message.has_value());
  // sqrt(max_dt) * sigma = sqrt(0.05) = 0.224, so a handful of sigma is the
  // bound; without clamping the step would be sqrt(100) = 10 times larger.
  EXPECT_LT(std::fabs(r.message->angular_velocity.z), 1.5);
}

TEST(ImuProcessor, ResetRestoresDeterminism)
{
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.01;
  c.gyro_bias_sigma = 0.002;
  ImuProcessor p(c);

  auto run = [&p]() {
      std::vector<double> out;
      double t = 0.0;
      for (int i = 0; i < 500; ++i) {
        t += 0.005;
        const auto r = p.process(make_imu(0.2, 0.0, t), t);
        if (r.message) {out.push_back(r.message->angular_velocity.z);}
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

TEST(ImuProcessor, AddsNothingAtNominalWhenTheInputAlreadyCarriesTheNoise)
{
  // Gazebo already applies the datasheet noise. Adding another full sigma on
  // top combines as sqrt(2)*sigma: the stream is 41 % noisier than the part we
  // claim to simulate, and every published covariance is wrong by the same
  // factor. At noise_scale = 1 this stage must inject exactly nothing.
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = 0.02;
  c.input_carries_nominal_noise = true;
  ImuProcessor p(c);

  double t = 0.0;
  for (int i = 0; i < 200; ++i) {
    t += 0.005;
    const auto r = p.process(make_imu(0.3, 0.0, t), t);
    ASSERT_TRUE(r.message.has_value());
    EXPECT_DOUBLE_EQ(r.message->angular_velocity.z, 0.3)
      << "nominal operation must not add noise to an already-noisy input";
  }
  // The REPORTED sigma is still the full nominal value, because that is what
  // the sample actually carries.
  EXPECT_NEAR(p.gyro_sigma(), 0.02, 1e-12);
}

TEST(ImuProcessor, InjectsOnlyTheExtraVarianceNeededToReachTheTarget)
{
  // Independent variances add, so reaching 10x sigma from an input that
  // already carries sigma means injecting sqrt(10^2 - 1) = 9.95 sigma, not 10.
  const double sigma = 0.02;
  const double scale = 10.0;
  ImuConfig c = quiet_config();
  c.gyro_white_sigma = sigma;
  c.input_carries_nominal_noise = true;
  ImuProcessor p(c);

  ImuFaultState f;
  f.noise_scale = scale;
  p.set_fault(f);

  std::vector<double> xs;
  double t = 0.0;
  for (int i = 0; i < 20000; ++i) {
    t += 0.005;
    const auto r = p.process(make_imu(0.0, 0.0, t), t);
    ASSERT_TRUE(r.message.has_value());
    xs.push_back(r.message->angular_velocity.z);
  }
  const double expected_added = sigma * std::sqrt(scale * scale - 1.0);
  EXPECT_NEAR(stdev_of(xs), expected_added, 0.03 * expected_added);
  // Total, including the input's own noise, is the full target.
  EXPECT_NEAR(p.gyro_sigma(), sigma * scale, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
