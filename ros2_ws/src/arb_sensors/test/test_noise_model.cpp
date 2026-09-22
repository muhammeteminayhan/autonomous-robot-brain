// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

#include "arb_sensors/noise_model.hpp"

using arb_sensors::ChannelNoise;
using arb_sensors::GaussMarkovBias;
using arb_sensors::RandomSource;

namespace
{

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

// ------------------------------------------------------------- RandomSource

TEST(RandomSource, SameSeedGivesIdenticalStream)
{
  // Reproducibility is the whole point: without it a benchmark cannot be
  // repeated and a regression cannot be bisected.
  RandomSource a(42u, "imu");
  RandomSource b(42u, "imu");
  for (int i = 0; i < 1000; ++i) {
    EXPECT_DOUBLE_EQ(a.gaussian(), b.gaussian());
  }
}

TEST(RandomSource, DifferentStreamNamesDecorrelate)
{
  // Changing the LiDAR configuration must not shift the IMU's noise sequence,
  // otherwise "I only changed the LiDAR" silently changes every other result.
  RandomSource imu(42u, "imu");
  RandomSource lidar(42u, "lidar");
  int identical = 0;
  for (int i = 0; i < 200; ++i) {
    if (imu.gaussian() == lidar.gaussian()) {++identical;}
  }
  EXPECT_EQ(identical, 0);
}

TEST(RandomSource, ReseedRestartsTheStream)
{
  RandomSource r(7u, "enc");
  std::vector<double> first;
  for (int i = 0; i < 100; ++i) {first.push_back(r.gaussian());}
  r.reseed(7u, "enc");
  for (int i = 0; i < 100; ++i) {EXPECT_DOUBLE_EQ(r.gaussian(), first[i]);}
}

TEST(RandomSource, GaussianHasRequestedMomentsAndZeroSigmaIsExact)
{
  RandomSource r(1u, "x");
  std::vector<double> xs;
  xs.reserve(20000);
  for (int i = 0; i < 20000; ++i) {xs.push_back(r.gaussian(2.0, 0.5));}
  EXPECT_NEAR(mean_of(xs), 2.0, 0.02);
  EXPECT_NEAR(stdev_of(xs), 0.5, 0.02);

  // A non-positive sigma must be exact, so "disable this noise" is a config
  // change rather than a separate code path.
  for (int i = 0; i < 50; ++i) {
    EXPECT_DOUBLE_EQ(r.gaussian(3.0, 0.0), 3.0);
    EXPECT_DOUBLE_EQ(r.gaussian(3.0, -1.0), 3.0);
  }
}

TEST(RandomSource, BernoulliRespectsTheProbabilityAndItsEdges)
{
  RandomSource r(5u, "drop");
  for (int i = 0; i < 100; ++i) {
    EXPECT_FALSE(r.bernoulli(0.0));
    EXPECT_FALSE(r.bernoulli(-0.5));
    EXPECT_TRUE(r.bernoulli(1.0));
    EXPECT_TRUE(r.bernoulli(2.0));
  }
  int hits = 0;
  for (int i = 0; i < 20000; ++i) {
    if (r.bernoulli(0.3)) {++hits;}
  }
  EXPECT_NEAR(hits / 20000.0, 0.3, 0.02);
}

// ---------------------------------------------------------- GaussMarkovBias

TEST(GaussMarkovBias, StationaryVarianceMatchesTheSpecifiedSigma)
{
  // The exact OU update must reproduce its steady-state spread regardless of
  // the step size. A naive Euler discretisation does not, and the resulting
  // bias is then quietly rate-dependent.
  const double sigma_ss = 0.02;
  const double tau = 5.0;
  RandomSource rng(11u, "bias");
  GaussMarkovBias bias(sigma_ss, tau);
  bias.initialise(rng);

  std::vector<double> samples;
  samples.reserve(60000);
  for (int i = 0; i < 60000; ++i) {samples.push_back(bias.update(0.01, rng));}

  EXPECT_NEAR(mean_of(samples), 0.0, 0.01);
  EXPECT_NEAR(stdev_of(samples), sigma_ss, 0.004);
}

TEST(GaussMarkovBias, OneStepUpdateMatchesTheExactOrnsteinUhlenbeckSolution)
{
  // The sharpest available check, and the one that distinguishes the exact
  // discretisation from a naive Euler step.
  //
  // Conditioned on b_k, the exact solution gives
  //     E[b_{k+1}]   = b_k * exp(-dt/tau)
  //     Std[b_{k+1}] = sigma_ss * sqrt(1 - exp(-2*dt/tau))
  //
  // A naive step uses sigma_ss * sqrt(2*dt/tau) for the second term. The two
  // agree to first order, so a test run only at dt/tau << 1 cannot tell them
  // apart: at dt/tau = 0.001 the error is 0.1 %. The error reaches 52 % at
  // dt/tau = 1 and 102 % at dt/tau = 2, so the sweep below deliberately
  // includes those ratios.
  //
  // Measuring the ONE-STEP conditional distribution over independent trials
  // also avoids the mixing-time problem: a stationary-variance test at small
  // dt/tau needs an impractical number of correlated samples to be precise.
  const double sigma_ss = 0.02;
  const double b0 = 0.013;
  const int kTrials = 20000;

  struct Case { double tau; double dt; };
  for (const Case c : {Case{5.0, 0.5}, Case{1.0, 0.5}, Case{0.5, 0.5}, Case{0.5, 1.0}}) {
    std::vector<double> after;
    after.reserve(kTrials);
    for (int trial = 0; trial < kTrials; ++trial) {
      RandomSource rng(static_cast<std::uint32_t>(trial), "ou");
      GaussMarkovBias bias(sigma_ss, c.tau);
      bias.set(b0);
      after.push_back(bias.update(c.dt, rng));
    }
    const double decay = std::exp(-c.dt / c.tau);
    const double expected_mean = b0 * decay;
    const double expected_std = sigma_ss * std::sqrt(1.0 - decay * decay);

    EXPECT_NEAR(mean_of(after), expected_mean, 0.03 * sigma_ss)
      << "tau=" << c.tau << " dt=" << c.dt;
    EXPECT_NEAR(stdev_of(after), expected_std, 0.03 * expected_std)
      << "tau=" << c.tau << " dt=" << c.dt
      << " (a naive Euler step would give "
      << sigma_ss * std::sqrt(2.0 * c.dt / c.tau) << ")";
  }
}

TEST(GaussMarkovBias, StationaryVarianceIsIndependentOfStepSize)
{
  // The consequence of the above: whatever rate the sensor runs at, the bias
  // settles to the same spread. A rate-dependent bias would silently change
  // every result when a sensor rate is retuned.
  const double sigma_ss = 0.02;
  const double tau = 0.5;
  for (const double dt : {0.05, 0.25, 1.0}) {
    RandomSource rng(3u, "bias");
    GaussMarkovBias bias(sigma_ss, tau);
    bias.initialise(rng);
    std::vector<double> samples;
    const int n = static_cast<int>(2000.0 / dt);
    for (int i = 0; i < n; ++i) {samples.push_back(bias.update(dt, rng));}
    EXPECT_NEAR(stdev_of(samples), sigma_ss, 0.06 * sigma_ss) << "dt=" << dt;
  }
}

TEST(GaussMarkovBias, RandomWalkVarianceGrowsLinearlyInTime)
{
  // tau <= 0 selects a pure random walk, whose variance must grow as
  // sigma^2 * t. This is the model behind "encoder drift", and it is the
  // reason that fault is unbounded while white noise is not.
  const double sigma_rw = 0.01;
  const double dt = 0.01;
  const int steps = 1000;          // 10 s
  std::vector<double> finals;
  for (int trial = 0; trial < 400; ++trial) {
    RandomSource rng(static_cast<std::uint32_t>(trial), "rw");
    GaussMarkovBias bias(0.0, -1.0, sigma_rw);
    bias.initialise(rng);
    double v = 0.0;
    for (int i = 0; i < steps; ++i) {v = bias.update(dt, rng);}
    finals.push_back(v);
  }
  const double expected = sigma_rw * std::sqrt(steps * dt);   // sigma * sqrt(t)
  EXPECT_NEAR(stdev_of(finals), expected, 0.25 * expected);
}

TEST(GaussMarkovBias, ZeroOrNegativeDtDoesNotAdvanceTheProcess)
{
  RandomSource rng(1u, "b");
  GaussMarkovBias bias(0.01, 10.0);
  bias.initialise(rng);
  const double before = bias.value();
  EXPECT_DOUBLE_EQ(bias.update(0.0, rng), before);
  EXPECT_DOUBLE_EQ(bias.update(-1.0, rng), before);
}

TEST(GaussMarkovBias, InitialisesFromTheStationaryDistributionNotZero)
{
  // A sensor that powers on at a perfect zero flatters the estimator for the
  // first seconds of every run.
  std::vector<double> starts;
  for (int trial = 0; trial < 500; ++trial) {
    RandomSource rng(static_cast<std::uint32_t>(trial), "b");
    GaussMarkovBias bias(0.05, 100.0);
    bias.initialise(rng);
    starts.push_back(bias.value());
  }
  EXPECT_NEAR(stdev_of(starts), 0.05, 0.01);
}

// -------------------------------------------------------------- ChannelNoise

TEST(ChannelNoise, ScaleErrorIsProportionalToTheSignal)
{
  // No white noise, no bias: the only corruption left is the scale factor,
  // which must vanish at zero input and grow with the signal. This is why a
  // scale error looks like it "appears" only when the robot moves.
  RandomSource rng(9u, "c");
  ChannelNoise ch(0.0, 0.0, 0.0, 0.0, 0.05);
  EXPECT_NEAR(ch.apply(0.0, 0.01, rng), 0.0, 1e-12);
  EXPECT_NEAR(ch.apply(1.0, 0.01, rng), 1.05, 1e-12);
  EXPECT_NEAR(ch.apply(10.0, 0.01, rng), 10.5, 1e-12);
}

TEST(ChannelNoise, NoiseScaleMultipliesSigmaWithoutTouchingTheSpec)
{
  RandomSource rng(4u, "c");
  ChannelNoise ch(0.1, 0.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(ch.effective_sigma(), 0.1, 1e-12);

  ch.set_noise_scale(10.0);
  EXPECT_NEAR(ch.effective_sigma(), 1.0, 1e-12);

  std::vector<double> xs;
  for (int i = 0; i < 20000; ++i) {xs.push_back(ch.apply(0.0, 0.005, rng));}
  EXPECT_NEAR(stdev_of(xs), 1.0, 0.05);

  // Restoring the scale must restore the original specification exactly: the
  // fault must never overwrite the nominal sigma.
  ch.set_noise_scale(1.0);
  EXPECT_NEAR(ch.effective_sigma(), 0.1, 1e-12);
}

TEST(ChannelNoise, ZeroNoiseScaleSilencesTheWhiteNoiseOnly)
{
  RandomSource rng(4u, "c");
  ChannelNoise ch(0.5, 0.0, 0.0, 0.0, 0.0);
  ch.set_noise_scale(0.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(ch.apply(2.0, 0.01, rng), 2.0);
  }
}

TEST(ChannelNoise, WhiteNoiseIsZeroMeanSoItAveragesAwayButBiasDoesNot)
{
  // The distinction that drives the whole estimator design: averaging kills
  // white noise and does nothing to a bias.
  RandomSource rng(17u, "c");
  ChannelNoise white(0.2, 0.0, 0.0, 0.0, 0.0);
  std::vector<double> w;
  for (int i = 0; i < 40000; ++i) {w.push_back(white.apply(1.0, 0.005, rng));}
  EXPECT_NEAR(mean_of(w), 1.0, 0.01);

  RandomSource rng2(17u, "c");
  ChannelNoise biased(0.0, 0.3, 1e9, 0.0, 0.0);  // huge tau: effectively constant
  std::vector<double> b;
  for (int i = 0; i < 40000; ++i) {b.push_back(biased.apply(1.0, 0.005, rng2));}
  EXPECT_GT(std::fabs(mean_of(b) - 1.0), 0.02) << "a constant bias must survive averaging";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
