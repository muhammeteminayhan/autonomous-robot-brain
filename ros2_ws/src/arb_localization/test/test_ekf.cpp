// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_localization/ekf.hpp"

using arb_core::normalizeAngle;
using arb_core::Pose2D;
using arb_localization::ExtendedKalmanFilter;
using EKF = ExtendedKalmanFilter;

namespace
{

constexpr double kPi = 3.14159265358979323846;

EKF::StateVector make_state(double x, double y, double th,
                            double v, double w, double b = 0.0)
{
  EKF::StateVector s;
  s << x, y, th, v, w, b;
  return s;
}

/// A filter that starts perfectly known, so that any covariance growth comes
/// from the model rather than the initial guess.
EKF certain_filter(const EKF::Config & cfg, const EKF::StateVector & s)
{
  EKF f(cfg);
  f.initialise(s, EKF::StateMatrix::Zero());
  return f;
}

EKF::Config quiet_config()
{
  EKF::Config c;
  c.accel_sigma = 0.0;
  c.angular_accel_sigma = 0.0;
  c.gyro_bias_random_walk = 0.0;
  c.position_process_sigma = 0.0;
  c.heading_process_sigma = 0.0;
  return c;
}

}  // namespace

// ===================================================================
//  PREDICTION
// ===================================================================

TEST(EkfPredict, MatchesTheAnalyticArcSolution)
{
  // Constant (v, omega) from the origin traces a circle of radius v/omega:
  //     x = (v/w) sin(w t),   y = (v/w)(1 - cos(w t))
  const double v = 0.4, w = 0.7, T = 3.0;
  EKF f = certain_filter(quiet_config(), make_state(0, 0, 0, v, w));
  const double dt = 0.005;
  for (int i = 0; i < static_cast<int>(T / dt); ++i) {ASSERT_TRUE(f.predict(dt));}

  const double R = v / w;
  EXPECT_NEAR(f.state()(EKF::X), R * std::sin(w * T), 1e-9);
  EXPECT_NEAR(f.state()(EKF::Y), R * (1.0 - std::cos(w * T)), 1e-9);
  EXPECT_NEAR(f.state()(EKF::THETA), normalizeAngle(w * T), 1e-9);
}

TEST(EkfPredict, StraightLineWhenOmegaIsZero)
{
  EKF f = certain_filter(quiet_config(), make_state(0, 0, 0.3, 0.5, 0.0));
  for (int i = 0; i < 200; ++i) {ASSERT_TRUE(f.predict(0.01));}
  EXPECT_NEAR(f.state()(EKF::X), 0.5 * 2.0 * std::cos(0.3), 1e-9);
  EXPECT_NEAR(f.state()(EKF::Y), 0.5 * 2.0 * std::sin(0.3), 1e-9);
  EXPECT_NEAR(f.state()(EKF::THETA), 0.3, 1e-12);
}

TEST(EkfPredict, RejectsNonPositiveNonFiniteAndOversizedIntervals)
{
  EKF f = certain_filter(quiet_config(), make_state(0, 0, 0, 1.0, 0.0));
  EXPECT_FALSE(f.predict(0.0));
  EXPECT_FALSE(f.predict(-0.01));
  EXPECT_FALSE(f.predict(std::nan("")));
  EXPECT_FALSE(f.predict(1e9));
  EXPECT_FALSE(f.predict(f.config().max_predict_dt * 2.0));
  EXPECT_NEAR(f.state()(EKF::X), 0.0, 1e-15) << "a rejected step must not move";
}

TEST(EkfPredict, JacobianMatchesACentralFiniteDifference)
{
  // The sharpest check available on the prediction. The covariance is
  // propagated with F, so an F that does not differentiate f leaves the
  // filter describing a different system from the one it is running.
  //
  // d(x')/d(omega) in particular needs the chain rule through BOTH the sinc
  // factor and the midpoint heading; dropping either term is invisible in the
  // state and wrong in the covariance.
  const double dt = 0.05;
  const EKF::StateVector base = make_state(1.2, -0.4, 0.7, 0.45, 0.9, 0.01);
  const double eps = 1e-7;

  // Extract F by propagating a tiny covariance: with P = eps^2 * e_j e_j^T,
  // F P F^T has (i,i) = eps^2 * F(i,j)^2, which loses the sign. Instead
  // reconstruct F directly from finite differences of the state update and
  // compare against the covariance the filter produces for a unit P.
  EKF::StateMatrix F_numeric = EKF::StateMatrix::Zero();
  for (int j = 0; j < EKF::kStateDim; ++j) {
    EKF::StateVector plus = base, minus = base;
    plus(j) += eps;
    minus(j) -= eps;
    EKF fp = certain_filter(quiet_config(), plus);
    EKF fm = certain_filter(quiet_config(), minus);
    ASSERT_TRUE(fp.predict(dt));
    ASSERT_TRUE(fm.predict(dt));
    for (int i = 0; i < EKF::kStateDim; ++i) {
      double d = fp.state()(i) - fm.state()(i);
      if (i == EKF::THETA) {d = normalizeAngle(d);}
      F_numeric(i, j) = d / (2 * eps);
    }
  }

  // The filter's own F is recovered by propagating the identity covariance
  // with zero process noise: P' = F * I * F^T = F F^T.
  EKF f(quiet_config());
  f.initialise(base, EKF::StateMatrix::Identity());
  ASSERT_TRUE(f.predict(dt));
  const EKF::StateMatrix FFt = f.covariance();

  EXPECT_LT((FFt - F_numeric * F_numeric.transpose()).cwiseAbs().maxCoeff(), 1e-6)
    << "analytic F:\n" << FFt << "\nnumeric F F^T:\n"
    << F_numeric * F_numeric.transpose();
}

TEST(EkfPredict, UncertaintyGrowsAndNeverShrinks)
{
  // Prediction adds information to nothing. A covariance that shrinks during
  // prediction is a filter inventing knowledge.
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0.4, 0.3), EKF::StateMatrix::Zero());
  double previous = 0.0;
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(f.predict(0.02));
    const double trace = f.covariance().trace();
    EXPECT_GE(trace, previous - 1e-15) << "step " << i;
    previous = trace;
  }
  EXPECT_GT(previous, 0.0);
}

TEST(EkfPredict, ProcessNoiseAccumulationIsIndependentOfTheStepSize)
{
  // The property every process noise term must have, and the one an earlier
  // version of this filter did not.
  //
  // A velocity driven by white acceleration is a random walk: its variance
  // accumulates as q*T regardless of how the interval is chopped up. Writing
  // the per-step variance as (sigma*dt)^2 gives sigma^2*T*dt instead, so the
  // same filter run at 200 Hz comes out four times more confident than at
  // 50 Hz -- and the chi-square gate then rejects real accelerations as
  // outliers.
  //
  // Every term here is checked at three step sizes spanning a factor of ten.
  EKF::Config c;
  c.accel_sigma = 1.0;
  c.angular_accel_sigma = 2.0;
  c.gyro_bias_random_walk = 0.01;
  c.position_process_sigma = 0.05;
  c.heading_process_sigma = 0.03;

  const double T = 2.0;
  std::vector<double> v_var, w_var, b_var, x_var;
  for (const double dt : {0.05, 0.01, 0.005}) {
    EKF f(c);
    f.initialise(make_state(0, 0, 0, 0, 0), EKF::StateMatrix::Zero());
    for (int i = 0; i < static_cast<int>(T / dt); ++i) {ASSERT_TRUE(f.predict(dt));}
    v_var.push_back(f.covariance()(EKF::V, EKF::V));
    w_var.push_back(f.covariance()(EKF::OMEGA, EKF::OMEGA));
    b_var.push_back(f.covariance()(EKF::GYRO_BIAS, EKF::GYRO_BIAS));
    x_var.push_back(f.covariance()(EKF::X, EKF::X));
  }
  for (std::size_t i = 1; i < v_var.size(); ++i) {
    EXPECT_NEAR(v_var[i] / v_var[0], 1.0, 1e-9) << "velocity, step " << i;
    EXPECT_NEAR(w_var[i] / w_var[0], 1.0, 1e-9) << "angular rate, step " << i;
    EXPECT_NEAR(b_var[i] / b_var[0], 1.0, 1e-9) << "gyro bias, step " << i;
  }
  // The pose variance is not exactly step-independent because it also picks up
  // the propagated velocity uncertainty through F, but it must stay the same
  // order rather than scaling with dt.
  for (std::size_t i = 1; i < x_var.size(); ++i) {
    EXPECT_NEAR(x_var[i] / x_var[0], 1.0, 0.05) << "position, step " << i;
  }

  // And the absolute values must be what the parameters claim: after T seconds
  // with no measurement, the 1-sigma velocity uncertainty is accel_sigma*sqrt(T).
  EXPECT_NEAR(std::sqrt(v_var[0]), c.accel_sigma * std::sqrt(T), 1e-9);
  EXPECT_NEAR(std::sqrt(b_var[0]), c.gyro_bias_random_walk * std::sqrt(T), 1e-12);
}

TEST(EkfPredict, AcceptsAccelerationWithinThePlatformLimits)
{
  // The regression the scaling bug caused. A robot limited to 1 m/s^2 changes
  // its velocity by 0.02 m/s between 50 Hz odometry updates. The filter must
  // find that unremarkable; if the gate rejects it, the filter cannot follow
  // its own robot decelerating.
  EKF::Config c;                          // production defaults
  EKF f(c);
  EKF::StateMatrix P = EKF::StateMatrix::Zero();
  P(EKF::V, EKF::V) = 1e-6;
  f.initialise(make_state(0, 0, 0, 0.25, 0.0), P);

  double v = 0.25;
  int rejected = 0;
  for (int i = 0; i < 13; ++i) {         // 0.25 m/s to 0 at 1 m/s^2
    for (int k = 0; k < 4; ++k) {f.predict(0.005);}    // IMU-rate prediction
    v = std::max(0.0, v - 0.02);
    if (!f.updateWheelTwist(v, 0.0, 0.002, 0.002).accepted) {++rejected;}
  }
  EXPECT_EQ(rejected, 0) << "the gate rejected a physically possible deceleration";
  EXPECT_NEAR(f.linearVelocity(), 0.0, 0.01)
    << "and the filter must actually follow it down to zero";
}

TEST(EkfPredict, StandingStillStillAccumulatesPoseUncertainty)
{
  // A stationary robot is not a known robot: it can be pushed, bumped, or
  // slip. Without the direct pose process noise the covariance would freeze
  // at zero and the filter would refuse every later correction.
  EKF::Config c;
  c.accel_sigma = 0.0;
  c.angular_accel_sigma = 0.0;
  EKF f = certain_filter(c, make_state(0, 0, 0, 0.0, 0.0));
  for (int i = 0; i < 100; ++i) {ASSERT_TRUE(f.predict(0.02));}
  EXPECT_GT(f.covariance()(EKF::X, EKF::X), 0.0);
  EXPECT_GT(f.covariance()(EKF::THETA, EKF::THETA), 0.0);
}

// ===================================================================
//  UPDATE
// ===================================================================

TEST(EkfUpdate, PerfectMeasurementOfAPerfectlyUnknownStateAdoptsIt)
{
  // With infinite prior uncertainty in v and a noiseless measurement, the
  // Kalman gain is 1 and the posterior is the measurement.
  EKF f(quiet_config());
  EKF::StateMatrix P = EKF::StateMatrix::Zero();
  P(EKF::V, EKF::V) = 1e6;
  P(EKF::OMEGA, EKF::OMEGA) = 1e6;
  f.initialise(make_state(0, 0, 0, 0, 0), P);

  const auto r = f.updateWheelTwist(0.42, -0.17, 1e-4, 1e-4);
  ASSERT_TRUE(r.accepted);
  EXPECT_NEAR(f.linearVelocity(), 0.42, 1e-6);
  EXPECT_NEAR(f.angularVelocity(), -0.17, 1e-6);
}

TEST(EkfUpdate, AWorthlessMeasurementChangesNothing)
{
  // The mirror image: an enormous R means the measurement carries no
  // information and the gain must vanish.
  EKF f(quiet_config());
  EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.01;
  f.initialise(make_state(0, 0, 0, 0.3, 0.1), P);
  const double before_v = f.linearVelocity();
  const double before_var = f.covariance()(EKF::V, EKF::V);

  EKF::Config c = quiet_config();
  c.enable_gating = false;      // a hugely inconsistent reading would be gated
  EKF g(c);
  g.initialise(make_state(0, 0, 0, 0.3, 0.1), P);
  const auto r = g.updateWheelTwist(99.0, 99.0, 1e6, 1e6);
  ASSERT_TRUE(r.accepted);
  EXPECT_NEAR(g.linearVelocity(), before_v, 1e-6);
  EXPECT_NEAR(g.covariance()(EKF::V, EKF::V), before_var, 1e-6);
}

TEST(EkfUpdate, AnUpdateNeverIncreasesUncertainty)
{
  // A fundamental property of the Kalman update: fusing a measurement cannot
  // make the estimate less certain. If it does, the implementation is wrong.
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0.4, 0.2), EKF::StateMatrix::Identity() * 0.05);
  const double before = f.covariance().trace();
  const auto r = f.updateWheelTwist(0.41, 0.19, 0.02, 0.03);
  ASSERT_TRUE(r.accepted);
  EXPECT_LE(f.covariance().trace(), before + 1e-12);
}

TEST(EkfUpdate, CovarianceStaysSymmetricAndPositiveDefinite)
{
  // Joseph form exists for exactly this. Hundreds of alternating predictions
  // and updates must not erode symmetry or definiteness.
  std::mt19937 rng(7);
  std::normal_distribution<double> n(0.0, 1.0);
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0, 0), EKF::StateMatrix::Identity() * 0.1);

  for (int i = 0; i < 2000; ++i) {
    f.predict(0.02);
    f.updateWheelTwist(0.4 + 0.02 * n(rng), 0.3 + 0.02 * n(rng), 0.02, 0.02);
    f.updateGyro(0.3 + 0.01 * n(rng), 0.01);

    const EKF::StateMatrix P = f.covariance();
    ASSERT_EQ((P - P.transpose()).cwiseAbs().maxCoeff(), 0.0) << "step " << i;
    const Eigen::SelfAdjointEigenSolver<EKF::StateMatrix> es(P);
    ASSERT_GT(es.eigenvalues().minCoeff(), 0.0)
      << "step " << i << " eigenvalues " << es.eigenvalues().transpose();
  }
}

TEST(EkfUpdate, JosephFormSurvivesAnIllConditionedUpdate)
{
  // The scenario Joseph form exists for, and the only one in which it differs
  // from the textbook update at double precision.
  //
  // A near-infinite prior (1e6) meeting a near-perfect measurement (R = 1e-14)
  // gives a Kalman gain of essentially 1, so (I - K H) is about 1e-20 in the
  // measured directions. The textbook form then computes
  //
  //     P <- (I - K H) P
  //
  // as 1e-20 * 1e6, while the rounding error in forming (I - K H) is already
  // 1e6 * eps ~ 1e-10: the answer is entirely noise, and roughly half the time
  // that noise is NEGATIVE. A covariance with a negative eigenvalue reports an
  // imaginary standard deviation, and the next Cholesky factorisation fails.
  //
  // Joseph form
  //
  //     P <- (I - K H) P (I - K H)^T + K R K^T
  //
  // is a sum of two positive semi-definite terms, so no amount of rounding can
  // make it indefinite. That is the entire argument, and it is why the extra
  // matrix product is worth paying for.
  // The assertion is a physical bound rather than a tolerance, which is what
  // makes it sharp. When the measurement is vastly more precise than the
  // prior, the posterior variance of the measured state must come out at
  // essentially R: the measurement has told the filter almost everything.
  // It certainly cannot be larger than R, and it certainly cannot be negative.
  //
  // With prior 1e6 and R = 1e-14 the textbook form lands around 2e-10, four
  // orders of magnitude above R and of arbitrary sign. Asserting on the
  // eigenvalues alone is too forgiving: 2e-10 is still "positive enough"
  // relative to a 1e6 prior, and the defect hides.
  const double sigma = 1e-7;
  const double R = sigma * sigma;          // 1e-14

  EKF::Config c = quiet_config();
  c.enable_gating = false;
  EKF f(c);

  for (int i = 0; i < 50; ++i) {
    f.initialise(make_state(0, 0, 0, 0, 0), EKF::StateMatrix::Identity() * 1e6);
    ASSERT_TRUE(f.updateWheelTwist(0.4, 0.2, sigma, sigma).accepted);

    const double var_v = f.covariance()(EKF::V, EKF::V);
    const double var_w = f.covariance()(EKF::OMEGA, EKF::OMEGA);
    ASSERT_GT(var_v, 0.0) << "negative variance at update " << i;
    ASSERT_GT(var_w, 0.0) << "negative variance at update " << i;
    ASSERT_LT(var_v, 10.0 * R)
      << "posterior variance " << var_v << " exceeds the measurement variance "
      << R << " by " << var_v / R << "x at update " << i;
    ASSERT_LT(var_w, 10.0 * R) << "update " << i;

    const Eigen::SelfAdjointEigenSolver<EKF::StateMatrix> es(f.covariance());
    ASSERT_GE(es.eigenvalues().minCoeff(), 0.0)
      << "covariance went indefinite at update " << i
      << "\neigenvalues: " << es.eigenvalues().transpose();
  }
}

TEST(EkfUpdate, AbsolutePoseHeadingResidualIsWrapped)
{
  // The estimate at +179 deg and a measurement at -179 deg differ by 2 deg,
  // not 358. An unwrapped residual hands the filter a fictitious enormous
  // error and it diverges in a single step.
  EKF::Config c = quiet_config();
  c.enable_gating = false;
  EKF f(c);
  EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.5;
  f.initialise(make_state(0, 0, arb_core::degToRad(179.0), 0, 0), P);

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-6;
  const auto r = f.updateAbsolutePose(
    Pose2D(0.0, 0.0, arb_core::degToRad(-179.0)), R);
  ASSERT_TRUE(r.accepted);
  EXPECT_NEAR(arb_core::radToDeg(r.innovation(2)), 2.0, 1e-6);
  EXPECT_NEAR(std::abs(arb_core::radToDeg(f.pose().theta)), 179.0, 0.1);
}

// ===================================================================
//  GYRO BIAS OBSERVABILITY  -- the reason the state vector has six entries
// ===================================================================

TEST(EkfBias, IsEstimatedFromTheDisagreementBetweenGyroAndEncoders)
{
  // Two independent measurements of the same rotation rate. The persistent
  // gap between them IS the bias, and the filter must converge to it.
  //
  // With only a gyro this state would be unobservable: any (omega, b) pair
  // summing to the reading explains it equally well.
  const double true_bias = 0.03;      // rad/s
  const double true_omega = 0.5;

  EKF::Config c;
  EKF f(c);

  // INITIALISATION MATTERS, and getting it wrong here taught that the hard
  // way. An earlier version started omega at 0 while the truth was 0.5, and
  // gave the bias a 0.1 rad/s prior -- five degrees per second, absurd for any
  // MEMS gyro. On the very first cycle the encoder reading was 5 sigma away
  // and the chi-square gate REJECTED it, while the gyro reading looked
  // plausible because the huge bias prior could explain it. The bias absorbed
  // the entire 0.5 rad/s and settled at 0.475, after which every encoder
  // measurement stayed implausible and the filter was locked into a wrong but
  // perfectly self-consistent state.
  //
  // The lesson is general: a gate protects a filter that is roughly right and
  // traps one that starts wrong. The real node therefore initialises omega
  // from the first odometry message, and gives the bias a prior matching the
  // datasheet rather than an arbitrarily large one.
  EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.01;
  P(EKF::GYRO_BIAS, EKF::GYRO_BIAS) = 1e-4;   // sigma = 0.01 rad/s, generous
  f.initialise(make_state(0, 0, 0, 0.0, true_omega, 0.0), P);

  for (int i = 0; i < 3000; ++i) {
    f.predict(0.02);
    f.updateWheelTwist(0.0, true_omega, 0.01, 0.01);
    f.updateGyro(true_omega + true_bias, 0.005);
  }

  EXPECT_NEAR(f.gyroBias(), true_bias, 0.004)
    << "the filter must attribute the gap to the bias";
  EXPECT_NEAR(f.angularVelocity(), true_omega, 0.01)
    << "and not to the rotation rate";
  EXPECT_LT(std::sqrt(f.covariance()(EKF::GYRO_BIAS, EKF::GYRO_BIAS)), 0.02)
    << "the bias estimate must also become confident";
}

TEST(EkfBias, StaysPutWhenTheGyroAgreesWithTheEncoders)
{
  EKF::Config c;
  EKF f(c);
  EKF::StateMatrix P = EKF::StateMatrix::Identity() * 0.01;
  P(EKF::GYRO_BIAS, EKF::GYRO_BIAS) = 1e-4;
  f.initialise(make_state(0, 0, 0, 0, 0.4, 0), P);
  for (int i = 0; i < 2000; ++i) {
    f.predict(0.02);
    f.updateWheelTwist(0.0, 0.4, 0.01, 0.01);
    f.updateGyro(0.4, 0.005);
  }
  EXPECT_NEAR(f.gyroBias(), 0.0, 0.003);
}

// ===================================================================
//  INNOVATION GATING
// ===================================================================

TEST(EkfGating, RejectsAnOutlierAndLeavesTheStateUntouched)
{
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Identity() * 1e-4);
  const EKF::StateVector before = f.state();

  const auto r = f.updateWheelTwist(50.0, 0.0, 0.01, 0.01);
  EXPECT_FALSE(r.accepted);
  EXPECT_GT(r.nis, r.gate);
  EXPECT_LT((f.state() - before).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_EQ(f.consecutiveRejections(), 1);
}

TEST(EkfGating, AcceptsAConsistentMeasurement)
{
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Identity() * 0.01);
  const auto r = f.updateWheelTwist(0.405, 0.002, 0.02, 0.02);
  EXPECT_TRUE(r.accepted);
  EXPECT_LT(r.nis, r.gate);
  EXPECT_EQ(f.consecutiveRejections(), 0);
}

TEST(EkfGating, OpensAfterATooLongRunOfRejectionsSoTheFilterCanRecover)
{
  // A gate that never yields is a trap: a genuinely lost filter finds every
  // correct measurement implausible and rejects it forever.
  EKF::Config c;
  c.max_consecutive_rejections = 5;
  EKF f(c);
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Identity() * 1e-6);

  int accepted = 0;
  for (int i = 0; i < 12; ++i) {
    // The truth is far from the estimate and the filter is (wrongly) certain.
    if (f.updateWheelTwist(5.0, 0.0, 0.01, 0.01).accepted) {++accepted;}
  }
  EXPECT_GT(accepted, 0) << "the filter must eventually let one through";
  EXPECT_GT(f.linearVelocity(), 0.4) << "and actually move towards it";
}

TEST(EkfGating, CanBeDisabled)
{
  EKF::Config c;
  c.enable_gating = false;
  EKF f(c);
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Identity() * 1e-4);
  const auto r = f.updateWheelTwist(50.0, 0.0, 0.01, 0.01);
  EXPECT_TRUE(r.accepted);
  EXPECT_GT(r.nis, r.gate) << "still reported, just not acted on";
}

// ===================================================================
//  CONSISTENCY  -- does the filter's own uncertainty mean anything?
// ===================================================================

TEST(EkfConsistency, AverageNisMatchesTheMeasurementDimension)
{
  // The definitive test of filter tuning. If the filter's assumptions are
  // right, the normalised innovation squared is chi-square distributed with
  // dim(z) degrees of freedom, whose MEAN is dim(z).
  //
  //   average NIS >> dim  ->  overconfident: R or Q too small, and the filter
  //                           will reject good data and eventually diverge
  //   average NIS << dim  ->  conservative: it is throwing away information
  //
  // This is measurable online, without ground truth, which is what makes it
  // the standard field diagnostic.
  std::mt19937 rng(42);
  const double sigma_v = 0.02, sigma_w = 0.03;
  std::normal_distribution<double> nv(0.0, sigma_v), nw(0.0, sigma_w);

  // For the NIS to have its theoretical distribution, the filter's model must
  // match the process that generated the data. The truth below is exactly
  // constant velocity, so the process noise is set small to match; leaving it
  // at the default (which allows for real acceleration) inflates S, and the
  // measured average NIS drops to 1.64 -- correctly reporting a conservative
  // filter rather than a broken one, but not what this test is checking.
  EKF::Config c;
  c.accel_sigma = 0.01;
  c.angular_accel_sigma = 0.01;
  c.position_process_sigma = 0.0;
  c.heading_process_sigma = 0.0;
  EKF f(c);
  f.initialise(make_state(0, 0, 0, 0.4, 0.2), EKF::StateMatrix::Identity() * 0.01);
  f.updateWheelTwist(0.4, 0.2, sigma_v, sigma_w);   // settle

  double nis_sum = 0.0;
  int n = 0;
  for (int i = 0; i < 20000; ++i) {
    f.predict(0.02);
    const auto r = f.updateWheelTwist(0.4 + nv(rng), 0.2 + nw(rng),
                                      sigma_v, sigma_w);
    if (i > 200 && r.accepted) {
      nis_sum += r.nis;
      ++n;
    }
  }
  const double average = nis_sum / n;
  EXPECT_NEAR(average, 2.0, 0.15)
    << "average NIS " << average << " over " << n
    << " accepted updates; the measurement has 2 degrees of freedom";
}

TEST(EkfConsistency, PositionErrorStaysInsideTheReportedCovariance)
{
  // Monte Carlo: run the filter against a simulated truth and check that the
  // error it actually makes is consistent with the uncertainty it claims.
  std::mt19937 rng(11);
  const double sigma_v = 0.01, sigma_w = 0.02;
  std::normal_distribution<double> nv(0.0, sigma_v), nw(0.0, sigma_w);

  const double true_v = 0.35, true_w = 0.25;
  EKF::Config c;
  c.accel_sigma = 0.2;
  c.angular_accel_sigma = 0.4;
  EKF f(c);
  f.initialise(make_state(0, 0, 0, true_v, true_w),
               EKF::StateMatrix::Identity() * 1e-6);

  double tx = 0.0, ty = 0.0, tth = 0.0;
  const double dt = 0.02;
  int inside = 0, total = 0;
  for (int i = 0; i < 2000; ++i) {
    // Truth advances with the same exact arc update.
    const double half = 0.5 * true_w * dt;
    const double k = EKF::sinc(half);
    tx += true_v * dt * k * std::cos(tth + half);
    ty += true_v * dt * k * std::sin(tth + half);
    tth = normalizeAngle(tth + true_w * dt);

    f.predict(dt);
    f.updateWheelTwist(true_v + nv(rng), true_w + nw(rng), sigma_v, sigma_w);

    const double ex = f.state()(EKF::X) - tx;
    const double ey = f.state()(EKF::Y) - ty;
    const double err = std::hypot(ex, ey);
    const double sigma = std::sqrt(
      f.covariance()(EKF::X, EKF::X) + f.covariance()(EKF::Y, EKF::Y));
    if (err <= 3.0 * sigma) {++inside;}
    ++total;
  }
  EXPECT_GT(static_cast<double>(inside) / total, 0.95)
    << inside << "/" << total << " samples inside the reported 3-sigma";
}

// ===================================================================
//  ROBUSTNESS
// ===================================================================

TEST(EkfRobustness, SurvivesAMeasurementOutageAndReConverges)
{
  // Phase 10 Case E in miniature. With no measurements the filter must keep
  // PREDICTING, let the covariance grow, and then re-converge when data
  // returns. Freezing, crashing, or quietly reusing the last measurement all
  // fail here.
  EKF f(EKF::Config{});
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Identity() * 1e-4);
  for (int i = 0; i < 100; ++i) {
    f.predict(0.02);
    f.updateWheelTwist(0.4, 0.0, 0.02, 0.02);
  }
  const double settled = f.covariance()(EKF::V, EKF::V);

  for (int i = 0; i < 500; ++i) {ASSERT_TRUE(f.predict(0.02));}   // 10 s outage
  const double during = f.covariance()(EKF::V, EKF::V);
  EXPECT_GT(during, settled * 10.0) << "uncertainty must grow during an outage";
  EXPECT_TRUE(std::isfinite(f.state()(EKF::X)));

  for (int i = 0; i < 200; ++i) {
    f.predict(0.02);
    f.updateWheelTwist(0.4, 0.0, 0.02, 0.02);
  }
  EXPECT_LT(f.covariance()(EKF::V, EKF::V), during * 0.1) << "and shrink again";
  EXPECT_NEAR(f.linearVelocity(), 0.4, 0.02);
}

TEST(EkfRobustness, HeadingStaysWrappedThroughManyRevolutions)
{
  EKF f = certain_filter(quiet_config(), make_state(0, 0, 0, 0.0, 2.0));
  for (int i = 0; i < 2000; ++i) {
    ASSERT_TRUE(f.predict(0.02));
    ASSERT_GE(f.state()(EKF::THETA), -kPi);
    ASSERT_LT(f.state()(EKF::THETA), kPi);
  }
}

TEST(EkfRobustness, ASingularInnovationCovarianceIsRejectedNotDivided)
{
  // R = 0 with P = 0 makes S singular. The filter must refuse rather than
  // produce an enormous gain from a numerically invented inverse.
  EKF::Config c = quiet_config();
  EKF f(c);
  f.initialise(make_state(0, 0, 0, 0.4, 0.0), EKF::StateMatrix::Zero());
  const auto r = f.updateWheelTwist(0.5, 0.0, 0.0, 0.0);
  EXPECT_FALSE(r.accepted);
  EXPECT_TRUE(std::isfinite(f.state()(EKF::V)));
  EXPECT_NEAR(f.linearVelocity(), 0.4, 1e-15);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
