// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_localization/differential_drive_odometry.hpp"

using arb_core::normalizeAngle;
using arb_core::Pose2D;
using arb_localization::DifferentialDriveOdometry;
using arb_localization::OdometryConfig;
using arb_localization::OdometryReject;

namespace
{

constexpr double kPi = 3.14159265358979323846;

OdometryConfig config()
{
  OdometryConfig c;
  c.wheel_radius_m = 0.06;
  c.wheel_separation_m = 0.35;
  return c;
}

/// Drive the model by commanding a body twist, converting it to wheel angles
/// with the INVERSE kinematics, and feeding the resulting wheel positions in.
/// The test therefore never touches the forward kinematics it is checking, so
/// an error in either direction shows up as a mismatch.
///
/// TRUTH AND BELIEF ARE SEPARATE CONFIGURATIONS, deliberately.
///
/// `truth` is the physical robot: it decides how much the wheels actually turn
/// for a commanded twist. `belief` is what the odometry has been told the robot
/// is. Using one config for both is a subtle and complete trap -- a "wrong"
/// wheel radius would then be used to generate the wheel angles AND to
/// interpret them, cancelling exactly and making every calibration-error test
/// silently vacuous. This test file made that mistake first.
class Driver
{
public:
  explicit Driver(const OdometryConfig & c)
  : Driver(c, c) {}

  Driver(const OdometryConfig & truth, const OdometryConfig & belief)
  : truth_(truth), odom_(belief) {}

  /// Command (v, omega) for `duration` seconds in steps of `dt`.
  ///
  /// `duration` is rounded to a whole number of steps; use `time()` rather
  /// than `duration` when asserting against an analytic solution, or pick a
  /// dt that divides the duration exactly.
  void drive(double v, double omega, double duration, double dt)
  {
    if (!started_) {
      odom_.update(left_rad_, right_rad_, t_);
      started_ = true;
    }
    const int steps = static_cast<int>(std::round(duration / dt));
    for (int i = 0; i < steps; ++i) {
      // v_r = v + omega*L/2, v_l = v - omega*L/2, then omega_wheel = v_wheel/r
      const double v_r = v + omega * truth_.wheel_separation_m / 2.0;
      const double v_l = v - omega * truth_.wheel_separation_m / 2.0;
      right_rad_ += (v_r / truth_.wheel_radius_m) * dt;
      left_rad_ += (v_l / truth_.wheel_radius_m) * dt;
      t_ += dt;
      odom_.update(left_rad_, right_rad_, t_);
    }
  }

  DifferentialDriveOdometry & odom() {return odom_;}
  double time() const {return t_;}

private:
  OdometryConfig truth_;
  DifferentialDriveOdometry odom_;
  double left_rad_{0.0};
  double right_rad_{0.0};
  double t_{0.0};
  bool started_{false};
};

}  // namespace

// ------------------------------------------------------------------ sinc

TEST(Sinc, IsOneAtZeroAndMatchesTheQuotientAwayFromIt)
{
  EXPECT_DOUBLE_EQ(DifferentialDriveOdometry::sinc(0.0), 1.0);
  for (const double u : {1e-12, 1e-8, 1e-5, 1e-3}) {
    EXPECT_NEAR(DifferentialDriveOdometry::sinc(u), 1.0, 1e-6)
      << "sinc must approach 1 smoothly at u=" << u;
  }
  for (const double u : {0.01, 0.5, 1.0, 2.0, 3.0}) {
    EXPECT_NEAR(DifferentialDriveOdometry::sinc(u), std::sin(u) / u, 1e-12)
      << "u=" << u;
  }
  EXPECT_NEAR(DifferentialDriveOdometry::sinc(kPi), 0.0, 1e-15);
}

TEST(SincDerivative, MatchesAFiniteDifferenceOfSinc)
{
  // The integration carries a sinc factor, so the Jacobian must differentiate
  // it. A wrong sinc derivative is invisible in the pose and shows up only as
  // a slightly misshapen covariance ellipse.
  const double eps = 1e-6;
  for (const double u : {-2.0, -0.5, -1e-2, 0.0, 1e-2, 0.3, 1.0, 2.5}) {
    const double fd = (DifferentialDriveOdometry::sinc(u + eps) -
                       DifferentialDriveOdometry::sinc(u - eps)) / (2 * eps);
    EXPECT_NEAR(DifferentialDriveOdometry::sincDerivative(u), fd, 1e-7)
      << "u=" << u;
  }
  EXPECT_DOUBLE_EQ(DifferentialDriveOdometry::sincDerivative(0.0), 0.0);
}

TEST(Sinc, IsEvenAndBoundedByOne)
{
  for (int i = 1; i <= 300; ++i) {
    const double u = i * 0.031;
    EXPECT_NEAR(DifferentialDriveOdometry::sinc(u),
                DifferentialDriveOdometry::sinc(-u), 1e-15);
    EXPECT_LE(std::abs(DifferentialDriveOdometry::sinc(u)), 1.0 + 1e-15)
      << "the chord can never be longer than the arc";
  }
}

// ---------------------------------------------------------- forward kinematics

TEST(Odometry, StraightLineTravelsTheWheelArcExactly)
{
  // Both wheels the same: pure translation, no rotation. The distance must be
  // exactly the wheel arc length, since chord == arc for a straight line.
  Driver d(config());
  d.drive(0.4, 0.0, 5.0, 0.02);
  const auto & p = d.odom().pose();
  EXPECT_NEAR(p.x, 2.0, 1e-9);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.theta, 0.0, 1e-12);
  EXPECT_NEAR(d.odom().distance_travelled_m(), 2.0, 1e-9);
}

TEST(Odometry, SpinInPlaceDoesNotTranslate)
{
  // Equal and opposite wheels: pure rotation about the midpoint. Any
  // translation here means the kinematics are wrong.
  Driver d(config());
  // dt is chosen to divide the duration exactly. Driver rounds to whole steps,
  // so an arbitrary dt drives for slightly less than the requested time and
  // the analytic comparison misses by that difference -- which is a test bug,
  // not an integration error, and worth not confusing with one.
  constexpr int kSteps = 1000;
  d.drive(0.0, 1.0, kPi / 2.0, (kPi / 2.0) / kSteps);
  const auto & p = d.odom().pose();
  EXPECT_NEAR(p.x, 0.0, 1e-12);
  EXPECT_NEAR(p.y, 0.0, 1e-12);
  EXPECT_NEAR(p.theta, kPi / 2.0, 1e-9);
}

TEST(Odometry, InverseThenForwardKinematicsRoundTrip)
{
  // The Driver converts (v, omega) into wheel rates; the odometry converts
  // them back. The pair must be an identity.
  for (const double v : {-0.3, 0.0, 0.25, 0.7}) {
    for (const double w : {-1.2, -0.3, 0.0, 0.5, 1.5}) {
      Driver d(config());
      d.drive(v, w, 1.0, 0.01);
      const auto & step = d.odom().last_step();
      ASSERT_TRUE(step.valid);
      EXPECT_NEAR(step.linear_velocity, v, 1e-9) << "v=" << v << " w=" << w;
      EXPECT_NEAR(step.angular_velocity, w, 1e-9) << "v=" << v << " w=" << w;
    }
  }
}

TEST(Odometry, OnlyTheWheelDifferenceProducesRotation)
{
  // omega = r*(omega_r - omega_l)/L. Adding the same amount to both wheels
  // must change the distance and leave the heading untouched.
  OdometryConfig c = config();
  DifferentialDriveOdometry a(c);
  DifferentialDriveOdometry b(c);
  a.update(0.0, 0.0, 0.0);
  b.update(0.0, 0.0, 0.0);
  // Increments must stay under max_wheel_step_rad, or the plausibility guard
  // rejects them and both poses stay at the origin -- which is the guard
  // working, not the kinematics failing.
  a.update(0.3, 0.9, 0.1);
  b.update(0.3 + 1.0, 0.9 + 1.0, 0.1);
  EXPECT_NEAR(a.pose().theta, b.pose().theta, 1e-12);
  EXPECT_GT(b.pose().x, a.pose().x);
}

// ------------------------------------------------- exact arc vs naive Euler

TEST(Odometry, CircularArcClosesOnItself)
{
  // The decisive accuracy test. Driving a full circle must return the robot to
  // its starting point. Forward Euler cuts every corner and spirals inwards,
  // so the closure error is a direct measure of the integration scheme.
  //
  // With r = 0.5 m and 200 steps per revolution, the per-step heading change
  // is 1.8 degrees. Euler would leave a closure error of roughly
  //   R * (1 - sinc(dtheta/2)) * 2*pi/dtheta ~ 2.5 mm.
  // The exact arc update closes to numerical precision.
  const double radius = 0.5;
  const double v = 0.5;
  const double omega = v / radius;
  const double revolution = 2.0 * kPi / omega;

  Driver d(config());
  d.drive(v, omega, revolution, revolution / 200.0);

  const auto & p = d.odom().pose();
  EXPECT_NEAR(std::hypot(p.x, p.y), 0.0, 1e-9)
    << "a full circle must close; got (" << p.x << ", " << p.y << ")";
  EXPECT_NEAR(std::abs(normalizeAngle(p.theta)), 0.0, 1e-9);
  EXPECT_NEAR(d.odom().distance_travelled_m(), 2.0 * kPi * radius, 1e-9);
}

TEST(Odometry, ArcAccuracyIsIndependentOfStepSize)
{
  // Exactness means the answer does not improve with smaller steps, because
  // there is nothing left to improve. A first-order scheme's error would fall
  // as O(dt); here it stays at machine precision across two orders of
  // magnitude of step size.
  const double v = 0.4;
  const double omega = 0.8;
  const double duration = 4.0;

  std::vector<Pose2D> results;
  for (const double dt : {0.001, 0.01, 0.05}) {
    Driver d(config());
    d.drive(v, omega, duration, dt);
    results.push_back(d.odom().pose());
  }
  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_NEAR(results[i].x, results[0].x, 1e-9) << "step size " << i;
    EXPECT_NEAR(results[i].y, results[0].y, 1e-9) << "step size " << i;
    EXPECT_NEAR(results[i].theta, results[0].theta, 1e-9) << "step size " << i;
  }
}

TEST(Odometry, MatchesTheAnalyticArcSolution)
{
  // Closed form for constant (v, omega) starting from the origin at theta = 0:
  //     x = (v/omega) * sin(omega*t)
  //     y = (v/omega) * (1 - cos(omega*t))
  const double v = 0.35;
  const double omega = 0.6;
  const double t = 3.0;

  Driver d(config());
  d.drive(v, omega, t, 0.01);

  const double R = v / omega;
  EXPECT_NEAR(d.odom().pose().x, R * std::sin(omega * t), 1e-9);
  EXPECT_NEAR(d.odom().pose().y, R * (1.0 - std::cos(omega * t)), 1e-9);
  EXPECT_NEAR(d.odom().pose().theta, normalizeAngle(omega * t), 1e-9);
}

TEST(Odometry, HeadingWrapsAndStaysInRange)
{
  Driver d(config());
  d.drive(0.0, 2.0, 20.0, 0.01);   // many revolutions
  const double th = d.odom().pose().theta;
  EXPECT_GE(th, -kPi);
  EXPECT_LT(th, kPi);
}

// ------------------------------------------------- calibration sensitivity

TEST(Odometry, WheelRadiusErrorIsAPureScaleError)
{
  // A wrong radius multiplies every distance by the same factor and leaves the
  // heading untouched, because omega = r*(dr - dl)/L scales identically in
  // numerator and denominator of the ratio that sets the path shape.
  const OdometryConfig truth = config();
  OdometryConfig belief = config();
  belief.wheel_radius_m *= 1.05;          // the odometry thinks r is 5 % larger

  Driver a(truth, truth);
  Driver b(truth, belief);       // same physical robot, miscalibrated estimator
  a.drive(0.4, 0.3, 6.0, 0.01);
  b.drive(0.4, 0.3, 6.0, 0.01);

  EXPECT_NEAR(b.odom().distance_travelled_m(),
              1.05 * a.odom().distance_travelled_m(), 1e-9);
  // Heading is scaled too, because omega = r*(d_r - d_l)/L carries the same
  // factor. The point is that the error enters as a clean multiplicative
  // constant, which is exactly why it looks like a drifting sensor rather
  // than a broken one.
  EXPECT_NEAR(b.odom().pose().theta,
              normalizeAngle(1.05 * 0.3 * 6.0), 1e-6);
}

TEST(Odometry, TrackWidthErrorOnlyAffectsHeading)
{
  // L appears only in omega, so an error in it leaves the distance travelled
  // exactly right and the heading proportionally wrong. That is the nastier of
  // the two: distance checks out, the robot still ends up somewhere else.
  const OdometryConfig truth = config();
  OdometryConfig belief = config();
  belief.wheel_separation_m *= 1.10;      // the odometry thinks L is 10 % wider

  Driver a(truth, truth);
  Driver b(truth, belief);
  a.drive(0.4, 0.5, 4.0, 0.01);
  b.drive(0.4, 0.5, 4.0, 0.01);

  EXPECT_NEAR(b.odom().distance_travelled_m(),
              a.odom().distance_travelled_m(), 1e-9);
  EXPECT_NEAR(b.odom().pose().theta, normalizeAngle(0.5 * 4.0 / 1.10), 1e-6);
}

// ------------------------------------------------------------- covariance

// ------------------------------------------------- Jacobians vs finite differences
//
// A Jacobian that has never been compared against a finite difference of the
// function it claims to differentiate is an assertion, not a fact. These two
// tests are the sharpest available check on the covariance propagation, and
// they exist because a mutation that used the STARTING heading instead of the
// midpoint heading in the Jacobians survived every statistical covariance test
// in this file: the trace, the symmetry and the positive-definiteness were all
// unchanged, only the shape of the uncertainty ellipse was wrong.

namespace
{

/// Apply one step of the model to an arbitrary starting pose and wheel pair,
/// returning the resulting pose. Used as the function f(x, u) that the
/// Jacobians are differentiated against.
Pose2D apply_step(const OdometryConfig & c, const Pose2D & start,
                  double s_right, double s_left)
{
  DifferentialDriveOdometry o(c);
  o.reset(start);
  const double dl = s_left / c.wheel_radius_m;
  const double dr = s_right / c.wheel_radius_m;
  o.update(0.0, 0.0, 0.0);
  o.update(dl, dr, 0.02);
  return o.pose();
}

}  // namespace

TEST(Odometry, PoseJacobianMatchesAFiniteDifference)
{
  OdometryConfig c = config();
  c.max_wheel_step_rad = 100.0;
  const Pose2D start(1.3, -0.7, 0.9);
  const double s_right = 0.031;
  const double s_left = 0.019;      // a genuinely curved step
  const double eps = 1e-6;

  DifferentialDriveOdometry probe(c);
  probe.reset(start);
  probe.update(0.0, 0.0, 0.0);
  const auto step = probe.update(
    s_left / c.wheel_radius_m, s_right / c.wheel_radius_m, 0.02);
  ASSERT_TRUE(step.valid);
  const Eigen::Matrix3d Fx = step.jacobian_pose;

  for (int col = 0; col < 3; ++col) {
    Pose2D plus = start;
    Pose2D minus = start;
    if (col == 0) {plus.x += eps; minus.x -= eps;} else if (col == 1) {
      plus.y += eps; minus.y -= eps;
    } else {
      plus = Pose2D(start.x, start.y, start.theta + eps);
      minus = Pose2D(start.x, start.y, start.theta - eps);
    }
    const Pose2D fp = apply_step(c, plus, s_right, s_left);
    const Pose2D fm = apply_step(c, minus, s_right, s_left);
    const double dx = (fp.x - fm.x) / (2 * eps);
    const double dy = (fp.y - fm.y) / (2 * eps);
    const double dth = normalizeAngle(fp.theta - fm.theta) / (2 * eps);

    EXPECT_NEAR(Fx(0, col), dx, 1e-5) << "Fx row 0 col " << col;
    EXPECT_NEAR(Fx(1, col), dy, 1e-5) << "Fx row 1 col " << col;
    EXPECT_NEAR(Fx(2, col), dth, 1e-5) << "Fx row 2 col " << col;
  }
}

TEST(Odometry, InputJacobianMatchesAFiniteDifference)
{
  OdometryConfig c = config();
  c.max_wheel_step_rad = 100.0;
  const Pose2D start(-0.4, 2.1, -1.2);
  const double s_right = 0.028;
  const double s_left = 0.015;
  const double eps = 1e-7;

  DifferentialDriveOdometry probe(c);
  probe.reset(start);
  probe.update(0.0, 0.0, 0.0);
  const auto step = probe.update(
    s_left / c.wheel_radius_m, s_right / c.wheel_radius_m, 0.02);
  ASSERT_TRUE(step.valid);
  const Eigen::Matrix<double, 3, 2> Fu = step.jacobian_input;

  for (int col = 0; col < 2; ++col) {
    const double dr_plus = s_right + (col == 0 ? eps : 0.0);
    const double dl_plus = s_left + (col == 1 ? eps : 0.0);
    const double dr_minus = s_right - (col == 0 ? eps : 0.0);
    const double dl_minus = s_left - (col == 1 ? eps : 0.0);
    const Pose2D fp = apply_step(c, start, dr_plus, dl_plus);
    const Pose2D fm = apply_step(c, start, dr_minus, dl_minus);

    EXPECT_NEAR(Fu(0, col), (fp.x - fm.x) / (2 * eps), 1e-5) << "Fu row 0 col " << col;
    EXPECT_NEAR(Fu(1, col), (fp.y - fm.y) / (2 * eps), 1e-5) << "Fu row 1 col " << col;
    EXPECT_NEAR(Fu(2, col), normalizeAngle(fp.theta - fm.theta) / (2 * eps), 1e-5)
      << "Fu row 2 col " << col;
  }
}

TEST(Odometry, CovarianceStartsAtZeroAndOnlyGrows)
{
  // Dead reckoning has no absolute reference, so its uncertainty can never
  // decrease. A covariance that shrinks is a filter that has invented
  // information.
  Driver d(config());
  d.drive(0.4, 0.2, 10.0, 0.02);

  Driver e(config());
  double previous_trace = 0.0;
  for (int i = 0; i < 200; ++i) {
    e.drive(0.4, 0.2, 0.05, 0.05);
    const double trace = e.odom().covariance().trace();
    EXPECT_GE(trace, previous_trace - 1e-15) << "covariance shrank at step " << i;
    previous_trace = trace;
  }
  EXPECT_GT(previous_trace, 0.0);
}

TEST(Odometry, CovarianceIsSymmetricAndPositiveSemiDefinite)
{
  Driver d(config());
  d.drive(0.45, 0.7, 12.0, 0.02);
  const Eigen::Matrix3d P = d.odom().covariance();

  // EXACT symmetry, not approximate. Fx*P*Fx' + Fu*Q*Fu' is symmetric in
  // exact arithmetic, but the (i,j) and (j,i) entries are accumulated in
  // different orders and can differ in the last bit. The implementation
  // therefore symmetrises explicitly, and the test asserts that guarantee
  // rather than a tolerance -- a tolerance here cannot distinguish "symmetric
  // by construction" from "symmetric by luck so far", and asymmetry compounds
  // once the Phase 4 update step starts factorising this matrix.
  EXPECT_EQ((P - P.transpose()).cwiseAbs().maxCoeff(), 0.0)
    << "covariance is not exactly symmetric:\n" << P;

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(P);
  EXPECT_GE(solver.eigenvalues().minCoeff(), -1e-12)
    << "eigenvalues: " << solver.eigenvalues().transpose();
}

TEST(Odometry, PositionVarianceGrowsWithDistanceNotTime)
{
  // The model says variance is linear in distance travelled. Driving twice as
  // far must roughly double it; standing still must not grow it appreciably.
  Driver fast(config());
  fast.drive(0.5, 0.0, 10.0, 0.02);      // 5 m
  Driver slow(config());
  slow.drive(0.25, 0.0, 10.0, 0.02);     // 2.5 m, same elapsed time

  const double v_fast = fast.odom().covariance()(0, 0) + fast.odom().covariance()(1, 1);
  const double v_slow = slow.odom().covariance()(0, 0) + slow.odom().covariance()(1, 1);
  EXPECT_GT(v_fast, 1.5 * v_slow)
    << "uncertainty must track distance, not elapsed time";
}

TEST(Odometry, HeadingVarianceGrowsEvenWhenDrivingStraight)
{
  // The classic result: a straight line still accumulates heading
  // uncertainty, because the two wheel errors are independent and their
  // difference is what sets the heading. That heading uncertainty then feeds
  // position error that grows FASTER than linearly with distance. This is
  // precisely why Phase 4 needs an independent heading source.
  Driver d(config());
  d.drive(0.5, 0.0, 20.0, 0.02);
  EXPECT_GT(d.odom().covariance()(2, 2), 0.0);
  EXPECT_GT(std::sqrt(d.odom().covariance()(2, 2)), 1e-3);
}

// ------------------------------------------------------------- robustness

TEST(Odometry, FirstSampleOnlyPrimesTheState)
{
  DifferentialDriveOdometry o(config());
  const auto step = o.update(10.0, -3.0, 0.0);
  EXPECT_FALSE(step.valid);
  EXPECT_EQ(step.reason, OdometryReject::kFirstSample);
  EXPECT_NEAR(o.pose().x, 0.0, 1e-15);
}

TEST(Odometry, RejectsBackwardsTimeNonFiniteAndImplausibleJumps)
{
  DifferentialDriveOdometry o(config());
  o.update(0.0, 0.0, 1.0);
  ASSERT_TRUE(o.update(0.1, 0.1, 1.02).valid);

  EXPECT_EQ(o.update(0.2, 0.2, 1.0).reason, OdometryReject::kNonMonotonicTime);
  EXPECT_EQ(o.update(0.2, 0.2, 1.02).reason, OdometryReject::kNonMonotonicTime);
  EXPECT_EQ(o.update(std::nan(""), 0.2, 1.04).reason, OdometryReject::kNonFinite);
  EXPECT_EQ(o.update(50.0, 0.2, 1.06).reason, OdometryReject::kImplausibleStep);
}

TEST(Odometry, ARejectedJumpDoesNotLoseTheMotionThatFollows)
{
  // After rejecting an implausible step the reference must NOT be resynced to
  // the bad value, or the motion between the last good sample and the next one
  // is silently dropped.
  OdometryConfig c = config();
  c.max_wheel_step_rad = 0.5;
  DifferentialDriveOdometry o(c);
  o.update(0.0, 0.0, 0.0);
  ASSERT_EQ(o.update(5.0, 5.0, 0.02).reason, OdometryReject::kImplausibleStep);

  // The next plausible-looking sample is measured against the LAST GOOD one,
  // so a 0.4 rad step from zero integrates normally.
  const auto step = o.update(0.4, 0.4, 0.04);
  ASSERT_TRUE(step.valid);
  EXPECT_NEAR(step.left_arc_m, 0.4 * c.wheel_radius_m, 1e-12);
}

TEST(Odometry, ResetClearsPoseCovarianceAndHistory)
{
  Driver d(config());
  d.drive(0.4, 0.3, 5.0, 0.02);
  ASSERT_GT(d.odom().covariance().trace(), 0.0);

  d.odom().reset(Pose2D(1.0, 2.0, 0.5));
  EXPECT_NEAR(d.odom().pose().x, 1.0, 1e-15);
  EXPECT_NEAR(d.odom().pose().theta, 0.5, 1e-15);
  EXPECT_NEAR(d.odom().covariance().trace(), 0.0, 1e-15);
  EXPECT_NEAR(d.odom().distance_travelled_m(), 0.0, 1e-15);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
