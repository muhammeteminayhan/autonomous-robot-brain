// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"

using arb_core::normalizeAngle;
using arb_core::normalizeAnglePositive;
using arb_core::angleDiff;
using arb_core::slerpAngle;
using arb_core::Pose2D;
using arb_core::pi;
using arb_core::two_pi;

namespace
{
constexpr double kEps = 1e-12;
constexpr double kLooseEps = 1e-9;
}  // namespace

// ---------------------------------------------------------------- normalizeAngle

TEST(NormalizeAngle, LeavesValuesInsideRangeUntouched)
{
  EXPECT_NEAR(normalizeAngle(0.0), 0.0, kEps);
  EXPECT_NEAR(normalizeAngle(1.0), 1.0, kEps);
  EXPECT_NEAR(normalizeAngle(-1.0), -1.0, kEps);
  EXPECT_NEAR(normalizeAngle(-pi()), -pi(), kEps);
}

TEST(NormalizeAngle, CollapsesPositivePiOntoNegativePi)
{
  // The interval is half-open [-pi, pi), so +pi must map to -pi.
  EXPECT_NEAR(normalizeAngle(pi()), -pi(), kEps);
}

TEST(NormalizeAngle, WrapsOneFullTurn)
{
  EXPECT_NEAR(normalizeAngle(two_pi()), 0.0, kEps);
  EXPECT_NEAR(normalizeAngle(-two_pi()), 0.0, kEps);
  EXPECT_NEAR(normalizeAngle(two_pi() + 0.5), 0.5, kLooseEps);
  EXPECT_NEAR(normalizeAngle(-two_pi() - 0.5), -0.5, kLooseEps);
}

TEST(NormalizeAngle, WrapsManyTurnsWithoutLoopingForever)
{
  // A naive `while (a > pi) a -= 2*pi;` would iterate ~160000 times here.
  EXPECT_NEAR(normalizeAngle(1000.0 * two_pi() + 0.25), 0.25, 1e-6);
  EXPECT_NEAR(normalizeAngle(-1000.0 * two_pi() - 0.25), -0.25, 1e-6);
}

TEST(NormalizeAngle, ResultAlwaysInsideHalfOpenInterval)
{
  for (int i = -2000; i <= 2000; ++i) {
    const double a = static_cast<double>(i) * 0.037;
    const double w = normalizeAngle(a);
    EXPECT_GE(w, -pi());
    EXPECT_LT(w, pi());
    // Wrapping must preserve the physical orientation: cos/sin unchanged.
    EXPECT_NEAR(std::cos(w), std::cos(a), kLooseEps);
    EXPECT_NEAR(std::sin(w), std::sin(a), kLooseEps);
  }
}

TEST(NormalizeAngle, IsIdempotent)
{
  for (int i = -100; i <= 100; ++i) {
    const double a = static_cast<double>(i) * 0.91;
    EXPECT_NEAR(normalizeAngle(normalizeAngle(a)), normalizeAngle(a), kEps);
  }
}

TEST(NormalizeAngle, NonFiniteInputYieldsNaNInsteadOfHanging)
{
  EXPECT_TRUE(std::isnan(normalizeAngle(std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizeAngle(-std::numeric_limits<double>::infinity())));
  EXPECT_TRUE(std::isnan(normalizeAngle(std::numeric_limits<double>::quiet_NaN())));
}

TEST(NormalizeAnglePositive, MapsIntoZeroToTwoPi)
{
  EXPECT_NEAR(normalizeAnglePositive(0.0), 0.0, kEps);
  EXPECT_NEAR(normalizeAnglePositive(-0.5), two_pi() - 0.5, kLooseEps);
  EXPECT_NEAR(normalizeAnglePositive(two_pi() + 0.5), 0.5, kLooseEps);
  for (int i = -500; i <= 500; ++i) {
    const double w = normalizeAnglePositive(static_cast<double>(i) * 0.13);
    EXPECT_GE(w, 0.0);
    EXPECT_LT(w, two_pi());
  }
}

// ---------------------------------------------------------------- angleDiff

TEST(AngleDiff, TakesTheShortWayAcrossTheWrapPoint)
{
  const double from = arb_core::degToRad(179.0);
  const double to = arb_core::degToRad(-179.0);
  // Naive (to - from) would be -358 deg; the true shortest rotation is +2 deg.
  EXPECT_NEAR(arb_core::radToDeg(angleDiff(to, from)), 2.0, 1e-9);
  EXPECT_NEAR(arb_core::radToDeg(angleDiff(from, to)), -2.0, 1e-9);
}

TEST(AngleDiff, IsAntisymmetric)
{
  for (int i = -50; i <= 50; ++i) {
    for (int j = -50; j <= 50; j += 7) {
      const double a = static_cast<double>(i) * 0.19;
      const double b = static_cast<double>(j) * 0.23;
      const double d = angleDiff(a, b);
      // Antisymmetry holds except at the +-pi endpoint, where both directions
      // are equally long and the half-open convention picks -pi for both.
      if (std::abs(std::abs(d) - pi()) > 1e-9) {
        EXPECT_NEAR(d, -angleDiff(b, a), kLooseEps);
      }
    }
  }
}

TEST(SlerpAngle, InterpolatesAlongShortestArc)
{
  const double a = arb_core::degToRad(170.0);
  const double b = arb_core::degToRad(-170.0);
  // Midpoint of the short arc is 180 deg, which normalises to -180 deg.
  EXPECT_NEAR(std::abs(arb_core::radToDeg(slerpAngle(a, b, 0.5))), 180.0, 1e-9);
  EXPECT_NEAR(slerpAngle(a, b, 0.0), a, kLooseEps);
  EXPECT_NEAR(slerpAngle(a, b, 1.0), arb_core::normalizeAngle(b), kLooseEps);
}

// ---------------------------------------------------------------- Pose2D

TEST(Pose2D, ConstructorNormalisesHeading)
{
  const Pose2D p(1.0, 2.0, 3.0 * two_pi() + 0.4);
  EXPECT_NEAR(p.theta, 0.4, 1e-9);
}

TEST(Pose2D, ComposeWithIdentityIsIdentity)
{
  const Pose2D p(1.5, -2.5, 0.7);
  const Pose2D id;
  const Pose2D r1 = p.compose(id);
  const Pose2D r2 = id.compose(p);
  EXPECT_NEAR(r1.x, p.x, kLooseEps);
  EXPECT_NEAR(r1.y, p.y, kLooseEps);
  EXPECT_NEAR(r1.theta, p.theta, kLooseEps);
  EXPECT_NEAR(r2.x, p.x, kLooseEps);
  EXPECT_NEAR(r2.y, p.y, kLooseEps);
  EXPECT_NEAR(r2.theta, p.theta, kLooseEps);
}

TEST(Pose2D, ComposeWithInverseReturnsIdentity)
{
  const Pose2D p(3.0, -1.0, 2.4);
  const Pose2D r = p.compose(p.inverse());
  EXPECT_NEAR(r.x, 0.0, kLooseEps);
  EXPECT_NEAR(r.y, 0.0, kLooseEps);
  EXPECT_NEAR(r.theta, 0.0, kLooseEps);
}

TEST(Pose2D, ComposeMatchesHomogeneousMatrixProduct)
{
  const Pose2D a(1.0, 2.0, 0.6);
  const Pose2D b(-0.5, 0.75, -1.2);
  const Eigen::Matrix3d expected = a.matrix() * b.matrix();
  const Eigen::Matrix3d actual = a.compose(b).matrix();
  EXPECT_TRUE(expected.isApprox(actual, 1e-9))
    << "expected:\n" << expected << "\nactual:\n" << actual;
}

TEST(Pose2D, RelativeToExpressesGoalInRobotFrame)
{
  // Robot at (2, 0) facing +90 deg. Goal at (2, 5) in world frame.
  // In robot frame the goal is 5 m straight ahead -> (5, 0).
  const Pose2D robot(2.0, 0.0, pi() / 2.0);
  const Pose2D goal(2.0, 5.0, pi() / 2.0);
  const Pose2D rel = robot.relativeTo(goal);
  EXPECT_NEAR(rel.x, 5.0, kLooseEps);
  EXPECT_NEAR(rel.y, 0.0, kLooseEps);
  EXPECT_NEAR(rel.theta, 0.0, kLooseEps);
}

TEST(Pose2D, DistanceAndHeadingErrorAreSymmetricAndWrapped)
{
  const Pose2D a(0.0, 0.0, arb_core::degToRad(179.0));
  const Pose2D b(3.0, 4.0, arb_core::degToRad(-179.0));
  EXPECT_NEAR(a.distanceTo(b), 5.0, kLooseEps);
  EXPECT_NEAR(b.distanceTo(a), 5.0, kLooseEps);
  EXPECT_NEAR(arb_core::radToDeg(a.headingErrorTo(b)), 2.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
