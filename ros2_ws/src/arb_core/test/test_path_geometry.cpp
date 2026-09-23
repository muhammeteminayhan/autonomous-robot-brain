// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_core/path_geometry.hpp"

using arb_core::PathProjection;
using arb_core::Pose2D;
using arb_core::carrotAt;
using arb_core::pathLength;
using arb_core::pathSpacing;
using arb_core::projectOntoPath;

namespace
{
/// Densify a polyline of corners to a fixed spacing, as a planner would.
std::vector<Pose2D> densify(const std::vector<Pose2D> & corners, double step)
{
  std::vector<Pose2D> out{corners.front()};
  for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
    const double dx = corners[i + 1].x - corners[i].x;
    const double dy = corners[i + 1].y - corners[i].y;
    const int n = std::max(1, static_cast<int>(std::round(std::hypot(dx, dy) / step)));
    for (int k = 1; k <= n; ++k) {
      const double a = static_cast<double>(k) / n;
      out.emplace_back(corners[i].x + a * dx, corners[i].y + a * dy, 0.0);
    }
  }
  return out;
}

/// Express a path given in WORLD coordinates in the frame of a robot at
/// (x, y, th). Every test that builds its path relative to the robot's current
/// position is testing a path that slides along with the robot, and a path like
/// that hides any bug in how far along it the controller thinks it is.
std::vector<Pose2D> toRobotFrame(const std::vector<Pose2D> & world,
                                 double x, double y, double th)
{
  std::vector<Pose2D> local;
  local.reserve(world.size());
  const double c = std::cos(-th);
  const double s = std::sin(-th);
  for (const auto & p : world) {
    const double dx = p.x - x;
    const double dy = p.y - y;
    local.emplace_back(c * dx - s * dy, s * dx + c * dy, 0.0);
  }
  return local;
}

std::vector<Pose2D> straight(double length, double step, double lateral)
{
  std::vector<Pose2D> p;
  for (double x = 0.0; x <= length + 1e-12; x += step) {p.emplace_back(x, lateral, 0.0);}
  return p;
}
}  // namespace

// ======================================================= length and spacing ==
TEST(PathGeometry, LengthAndSpacing)
{
  const auto p = straight(4.0, 0.05, 0.0);
  EXPECT_NEAR(pathLength(p), 4.0, 1e-9);
  EXPECT_NEAR(pathSpacing(p), 0.05, 1e-9);
  EXPECT_NEAR(pathLength({}), 0.0, 1e-12);
  EXPECT_NEAR(pathLength({Pose2D(1.0, 1.0, 0.0)}), 0.0, 1e-12);
  // A degenerate path must not divide by zero.
  EXPECT_NEAR(pathSpacing({Pose2D(1.0, 1.0, 0.0)}), 1.0, 1e-12);
}

// ================================================================ projection ==
TEST(Projection, IsInvalidForAnEmptyPath)
{
  const auto p = projectOntoPath({});
  EXPECT_FALSE(p.valid);
}

TEST(Projection, IsZeroOnThePath)
{
  const auto p = projectOntoPath(straight(4.0, 0.05, 0.0));
  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.signed_distance, 0.0, 1e-12);
  EXPECT_NEAR(p.arc, 0.0, 1e-12);
}

TEST(Projection, IsSignedWithPositiveMeaningThePathIsToTheLeft)
{
  EXPECT_GT(projectOntoPath(straight(4.0, 0.05, 0.5)).signed_distance, 0.0);
  EXPECT_LT(projectOntoPath(straight(4.0, 0.05, -0.5)).signed_distance, 0.0);
  EXPECT_NEAR(projectOntoPath(straight(4.0, 0.05, 0.5)).signed_distance, 0.5, 1e-9);
}

TEST(Projection, MeasuresToTheSegmentNotTheNearestVertex)
{
  // Two vertices 2 m apart straddling the robot: the nearest VERTEX is 1.0 m
  // away, the nearest point ON THE SEGMENT is 0.3 m away.
  const std::vector<Pose2D> coarse = {Pose2D(-1.0, 0.3, 0.0), Pose2D(1.0, 0.3, 0.0)};
  EXPECT_NEAR(projectOntoPath(coarse).signed_distance, 0.3, 1e-9);
}

TEST(Projection, ReportsArcLengthSoACarrotHasSomethingToMeasureFrom)
{
  // The same path seen from 1.5 m along it: the nearest point is at arc 1.5.
  const auto world = straight(4.0, 0.05, 0.0);
  const auto p = projectOntoPath(toRobotFrame(world, 1.5, 0.0, 0.0));
  EXPECT_NEAR(p.arc, 1.5, 1e-6);
  EXPECT_NEAR(p.signed_distance, 0.0, 1e-6);
}

TEST(Projection, ProgressWindowKeepsItOffALegAlreadyDriven)
{
  // Out along y = 0, up, then back along y = 0.5. The robot is on the RETURN
  // leg but 0.20 m from the outbound one and 0.30 m from the return one, so an
  // unrestricted search snaps backwards -- and reports the opposite sign, which
  // would steer the robot away from the path it is on.
  const auto world = densify(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), Pose2D(3.0, 0.5, 0.0),
      Pose2D(0.0, 0.5, 0.0)}, 0.05);
  const double back = arb_core::pi<double>();
  const auto local = toRobotFrame(world, 1.5, 0.20, back);

  const auto unrestricted = projectOntoPath(local, 0);
  EXPECT_NEAR(unrestricted.arc, 1.5, 0.05) << "should have snapped to the outbound leg";
  EXPECT_GT(unrestricted.signed_distance, 0.0);

  // The return leg starts at arc 3.5, i.e. segment 70 at 5 cm spacing.
  const auto windowed = projectOntoPath(local, 70);
  EXPECT_NEAR(windowed.arc, 3.5 + 1.5, 0.05) << "should stay on the return leg";
  EXPECT_LT(windowed.signed_distance, 0.0)
    << "facing -x, the return leg at larger y is to the robot's RIGHT";
}

TEST(Projection, FallsBackToAGlobalSearchWhenTheWindowExcludesEverything)
{
  // A window past the end of the path must not report a zero error it has not
  // earned; it falls back to searching the whole path.
  const auto p = projectOntoPath(straight(4.0, 0.05, 0.5), 10000);
  EXPECT_TRUE(p.valid);
  EXPECT_NEAR(p.signed_distance, 0.5, 1e-9);
}

// ==================================================================== carrot ==
TEST(Carrot, SitsOneLookaheadFurtherAlongThanTheProjection)
{
  const auto world = straight(6.0, 0.05, 0.0);
  // Robot 2.0 m along the path. The carrot must be at arc 2.0 + 0.8 = 2.8,
  // i.e. 0.8 m AHEAD of the robot -- not 0.8 m from the path's first vertex,
  // which would be 1.2 m BEHIND it.
  const auto local = toRobotFrame(world, 2.0, 0.0, 0.0);
  const auto p = projectOntoPath(local);
  const Pose2D carrot = carrotAt(local, p.arc, 0.8);
  EXPECT_NEAR(carrot.x, 0.8, 1e-6);
  EXPECT_NEAR(carrot.y, 0.0, 1e-6);
  EXPECT_GT(carrot.x, 0.0) << "carrot fell behind the robot";
}

TEST(Carrot, IsInterpolatedWithinASegmentRatherThanSnappedToAVertex)
{
  // On a path densified to 50 cm, a 0.30 m lookahead must land at 0.30 m, not
  // at the 0.50 m vertex: snapping quantises the carrot's bearing and the
  // steering command inherits the steps.
  const auto coarse = straight(4.0, 0.5, 0.0);
  const Pose2D carrot = carrotAt(coarse, 0.0, 0.30);
  EXPECT_NEAR(carrot.x, 0.30, 1e-9);
}

TEST(Carrot, ClampsToThePathEndWhenTheLookaheadOverrunsIt)
{
  const auto p = straight(1.0, 0.05, 0.0);
  const Pose2D carrot = carrotAt(p, 0.0, 5.0);
  EXPECT_NEAR(carrot.x, 1.0, 1e-9);
  EXPECT_NEAR(carrotAt({}, 0.0, 1.0).x, 0.0, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
