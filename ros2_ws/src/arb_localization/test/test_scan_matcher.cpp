// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_localization/scan_matcher.hpp"

using arb_core::Pose2D;
using arb_localization::LikelihoodField;
using arb_localization::MatcherConfig;
using arb_localization::MatchResult;
using arb_localization::ScanMatcher;
using arb_localization::ScanPoint;

namespace
{

constexpr double kRes = 0.05;
constexpr int kW = 240;     // 12 m
constexpr int kH = 240;
constexpr double kOX = -6.0;
constexpr double kOY = -6.0;

/// A closed rectangular room, as occupancy bytes.
std::vector<std::int8_t> make_room(double x0, double y0, double x1, double y1)
{
  std::vector<std::int8_t> occ(static_cast<std::size_t>(kW) * kH, -1);
  auto set = [&](double wx, double wy) {
      const int cx = static_cast<int>(std::floor((wx - kOX) / kRes));
      const int cy = static_cast<int>(std::floor((wy - kOY) / kRes));
      if (cx >= 0 && cy >= 0 && cx < kW && cy < kH) {
        occ[static_cast<std::size_t>(cy) * kW + cx] = 100;
      }
    };
  for (double x = x0; x <= x1; x += kRes / 2) {set(x, y0); set(x, y1);}
  for (double y = y0; y <= y1; y += kRes / 2) {set(x0, y); set(x1, y);}
  return occ;
}

/// Ray-cast an ideal 360-beam scan of that room from `sensor`, returned in the
/// SENSOR frame, which is what the matcher expects.
std::vector<ScanPoint> simulate_scan(
  const Pose2D & sensor, double x0, double y0, double x1, double y1, int beams = 180)
{
  std::vector<ScanPoint> points;
  for (int i = 0; i < beams; ++i) {
    const double a = -arb_core::pi<double>() +
      2.0 * arb_core::pi<double>() * i / beams;
    const double world_a = sensor.theta + a;
    const double c = std::cos(world_a);
    const double s = std::sin(world_a);
    // Distance to the axis-aligned box from inside.
    double best = 1e9;
    if (c > 1e-9) {best = std::min(best, (x1 - sensor.x) / c);}
    if (c < -1e-9) {best = std::min(best, (x0 - sensor.x) / c);}
    if (s > 1e-9) {best = std::min(best, (y1 - sensor.y) / s);}
    if (s < -1e-9) {best = std::min(best, (y0 - sensor.y) / s);}
    if (best > 100.0) {continue;}
    // Expressed in the sensor frame: range along the beam's own bearing.
    points.push_back({best * std::cos(a), best * std::sin(a)});
  }
  return points;
}

}  // namespace

// =================================================================
//  LIKELIHOOD FIELD
// =================================================================

TEST(LikelihoodField, InterpolatesBetweenCells)
{
  // The field is sampled bilinearly, so the score surface is continuous rather
  // than a staircase of one-cell steps. Without it the matcher can only
  // resolve to about half a cell and reports a 4 cm "correction" for a pose
  // that is already exactly right.
  std::vector<std::int8_t> occ(static_cast<std::size_t>(kW) * kH, -1);
  const int cx = kW / 2;
  const int cy = kH / 2;
  occ[static_cast<std::size_t>(cy) * kW + cx] = 100;
  LikelihoodField f(occ, kW, kH, kRes, kOX, kOY, 65, 0.15, 0.75);

  const double wx = kOX + (cx + 0.5) * kRes;
  const double wy = kOY + (cy + 0.5) * kRes;

  // Sampling across one cell must change smoothly and strictly monotonically,
  // which a nearest-cell lookup cannot do.
  double previous = f.at(wx, wy);
  int strictly_decreasing = 0;
  for (double d = kRes / 8; d <= kRes; d += kRes / 8) {
    const double v = f.at(wx + d, wy);
    if (v < previous - 1e-12) {++strictly_decreasing;}
    previous = v;
  }
  EXPECT_GE(strictly_decreasing, 6)
    << "the field must vary WITHIN a cell, not only between cells";
}

TEST(LikelihoodField, PeaksOnObstaclesAndDecaysAway)
{
  std::vector<std::int8_t> occ(static_cast<std::size_t>(kW) * kH, -1);
  const int cx = kW / 2;
  const int cy = kH / 2;
  occ[static_cast<std::size_t>(cy) * kW + cx] = 100;

  LikelihoodField f(occ, kW, kH, kRes, kOX, kOY, 65, 0.15, 0.75);
  ASSERT_EQ(f.obstacleCount(), 1);

  const double wx = kOX + (cx + 0.5) * kRes;
  const double wy = kOY + (cy + 0.5) * kRes;
  EXPECT_NEAR(f.at(wx, wy), 1.0, 1e-6);

  // Monotone decay, which is the property that makes any search work.
  double previous = 1.0;
  for (double d = kRes; d < 0.7; d += kRes) {
    const double v = f.at(wx + d, wy);
    EXPECT_LT(v, previous + 1e-9) << "d=" << d;
    EXPECT_GT(v, 0.0);
    previous = v;
  }
  EXPECT_NEAR(f.at(wx + 3.0, wy), 0.0, 1e-9) << "far away must be zero";
}

TEST(LikelihoodField, UsesTheMaximumNotTheSumOfOverlappingObstacles)
{
  // Summing would make a thick wall score higher than a thin one, and the
  // matcher would prefer poses that pile points onto clutter rather than poses
  // that fit the geometry.
  std::vector<std::int8_t> one(static_cast<std::size_t>(kW) * kH, -1);
  std::vector<std::int8_t> many(static_cast<std::size_t>(kW) * kH, -1);
  const int cx = kW / 2;
  const int cy = kH / 2;
  one[static_cast<std::size_t>(cy) * kW + cx] = 100;
  for (int d = -2; d <= 2; ++d) {
    many[static_cast<std::size_t>(cy + d) * kW + cx] = 100;
  }
  LikelihoodField a(one, kW, kH, kRes, kOX, kOY);
  LikelihoodField b(many, kW, kH, kRes, kOX, kOY);
  const double wx = kOX + (cx + 0.5) * kRes;
  const double wy = kOY + (cy + 0.5) * kRes;
  EXPECT_NEAR(a.at(wx, wy), 1.0, 1e-6);
  EXPECT_NEAR(b.at(wx, wy), 1.0, 1e-6) << "five obstacles must not score five";
}

TEST(LikelihoodField, IgnoresUnknownAndFreeCells)
{
  std::vector<std::int8_t> occ(static_cast<std::size_t>(kW) * kH, -1);
  for (std::size_t i = 0; i < occ.size() / 2; ++i) {occ[i] = 20;}   // free
  LikelihoodField f(occ, kW, kH, kRes, kOX, kOY, 65);
  EXPECT_TRUE(f.empty());
  EXPECT_NEAR(f.at(0.0, 0.0), 0.0, 1e-12);
}

TEST(LikelihoodField, IsZeroOutsideTheGrid)
{
  LikelihoodField f(make_room(-3, -3, 3, 3), kW, kH, kRes, kOX, kOY);
  EXPECT_NEAR(f.at(100.0, 0.0), 0.0, 1e-12);
  EXPECT_NEAR(f.at(0.0, -100.0), 0.0, 1e-12);
}

// =================================================================
//  MATCHING
// =================================================================

TEST(ScanMatcher, RecoversAKnownTranslation)
{
  const double x0 = -3.0, y0 = -3.0, x1 = 3.0, y1 = 3.0;
  LikelihoodField field(make_room(x0, y0, x1, y1), kW, kH, kRes, kOX, kOY);
  const Pose2D truth(0.4, -0.6, 0.3);
  const auto points = simulate_scan(truth, x0, y0, x1, y1);
  ASSERT_GT(points.size(), 100u);

  // The filter thinks it is 18 cm away from where it is.
  const Pose2D guess(truth.x - 0.18, truth.y + 0.12, truth.theta);

  const double initial_error = std::hypot(guess.x - truth.x, guess.y - truth.y);

  ScanMatcher matcher;
  const MatchResult r = matcher.match(field, guess, points);
  ASSERT_TRUE(r.valid) << "score " << r.score;

  const double residual = std::hypot(r.pose.x - truth.x, r.pose.y - truth.y);

  // The bound is ONE CELL, and it cannot be tighter. The map is a raster: a
  // wall at x = 3.000 is stored in the cell whose centre is at x = 3.025, so
  // the map's own idea of where the wall is differs from the truth by up to
  // half a cell. A matcher cannot be more accurate than the map it matches
  // against, and claiming otherwise in a test only means the test is fitting
  // the raster's rounding.
  EXPECT_LT(residual, kRes)
    << "matched (" << r.pose.x << ", " << r.pose.y << ") vs truth ("
    << truth.x << ", " << truth.y << ")";

  // The operationally meaningful claim: the correction removed most of the
  // error it was given.
  EXPECT_LT(residual, 0.35 * initial_error)
    << "residual " << residual << " against an initial " << initial_error;
  EXPECT_GT(r.score, r.initial_score);
}

TEST(ScanMatcher, RecoversAKnownRotation)
{
  // The error that actually matters here: Phase 6 measured the whole map
  // rotating by several degrees as heading drift accumulated over 82 m.
  const double x0 = -3.0, y0 = -3.0, x1 = 3.0, y1 = 3.0;
  LikelihoodField field(make_room(x0, y0, x1, y1), kW, kH, kRes, kOX, kOY);
  const Pose2D truth(0.2, 0.1, 0.0);
  const auto points = simulate_scan(truth, x0, y0, x1, y1);

  const double drift = arb_core::degToRad(4.0);
  const Pose2D guess(truth.x, truth.y, truth.theta + drift);

  ScanMatcher matcher;
  const MatchResult r = matcher.match(field, guess, points);
  ASSERT_TRUE(r.valid) << "score " << r.score;
  EXPECT_LT(std::abs(arb_core::angleDiff(r.pose.theta, truth.theta)),
            arb_core::degToRad(1.0))
    << "residual " << arb_core::radToDeg(
    arb_core::angleDiff(r.pose.theta, truth.theta)) << " deg";
}

TEST(ScanMatcher, LeavesAnAlreadyCorrectPoseAlone)
{
  const double x0 = -3.0, y0 = -3.0, x1 = 3.0, y1 = 3.0;
  LikelihoodField field(make_room(x0, y0, x1, y1), kW, kH, kRes, kOX, kOY);
  const Pose2D truth(-0.5, 0.7, -0.4);
  const auto points = simulate_scan(truth, x0, y0, x1, y1);

  ScanMatcher matcher;
  const MatchResult r = matcher.match(field, truth, points);
  ASSERT_TRUE(r.valid);
  // The bound is one cell, not zero. The map is a raster: a wall's true
  // position falls somewhere inside a cell, and no matcher can recover
  // information the map does not contain. Bilinear interpolation gets this
  // well inside one cell; nearest-cell lookup does not.
  EXPECT_LT(std::hypot(r.pose.x - truth.x, r.pose.y - truth.y), kRes)
    << "a correct pose was moved by more than one cell";
  EXPECT_LT(std::abs(arb_core::angleDiff(r.pose.theta, truth.theta)),
            arb_core::degToRad(0.5));
}

TEST(ScanMatcher, RejectsAMatchAgainstAnEmptyMap)
{
  std::vector<std::int8_t> occ(static_cast<std::size_t>(kW) * kH, -1);
  LikelihoodField field(occ, kW, kH, kRes, kOX, kOY);
  const auto points = simulate_scan(Pose2D(0, 0, 0), -3, -3, 3, 3);
  ScanMatcher matcher;
  const MatchResult r = matcher.match(field, Pose2D(0, 0, 0), points);
  EXPECT_FALSE(r.valid);
  EXPECT_NEAR(r.pose.x, 0.0, 1e-12) << "a rejected match must not move the pose";
}

TEST(ScanMatcher, RejectsAMatchWithTooFewPoints)
{
  LikelihoodField field(make_room(-3, -3, 3, 3), kW, kH, kRes, kOX, kOY);
  std::vector<ScanPoint> few = {{1.0, 0.0}, {0.0, 1.0}};
  ScanMatcher matcher;
  EXPECT_FALSE(matcher.match(field, Pose2D(0, 0, 0), few).valid);
}

TEST(ScanMatcher, RejectsAScanThatDoesNotFitTheMap)
{
  // A scan of a different room. The matcher must report a low score rather
  // than confidently snapping to the best of a set of bad options: a bad
  // correction moves the estimate, which makes the next scan fit even worse.
  LikelihoodField field(make_room(-3.0, -3.0, 3.0, 3.0), kW, kH, kRes, kOX, kOY);
  const auto points = simulate_scan(Pose2D(0, 0, 0), -1.0, -1.0, 1.0, 1.0);

  MatcherConfig c;
  c.min_score = 0.5;
  ScanMatcher matcher(c);
  const MatchResult r = matcher.match(field, Pose2D(0, 0, 0), points);
  EXPECT_FALSE(r.valid) << "score " << r.score;
}

TEST(ScanMatcher, IsRobustToMeasurementNoise)
{
  const double x0 = -3.0, y0 = -3.0, x1 = 3.0, y1 = 3.0;
  LikelihoodField field(make_room(x0, y0, x1, y1), kW, kH, kRes, kOX, kOY);
  const Pose2D truth(0.3, -0.4, 0.15);
  auto points = simulate_scan(truth, x0, y0, x1, y1);

  std::mt19937 rng(3);
  std::normal_distribution<double> n(0.0, 0.015);   // the configured LiDAR sigma
  for (auto & p : points) {
    const double r = std::hypot(p.x, p.y) + n(rng);
    const double a = std::atan2(p.y, p.x);
    p.x = r * std::cos(a);
    p.y = r * std::sin(a);
  }

  ScanMatcher matcher;
  const MatchResult r = matcher.match(
    field, Pose2D(truth.x - 0.1, truth.y + 0.08, truth.theta + 0.03), points);
  ASSERT_TRUE(r.valid) << "score " << r.score;
  EXPECT_LT(std::hypot(r.pose.x - truth.x, r.pose.y - truth.y), 0.05);
}

TEST(ScanMatcher, CovarianceShrinksWithTheScoreButNeverToZero)
{
  // The map was built from the same estimate the matcher is correcting, so a
  // perfect match proves the scan agrees with the robot's own beliefs, not
  // with the world. A floor on the covariance stops the filter driving its
  // uncertainty to zero on self-referential evidence.
  const double x0 = -3.0, y0 = -3.0, x1 = 3.0, y1 = 3.0;
  LikelihoodField field(make_room(x0, y0, x1, y1), kW, kH, kRes, kOX, kOY);
  const Pose2D truth(0.0, 0.0, 0.0);
  const auto points = simulate_scan(truth, x0, y0, x1, y1);

  MatcherConfig c;
  ScanMatcher matcher(c);
  const MatchResult r = matcher.match(field, truth, points);
  ASSERT_TRUE(r.valid);
  EXPECT_GE(std::sqrt(r.covariance(0, 0)), c.position_sigma_m - 1e-9);
  EXPECT_GE(std::sqrt(r.covariance(2, 2)), c.heading_sigma_rad - 1e-9);
  EXPECT_LT(std::sqrt(r.covariance(0, 0)), 10.0 * c.position_sigma_m);
}

TEST(ScanMatcher, ScoreIsInvariantToWhereThePointsAreListed)
{
  LikelihoodField field(make_room(-3, -3, 3, 3), kW, kH, kRes, kOX, kOY);
  auto points = simulate_scan(Pose2D(0.2, 0.1, 0.0), -3, -3, 3, 3);
  const double a = ScanMatcher::score(field, Pose2D(0.2, 0.1, 0.0), points);
  std::reverse(points.begin(), points.end());
  const double b = ScanMatcher::score(field, Pose2D(0.2, 0.1, 0.0), points);
  EXPECT_NEAR(a, b, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
