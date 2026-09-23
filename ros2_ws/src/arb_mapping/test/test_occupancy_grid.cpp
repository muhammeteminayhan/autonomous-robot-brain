// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <array>
#include <set>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_mapping/occupancy_grid.hpp"

using arb_core::Pose2D;
using arb_mapping::CellState;
using arb_mapping::GridConfig;
using arb_mapping::OccupancyGrid;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

GridConfig small_config()
{
  GridConfig c;
  c.resolution_m = 0.1;
  c.width_m = 10.0;
  c.height_m = 10.0;
  c.origin_x_m = -5.0;
  c.origin_y_m = -5.0;
  return c;
}

}  // namespace

// ===================================================================
//  LOG-ODDS
// ===================================================================

TEST(LogOdds, RoundTripsThroughProbability)
{
  for (const double p : {0.01, 0.1, 0.35, 0.5, 0.75, 0.9, 0.99}) {
    const double l = OccupancyGrid::probabilityToLogOdds(p);
    EXPECT_NEAR(OccupancyGrid::logOddsToProbability(l), p, 1e-9) << "p=" << p;
  }
}

TEST(LogOdds, ZeroIsTheUninformativePrior)
{
  EXPECT_NEAR(OccupancyGrid::probabilityToLogOdds(0.5), 0.0, 1e-12);
  EXPECT_NEAR(OccupancyGrid::logOddsToProbability(0.0), 0.5, 1e-12);
}

TEST(LogOdds, IsAntisymmetricAboutTheHalfProbability)
{
  // l(p) = -l(1-p). This is what makes the update symmetric: evidence for and
  // against a cell cancel exactly, which is what lets a map correct itself.
  for (const double p : {0.05, 0.2, 0.4, 0.49}) {
    EXPECT_NEAR(OccupancyGrid::probabilityToLogOdds(p),
                -OccupancyGrid::probabilityToLogOdds(1.0 - p), 1e-9);
  }
}

TEST(LogOdds, CertaintyIsClampedSoACellCanAlwaysBeRevised)
{
  // log-odds of p = 1 is infinite, and an infinite cell can never be moved by
  // any finite amount of contradicting evidence.
  EXPECT_TRUE(std::isfinite(OccupancyGrid::probabilityToLogOdds(0.0)));
  EXPECT_TRUE(std::isfinite(OccupancyGrid::probabilityToLogOdds(1.0)));
}

TEST(LogOdds, UpdatesAreAdditiveAndOrderIndependent)
{
  // The whole reason for working in log-odds: Bayes' rule becomes ADDITION.
  // The consequence is that the result depends only on how many observations
  // of each kind a cell received, never on the order they arrived in.
  GridConfig c = small_config();
  c.log_odds_min = -1e9;     // disable clamping to expose the pure algebra
  c.log_odds_max = 1e9;
  const Pose2D sensor(0.0, 0.0, 0.0);

  auto run = [&](const std::vector<bool> & hits) {
      OccupancyGrid g(c);
      for (const bool hit : hits) {
        g.integrateBeam(sensor, 0.0, hit ? 1.05 : 3.0);
      }
      int cx = 0;
      int cy = 0;
      EXPECT_TRUE(g.worldToCell(1.0, 0.0, cx, cy));
      return g.logOdds(cx, cy);
    };

  const double grouped = run({true, true, true, false, false});
  const double interleaved = run({true, false, true, false, true});
  const double reversed = run({false, false, true, true, true});
  EXPECT_NEAR(grouped, interleaved, 1e-6);
  EXPECT_NEAR(grouped, reversed, 1e-6);

  // And the value is exactly the sum of the per-observation increments.
  const double d_hit = OccupancyGrid::probabilityToLogOdds(c.p_hit) -
    OccupancyGrid::probabilityToLogOdds(c.p_prior);
  const double d_miss = OccupancyGrid::probabilityToLogOdds(c.p_miss) -
    OccupancyGrid::probabilityToLogOdds(c.p_prior);
  EXPECT_NEAR(grouped, 3 * d_hit + 2 * d_miss, 1e-6);
}

TEST(LogOdds, SymmetricEvidenceCancelsExactly)
{
  // Cancellation needs p_miss = 1 - p_hit, because the increments are
  // l(p_hit) - l(p_prior) and l(p_miss) - l(p_prior), and those are equal and
  // opposite only for symmetric probabilities.
  //
  // The defaults are NOT symmetric (0.75 and 0.35), and that asymmetry is
  // deliberate: a hit is stronger evidence than a miss, because a beam that
  // stopped somewhere says more about that cell than a beam that passed
  // through says about each cell along the way. The asymmetry is also why a
  // map slowly fills in rather than slowly empties.
  GridConfig c = small_config();
  c.p_hit = 0.7;
  c.p_miss = 0.3;
  c.log_odds_min = -1e9;
  c.log_odds_max = 1e9;
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);

  int cx = 0;
  int cy = 0;
  ASSERT_TRUE(g.worldToCell(1.0, 0.0, cx, cy));
  const double before = g.logOdds(cx, cy);

  for (int i = 0; i < 5; ++i) {g.integrateBeam(sensor, 0.0, 1.05);}
  EXPECT_GT(g.logOdds(cx, cy), before);
  for (int i = 0; i < 5; ++i) {g.integrateBeam(sensor, 0.0, 3.0);}
  EXPECT_NEAR(g.logOdds(cx, cy), before, 1e-6);
}

// ===================================================================
//  INDEXING
// ===================================================================

TEST(Grid, WorldToCellRoundTripsThroughCellCentres)
{
  OccupancyGrid g(small_config());
  for (int x = 0; x < g.width(); x += 7) {
    for (int y = 0; y < g.height(); y += 11) {
      double wx = 0.0;
      double wy = 0.0;
      g.cellToWorld(x, y, wx, wy);
      int bx = 0;
      int by = 0;
      ASSERT_TRUE(g.worldToCell(wx, wy, bx, by));
      EXPECT_EQ(bx, x);
      EXPECT_EQ(by, y);
    }
  }
}

TEST(Grid, NegativeCoordinatesUseFloorNotTruncation)
{
  // A cast to int truncates towards zero, so cells just below the origin get
  // the same index as cells just above it and the grid is off by one on that
  // side only. Using an origin at exactly zero exposes it.
  GridConfig c = small_config();
  c.origin_x_m = 0.0;
  c.origin_y_m = 0.0;
  OccupancyGrid g(c);

  int cx = 0;
  int cy = 0;
  EXPECT_TRUE(g.worldToCell(0.05, 0.05, cx, cy));
  EXPECT_EQ(cx, 0);
  EXPECT_EQ(cy, 0);
  // Just below the origin is OUTSIDE the grid, not cell 0.
  EXPECT_FALSE(g.worldToCell(-0.05, 0.05, cx, cy));
  EXPECT_EQ(cx, -1) << "the index must be -1, which truncation would make 0";
}

TEST(Grid, RejectsNonFiniteCoordinates)
{
  OccupancyGrid g(small_config());
  int cx = 0;
  int cy = 0;
  EXPECT_FALSE(g.worldToCell(kInf, 0.0, cx, cy));
  EXPECT_FALSE(g.worldToCell(0.0, std::nan(""), cx, cy));
}

TEST(Grid, StartsEntirelyUnknown)
{
  OccupancyGrid g(small_config());
  const auto c = g.counts();
  EXPECT_EQ(c.unknown, g.width() * g.height());
  EXPECT_EQ(c.free, 0);
  EXPECT_EQ(c.occupied, 0);
}

// ===================================================================
//  BRESENHAM
// ===================================================================

TEST(Bresenham, ExcludesTheEndpoint)
{
  // The endpoint is the cell the beam HIT. Marking it free before marking it
  // occupied would have the two updates fight each other.
  const auto cells = OccupancyGrid::bresenham(0, 0, 5, 0);
  ASSERT_EQ(cells.size(), 5u);
  EXPECT_EQ(cells.front(), std::make_pair(0, 0));
  EXPECT_EQ(cells.back(), std::make_pair(4, 0));
}

TEST(Bresenham, VisitsEveryCellExactlyOnceAndNeverSkips)
{
  // The two failure modes of a naive sub-stepping ray march: revisiting a cell
  // double-counts its evidence, and skipping one leaves a diagonal gap a
  // planner can path straight through.
  for (const auto & [x1, y1] : std::vector<std::pair<int, int>>{
      {7, 3}, {3, 7}, {-5, 2}, {2, -5}, {-6, -6}, {9, 9}, {0, 4}, {4, 0}})
  {
    const auto cells = OccupancyGrid::bresenham(0, 0, x1, y1);
    std::set<std::pair<int, int>> unique(cells.begin(), cells.end());
    EXPECT_EQ(unique.size(), cells.size())
      << "revisited a cell on the way to (" << x1 << ", " << y1 << ")";

    for (std::size_t i = 1; i < cells.size(); ++i) {
      const int dx = std::abs(cells[i].first - cells[i - 1].first);
      const int dy = std::abs(cells[i].second - cells[i - 1].second);
      EXPECT_LE(std::max(dx, dy), 1)
        << "skipped a cell between step " << i - 1 << " and " << i;
      EXPECT_GT(dx + dy, 0);
    }
    // And the line must actually get where it was going.
    const int dx = std::abs(cells.back().first - x1);
    const int dy = std::abs(cells.back().second - y1);
    EXPECT_LE(std::max(dx, dy), 1) << "did not reach (" << x1 << ", " << y1 << ")";
  }
}

TEST(Bresenham, IsSymmetricUnderReversal)
{
  // The same physical segment must cover the same cells whichever end the
  // traversal starts from. Plain Bresenham does NOT have this property: it
  // breaks exact diagonal ties in favour of its starting end, so a wall
  // scanned from the left and from the right clears marginally different
  // cells and the map grows direction-dependent seams along diagonals.
  //
  // This implementation sorts the endpoints before traversing and reverses
  // afterwards, which makes the cell set a property of the segment.
  for (const auto & seg : std::vector<std::array<int, 4>>{
      {1, 2, 9, 7}, {0, 0, 8, 4}, {-3, 5, 6, -2}, {2, 2, 2, 9}, {4, 1, 11, 8}})
  {
    const auto forward = OccupancyGrid::bresenham(seg[0], seg[1], seg[2], seg[3]);
    const auto backward = OccupancyGrid::bresenham(seg[2], seg[3], seg[0], seg[1]);
    std::set<std::pair<int, int>> f(forward.begin(), forward.end());
    std::set<std::pair<int, int>> b(backward.begin(), backward.end());
    f.insert({seg[2], seg[3]});
    b.insert({seg[0], seg[1]});
    EXPECT_EQ(f, b) << "segment (" << seg[0] << "," << seg[1] << ") -> ("
                    << seg[2] << "," << seg[3] << ")";
  }
}

TEST(Bresenham, StartsAtTheOriginAndKeepsTheCallersDirection)
{
  // Canonicalising the traversal must not leak into the ORDER the caller sees:
  // the first cell returned is always the caller's start.
  const auto a = OccupancyGrid::bresenham(9, 7, 1, 2);
  ASSERT_FALSE(a.empty());
  EXPECT_EQ(a.front(), std::make_pair(9, 7));
  const auto b = OccupancyGrid::bresenham(1, 2, 9, 7);
  ASSERT_FALSE(b.empty());
  EXPECT_EQ(b.front(), std::make_pair(1, 2));
}

TEST(Bresenham, HandlesAZeroLengthLine)
{
  EXPECT_TRUE(OccupancyGrid::bresenham(3, 3, 3, 3).empty());
}

TEST(Bresenham, StaysCloseToTheIdealLine)
{
  // Each visited cell must lie within half a cell of the true line, which is
  // the defining property of the algorithm.
  const int x1 = 17;
  const int y1 = 5;
  for (const auto & [x, y] : OccupancyGrid::bresenham(0, 0, x1, y1)) {
    const double ideal = static_cast<double>(y1) * x / x1;
    EXPECT_LE(std::abs(y - ideal), 0.5 + 1e-9) << "at x=" << x;
  }
}

// ===================================================================
//  INVERSE SENSOR MODEL
// ===================================================================

TEST(SensorModel, AHitMarksTheEndpointOccupiedAndTheRayFree)
{
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);

  // Enough repetitions to cross the thresholds.
  for (int i = 0; i < 12; ++i) {g.integrateBeam(sensor, 0.0, 2.0);}

  int hx = 0;
  int hy = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, hx, hy));
  EXPECT_EQ(g.state(hx, hy), CellState::kOccupied);

  int mx = 0;
  int my = 0;
  ASSERT_TRUE(g.worldToCell(1.0, 0.0, mx, my));
  EXPECT_EQ(g.state(mx, my), CellState::kFree);

  // Beyond the hit is NOT free: the beam never got there, so nothing is known.
  int bx = 0;
  int by = 0;
  ASSERT_TRUE(g.worldToCell(3.0, 0.0, bx, by));
  EXPECT_EQ(g.state(bx, by), CellState::kUnknown)
    << "space behind an obstacle is unknown, not free";
}

TEST(SensorModel, AMaxRangeBeamClearsWithoutPaintingAPhantomWall)
{
  // A beam that returns nothing still says the space was empty. Discarding it
  // leaves open floor permanently unknown; marking its endpoint occupied
  // paints a perfect arc of phantom wall at exactly the sensor's range limit.
  GridConfig c = small_config();
  c.max_usable_range_m = 3.0;
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);

  for (int i = 0; i < 12; ++i) {g.integrateBeam(sensor, 0.0, kInf);}

  int mx = 0;
  int my = 0;
  ASSERT_TRUE(g.worldToCell(1.5, 0.0, mx, my));
  EXPECT_EQ(g.state(mx, my), CellState::kFree) << "the ray must still clear";

  // The cell that WOULD be painted is the one at exactly max_usable_range,
  // not one just short of it. Checking 2.98 m instead of 3.0 m tests the cell
  // next door and lets the defect through -- which is exactly what an earlier
  // version of this test did.
  int ex = 0;
  int ey = 0;
  ASSERT_TRUE(g.worldToCell(c.max_usable_range_m, 0.0, ex, ey));
  EXPECT_NE(g.state(ex, ey), CellState::kOccupied)
    << "a no-return beam must not create an obstacle at the range limit";
  EXPECT_LE(g.logOdds(ex, ey), 0.0)
    << "the endpoint of a no-return beam must carry no evidence FOR occupancy";

  // And the whole arc, not just the beam straight ahead: a phantom wall from
  // this defect is a perfect circle around the robot.
  for (int deg = 0; deg < 360; deg += 30) {
    const double bearing = arb_core::degToRad(static_cast<double>(deg));
    for (int i = 0; i < 12; ++i) {g.integrateBeam(sensor, bearing, kInf);}
  }
  EXPECT_EQ(g.counts().occupied, 0)
    << "no-return beams must never create occupied cells anywhere";
}

TEST(SensorModel, DiscardsReturnsInsideTheMinimumRange)
{
  GridConfig c = small_config();
  c.min_usable_range_m = 0.2;
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(g.integrateBeam(sensor, 0.0, 0.1));
  }
  EXPECT_EQ(g.counts().unknown, g.width() * g.height());
}

TEST(SensorModel, RespectsTheSensorBearingAndPose)
{
  // A beam fired at 90 degrees from a rotated, translated sensor must land
  // where the geometry says, not where a forgotten rotation would put it.
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const Pose2D sensor(1.0, -1.0, arb_core::degToRad(90.0));

  const double bearing = sensor.theta;    // straight ahead in the sensor frame
  for (int i = 0; i < 12; ++i) {g.integrateBeam(sensor, bearing, 2.0);}

  int hx = 0;
  int hy = 0;
  ASSERT_TRUE(g.worldToCell(1.0, 1.0, hx, hy));   // 2 m along +y from (1, -1)
  EXPECT_EQ(g.state(hx, hy), CellState::kOccupied);
}

TEST(SensorModel, ClampingBoundsHowLongACellTakesToChangeItsMind)
{
  // Without clamping, a wall seen 500 times reaches log-odds +250 and a door
  // opened in it would need 500 contradicting observations to register.
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);

  for (int i = 0; i < 500; ++i) {g.integrateBeam(sensor, 0.0, 2.0);}
  int cx = 0;
  int cy = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, cx, cy));
  EXPECT_LE(g.logOdds(cx, cy), c.log_odds_max + 1e-6);
  EXPECT_EQ(g.state(cx, cy), CellState::kOccupied);

  // Now the obstacle is removed. The cell must become free again in a bounded
  // number of observations, not 500.
  int observations = 0;
  while (g.state(cx, cy) != CellState::kFree && observations < 200) {
    g.integrateBeam(sensor, 0.0, 4.0);
    ++observations;
  }
  EXPECT_EQ(g.state(cx, cy), CellState::kFree);
  EXPECT_LT(observations, 50)
    << "took " << observations << " observations to forget a wall";
}

TEST(SensorModel, ASensorOutsideTheGridIsIgnoredRatherThanCrashing)
{
  OccupancyGrid g(small_config());
  EXPECT_FALSE(g.integrateBeam(Pose2D(100.0, 100.0, 0.0), 0.0, 1.0));
  EXPECT_EQ(g.counts().unknown, g.width() * g.height());
}

TEST(SensorModel, ABeamLeavingTheGridStillClearsWhatItCrossed)
{
  GridConfig c = small_config();
  c.max_usable_range_m = 50.0;
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);
  for (int i = 0; i < 12; ++i) {g.integrateBeam(sensor, 0.0, 40.0);}

  int mx = 0;
  int my = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, mx, my));
  EXPECT_EQ(g.state(mx, my), CellState::kFree);
}

// ===================================================================
//  SCAN INTEGRATION
// ===================================================================

TEST(Scan, BeamIndexZeroIsNotStraightAhead)
{
  // For a scan spanning -pi..+pi, index 0 points BACKWARDS. Assuming index 0
  // is forward rotates the entire map by 180 degrees, which looks plausible
  // until it is compared against anything.
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);

  const int n = 360;
  std::vector<float> ranges(n, std::numeric_limits<float>::infinity());
  ranges[0] = 2.0f;                                    // at angle_min = -pi
  const double angle_min = -arb_core::pi<double>();
  const double increment = 2.0 * arb_core::pi<double>() / n;

  for (int i = 0; i < 12; ++i) {
    g.integrateScan(sensor, ranges, angle_min, increment, 0.12, 12.0);
  }

  int bx = 0;
  int by = 0;
  ASSERT_TRUE(g.worldToCell(-2.0, 0.0, bx, by));       // BEHIND the sensor
  EXPECT_EQ(g.state(bx, by), CellState::kOccupied);

  int fx = 0;
  int fy = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, fx, fy));
  EXPECT_NE(g.state(fx, fy), CellState::kOccupied);
}

TEST(Scan, AppliesTheSensorHeadingToEveryBearing)
{
  // integrateScan must add the sensor's heading to each beam's angle. A scan
  // integrated without it lands in the robot's BODY frame while the grid is in
  // the world frame: the map then rotates with the robot and every wall is
  // drawn in a different place each time the robot turns.
  //
  // The earlier tests all used a sensor at theta = 0, where the defect is
  // invisible, and a mutation that dropped the term survived them all.
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const double heading = arb_core::degToRad(90.0);
  const Pose2D sensor(0.0, 0.0, heading);

  // One beam straight ahead in the SENSOR frame.
  const int n = 8;
  std::vector<float> ranges(n, std::numeric_limits<float>::infinity());
  ranges[0] = 2.0f;
  const double angle_min = 0.0;
  const double increment = 0.05;

  for (int i = 0; i < 12; ++i) {
    g.integrateScan(sensor, ranges, angle_min, increment, 0.12, 12.0);
  }

  // Facing +y, so the hit belongs at (0, 2), not at (2, 0).
  int hx = 0;
  int hy = 0;
  ASSERT_TRUE(g.worldToCell(0.0, 2.0, hx, hy));
  EXPECT_EQ(g.state(hx, hy), CellState::kOccupied)
    << "the beam must land in the world frame, rotated by the sensor heading";

  int wrong_x = 0;
  int wrong_y = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, wrong_x, wrong_y));
  EXPECT_NE(g.state(wrong_x, wrong_y), CellState::kOccupied)
    << "and not in the body frame";
}

TEST(SensorModel, EvidenceIsRelativeToThePriorNotAbsolute)
{
  // The update is  l += l_sensor - l_prior, and the subtraction matters.
  //
  // With the usual uninformative prior of 0.5 the term is zero and dropping it
  // changes nothing, which is why the bug hides. With a NON-uniform prior --
  // an indoor map that starts biased towards "probably free", say -- omitting
  // it means every observation, including one that only confirms the prior,
  // pushes the cell further from it. Cells then drift purely from being looked
  // at.
  GridConfig c = small_config();
  c.p_prior = 0.4;            // "probably free" before any measurement
  c.p_miss = 0.4;             // a miss confirms the prior exactly
  c.log_odds_min = -1e9;
  c.log_odds_max = 1e9;
  OccupancyGrid g(c);

  const Pose2D sensor(0.0, 0.0, 0.0);
  int cx = 0;
  int cy = 0;
  ASSERT_TRUE(g.worldToCell(1.0, 0.0, cx, cy));
  const double before = g.logOdds(cx, cy);
  // 1e-6, not 1e-9: the grid stores log-odds as float to halve its memory
  // (a 24 x 18 m map at 5 cm is 172 800 cells, 691 KB as float against 1.4 MB
  // as double), and float carries about seven significant digits.
  EXPECT_NEAR(before, OccupancyGrid::probabilityToLogOdds(0.4), 1e-6)
    << "an unobserved cell must start AT the prior, not at zero";

  // 50 beams that pass through, each carrying exactly the prior's evidence.
  for (int i = 0; i < 50; ++i) {g.integrateBeam(sensor, 0.0, 3.0);}
  EXPECT_NEAR(g.logOdds(cx, cy), before, 1e-6)
    << "a measurement that only confirms the prior must not move the cell";
}

TEST(Scan, CountsOnlyTheBeamsItUsed)
{
  GridConfig c = small_config();
  c.min_usable_range_m = 0.2;
  OccupancyGrid g(c);
  std::vector<float> ranges = {2.0f, 0.05f, 3.0f, std::nanf(""), 1.0f};
  const int used = g.integrateScan(
    Pose2D(0, 0, 0), ranges, -0.2, 0.1, 0.12, 12.0);
  // 0.05 is below range_min and skipped; NaN becomes a no-return and is used.
  EXPECT_EQ(used, 4);
}

// ===================================================================
//  OUTPUT FORMAT
// ===================================================================

TEST(Output, UnknownIsMinusOneNotFiftyPercent)
{
  // nav_msgs/OccupancyGrid uses -1 for unknown. Publishing 50 instead tells
  // every consumer "this cell is 50 % likely to be occupied", which a planner
  // reads as traversable-but-risky rather than never-observed.
  OccupancyGrid g(small_config());
  const auto bytes = g.toOccupancyBytes();
  ASSERT_EQ(bytes.size(), static_cast<std::size_t>(g.width() * g.height()));
  for (const auto b : bytes) {EXPECT_EQ(b, -1);}
}

TEST(Output, OccupiedAndFreeCellsMapIntoTheExpectedRanges)
{
  GridConfig c = small_config();
  OccupancyGrid g(c);
  const Pose2D sensor(0.0, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {g.integrateBeam(sensor, 0.0, 2.0);}

  int hx = 0;
  int hy = 0;
  ASSERT_TRUE(g.worldToCell(2.0, 0.0, hx, hy));
  int mx = 0;
  int my = 0;
  ASSERT_TRUE(g.worldToCell(1.0, 0.0, mx, my));

  const auto bytes = g.toOccupancyBytes();
  const auto at = [&](int x, int y) {
      return bytes[static_cast<std::size_t>(y) * g.width() + x];
    };
  EXPECT_GE(at(hx, hy), 70);
  EXPECT_GE(at(mx, my), 0);
  EXPECT_LE(at(mx, my), 40);
}

TEST(Output, RowMajorLayoutMatchesTheMessageConvention)
{
  // nav_msgs/OccupancyGrid is row-major with y as the slow axis. Transposing
  // it produces a map that looks like a plausible but mirrored world.
  OccupancyGrid g(small_config());
  const Pose2D sensor(0.0, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    g.integrateBeam(sensor, arb_core::degToRad(90.0), 2.0);
  }
  int hx = 0;
  int hy = 0;
  ASSERT_TRUE(g.worldToCell(0.0, 2.0, hx, hy));

  const auto bytes = g.toOccupancyBytes();
  EXPECT_GE(bytes[static_cast<std::size_t>(hy) * g.width() + hx], 70);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
