// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#include "arb_planning/grid_planner.hpp"

using arb_planning::GridIndex;
using arb_planning::GridPlanner;
using arb_planning::Heuristic;
using arb_planning::PlannerConfig;
using arb_planning::PlanResult;

namespace
{

constexpr double kSqrt2 = 1.41421356237309504880;

/// A planner with no inflation and no soft cost, so path costs are exactly the
/// geometric ones and optimality can be asserted to the digit.
PlannerConfig bare_config()
{
  PlannerConfig c;
  c.inflation_radius_m = 0.0;
  c.cost_decay_radius_m = 0.0;
  c.max_soft_cost = 0.0;
  c.unknown_is_lethal = true;
  return c;
}

/// An all-free grid.
std::vector<std::int8_t> empty_grid(int w, int h)
{
  return std::vector<std::int8_t>(static_cast<std::size_t>(w) * h, 0);
}

void block(std::vector<std::int8_t> & g, int w, int x, int y)
{
  g[static_cast<std::size_t>(y) * w + x] = 100;
}

}  // namespace

// =====================================================================
//  HEURISTICS
// =====================================================================

TEST(Heuristics, OctileEqualsTheTrueCostOnAnEmptyGrid)
{
  // The defining property of a TIGHT heuristic: on an empty 8-connected grid
  // it is not an estimate at all, it is the answer. That is why A* with octile
  // expands almost nothing there.
  GridPlanner p(bare_config());
  const int n = 40;
  p.setMap(empty_grid(n, n), n, n, 0.05);

  for (const auto & goal : std::vector<GridIndex>{
      {35, 5}, {5, 35}, {30, 30}, {39, 20}, {20, 39}})
  {
    const PlanResult r = p.plan({5, 5}, goal);
    ASSERT_TRUE(r.success) << r.failure_reason;
    const double h = GridPlanner::heuristic(Heuristic::kOctile, {5, 5}, goal);
    EXPECT_NEAR(r.length_cells, h, 1e-9)
      << "goal (" << goal.x << ", " << goal.y << ")";
  }
}

TEST(Heuristics, EuclideanIsAdmissibleButLooserThanOctile)
{
  // Admissible means "never above the truth". Loose means "sometimes well
  // below it", which costs expansions without costing optimality.
  for (int dx = 0; dx <= 20; ++dx) {
    for (int dy = 0; dy <= 20; ++dy) {
      const GridIndex a{0, 0};
      const GridIndex b{dx, dy};
      const double truth = GridPlanner::heuristic(Heuristic::kOctile, a, b);
      const double euclid = GridPlanner::heuristic(Heuristic::kEuclidean, a, b);
      EXPECT_LE(euclid, truth + 1e-12) << "dx=" << dx << " dy=" << dy;
    }
  }
  // Strictly looser somewhere, or it would be the same heuristic.
  EXPECT_LT(GridPlanner::heuristic(Heuristic::kEuclidean, {0, 0}, {10, 5}),
            GridPlanner::heuristic(Heuristic::kOctile, {0, 0}, {10, 5}));
}

TEST(Heuristics, ManhattanIsInadmissibleWithDiagonalMoves)
{
  // The single most common planning bug. Manhattan is the right heuristic for
  // a 4-connected grid and WRONG for an 8-connected one: a pure diagonal costs
  // 1.41d and Manhattan claims 2d.
  const GridIndex a{0, 0};
  const GridIndex b{10, 10};
  EXPECT_NEAR(GridPlanner::heuristic(Heuristic::kOctile, a, b), 10 * kSqrt2, 1e-9);
  EXPECT_NEAR(GridPlanner::heuristic(Heuristic::kManhattan, a, b), 20.0, 1e-9);
  EXPECT_GT(GridPlanner::heuristic(Heuristic::kManhattan, a, b),
            GridPlanner::heuristic(Heuristic::kOctile, a, b))
    << "an inadmissible heuristic overestimates, and A* then returns a "
       "plausible but non-optimal path";
}

TEST(Heuristics, ZeroTurnsAStarIntoDijkstra)
{
  EXPECT_DOUBLE_EQ(GridPlanner::heuristic(Heuristic::kZero, {0, 0}, {50, 50}), 0.0);
}

TEST(Heuristics, AreSymmetricAndZeroAtTheGoal)
{
  for (const auto h : {Heuristic::kOctile, Heuristic::kEuclidean,
      Heuristic::kManhattan})
  {
    EXPECT_DOUBLE_EQ(GridPlanner::heuristic(h, {7, 3}, {7, 3}), 0.0);
    EXPECT_NEAR(GridPlanner::heuristic(h, {1, 2}, {9, 8}),
                GridPlanner::heuristic(h, {9, 8}, {1, 2}), 1e-12);
  }
}

// =====================================================================
//  CORRECTNESS
// =====================================================================

TEST(Planner, FindsTheOptimalPathAroundAWall)
{
  // A wall with a single gap. The optimal route is forced, so the cost is
  // computable by hand and the planner has nowhere to hide.
  const int w = 21;
  const int h = 21;
  auto grid = empty_grid(w, h);
  for (int y = 0; y < h; ++y) {
    if (y == 10) {continue;}          // the gap
    block(grid, w, 10, y);
  }

  GridPlanner p(bare_config());
  p.setMap(grid, w, h, 0.05);
  const PlanResult r = p.plan({2, 10}, {18, 10});
  ASSERT_TRUE(r.success) << r.failure_reason;

  // Straight through the gap: 16 cells.
  EXPECT_NEAR(r.length_cells, 16.0, 1e-9);
  for (const auto & c : r.path) {
    EXPECT_FALSE(p.isLethal(c)) << "path enters an obstacle at ("
                                << c.x << ", " << c.y << ")";
  }
}

TEST(Planner, AStarAndDijkstraAgreeOnCost)
{
  // The definitive optimality check. Dijkstra is optimal by construction; if
  // A* with an admissible heuristic disagrees with it, A* is wrong.
  const int w = 60;
  const int h = 45;
  auto grid = empty_grid(w, h);
  for (int y = 5; y < 35; ++y) {block(grid, w, 20, y);}
  for (int y = 12; y < 44; ++y) {block(grid, w, 38, y);}
  for (int x = 25; x < 50; ++x) {block(grid, w, x, 25);}

  PlannerConfig c = bare_config();
  GridPlanner dijkstra(c);
  dijkstra.setMap(grid, w, h, 0.05);
  dijkstra.mutableConfig().heuristic = Heuristic::kZero;

  for (const auto hh : {Heuristic::kOctile, Heuristic::kEuclidean}) {
    GridPlanner astar(c);
    astar.setMap(grid, w, h, 0.05);
    astar.mutableConfig().heuristic = hh;

    const PlanResult a = astar.plan({2, 2}, {57, 42});
    const PlanResult d = dijkstra.plan({2, 2}, {57, 42});
    ASSERT_TRUE(a.success) << a.failure_reason;
    ASSERT_TRUE(d.success) << d.failure_reason;
    EXPECT_NEAR(a.cost, d.cost, 1e-9)
      << "A* with " << arb_planning::toString(hh)
      << " found cost " << a.cost << ", Dijkstra " << d.cost;
  }
}

TEST(Planner, AStarExpandsFarFewerNodesThanDijkstra)
{
  // The entire point of the heuristic. Same answer, less work.
  const int w = 80;
  const int h = 80;
  const auto grid = empty_grid(w, h);

  PlannerConfig c = bare_config();
  GridPlanner astar(c);
  astar.setMap(grid, w, h, 0.05);
  GridPlanner dijkstra(c);
  dijkstra.setMap(grid, w, h, 0.05);
  dijkstra.mutableConfig().heuristic = Heuristic::kZero;

  const PlanResult a = astar.plan({5, 5}, {74, 74});
  const PlanResult d = dijkstra.plan({5, 5}, {74, 74});
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(d.success);
  EXPECT_NEAR(a.cost, d.cost, 1e-9);
  EXPECT_LT(a.nodes_expanded, d.nodes_expanded / 4)
    << "A* expanded " << a.nodes_expanded << ", Dijkstra " << d.nodes_expanded;
}

TEST(Planner, AnInadmissibleHeuristicLosesTheOptimalityGuarantee)
{
  // The claim is NOT "Manhattan always returns a worse path". On easy
  // instances it often finds the optimum anyway, and an earlier version of
  // this test asserted strict suboptimality on one such instance and failed.
  //
  // The claim that matters is the one A* actually makes: with an admissible
  // heuristic the result is optimal, and with an inadmissible one it is not
  // guaranteed to be. A guarantee is falsified by a single counterexample, so
  // the test sweeps a set of instances and requires that at least one of them
  // is suboptimal -- and that none is ever BETTER than optimal, which would
  // mean the reference itself is wrong.
  //
  // This is why the failure is dangerous in practice: most of the time the
  // path looks fine.
  const int w = 60;
  const int h = 60;

  PlannerConfig c = bare_config();
  int suboptimal = 0;
  int instances = 0;
  double worst_excess = 0.0;

  // A deterministic family of maps: a wall with a gap that moves, which is
  // enough to make the diagonal-heavy detour optimal on some of them.
  for (int gap = 4; gap < 56; gap += 3) {
    auto grid = empty_grid(w, h);
    for (int y = 0; y < h; ++y) {
      if (y >= gap && y < gap + 4) {continue;}
      block(grid, w, 30, y);
    }
    for (int x = 10; x < 45; ++x) {
      if (x >= 24 && x < 30) {continue;}
      block(grid, w, x, 45);
    }

    GridPlanner optimal(c);
    optimal.setMap(grid, w, h, 0.05);
    GridPlanner bad(c);
    bad.setMap(grid, w, h, 0.05);
    bad.mutableConfig().heuristic = Heuristic::kManhattan;

    const PlanResult a = optimal.plan({3, 55}, {56, 3});
    const PlanResult m = bad.plan({3, 55}, {56, 3});
    if (!a.success || !m.success) {continue;}
    ++instances;

    // Manhattan can never beat the optimum; if it does, the reference is wrong.
    EXPECT_GE(m.cost, a.cost - 1e-9)
      << "gap at " << gap << ": Manhattan " << m.cost << " beat optimal " << a.cost;
    if (m.cost > a.cost + 1e-6) {
      ++suboptimal;
      worst_excess = std::max(worst_excess, 100.0 * (m.cost / a.cost - 1.0));
    }
  }

  ASSERT_GT(instances, 5) << "the sweep produced too few solvable instances";
  EXPECT_GT(suboptimal, 0)
    << "Manhattan matched the optimum on all " << instances
    << " instances; the sweep is not hard enough to expose the lost guarantee";
  RecordProperty("suboptimal_instances", suboptimal);
  RecordProperty("instances", instances);
  std::cout << "  [ INFO ] Manhattan was suboptimal on " << suboptimal << " of "
            << instances << " instances, worst excess " << worst_excess
            << " %\n";
}

TEST(Planner, AnInadmissibleHeuristicIsFasterWhichIsWhyPeopleUseIt)
{
  const int w = 80;
  const int h = 80;
  auto grid = empty_grid(w, h);
  for (int y = 10; y < 70; ++y) {block(grid, w, 40, y);}

  PlannerConfig c = bare_config();
  GridPlanner optimal(c);
  optimal.setMap(grid, w, h, 0.05);
  GridPlanner bad(c);
  bad.setMap(grid, w, h, 0.05);
  bad.mutableConfig().heuristic = Heuristic::kManhattan;

  const PlanResult a = optimal.plan({4, 40}, {75, 4});
  const PlanResult m = bad.plan({4, 40}, {75, 4});
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(m.success);
  EXPECT_LT(m.nodes_expanded, a.nodes_expanded)
    << "octile " << a.nodes_expanded << ", manhattan " << m.nodes_expanded;
}

TEST(Planner, PathIsContiguousAndStartsAndEndsWhereAsked)
{
  const int w = 50;
  const int h = 50;
  auto grid = empty_grid(w, h);
  for (int y = 0; y < 40; ++y) {block(grid, w, 25, y);}

  GridPlanner p(bare_config());
  p.setMap(grid, w, h, 0.05);
  const GridIndex start{3, 3};
  const GridIndex goal{46, 46};
  const PlanResult r = p.plan(start, goal);
  ASSERT_TRUE(r.success);

  EXPECT_EQ(r.path.front(), start);
  EXPECT_EQ(r.path.back(), goal);
  for (std::size_t i = 1; i < r.path.size(); ++i) {
    const int dx = std::abs(r.path[i].x - r.path[i - 1].x);
    const int dy = std::abs(r.path[i].y - r.path[i - 1].y);
    EXPECT_LE(std::max(dx, dy), 1) << "jump at step " << i;
    EXPECT_GT(dx + dy, 0) << "repeated cell at step " << i;
  }
  std::set<std::pair<int, int>> unique;
  for (const auto & c : r.path) {unique.insert({c.x, c.y});}
  EXPECT_EQ(unique.size(), r.path.size()) << "the path revisits a cell";
}

TEST(Planner, DiagonalMovesDoNotCutCorners)
{
  // Two obstacles meeting at a diagonal leave a gap a POINT can slip through
  // and a disc cannot. Allowing it produces geometrically shortest and
  // physically impossible paths.
  const int w = 11;
  const int h = 11;
  auto grid = empty_grid(w, h);
  block(grid, w, 5, 4);
  block(grid, w, 4, 5);

  GridPlanner p(bare_config());
  p.setMap(grid, w, h, 0.05);
  const PlanResult r = p.plan({4, 4}, {5, 5});
  ASSERT_TRUE(r.success) << r.failure_reason;
  // The direct diagonal (4,4) -> (5,5) squeezes between the two obstacles and
  // must be refused, so the path has to go the long way round.
  EXPECT_GT(r.path.size(), 2u) << "the planner cut the corner";
}

TEST(Planner, FailsCleanlyWhenThereIsNoPath)
{
  const int w = 20;
  const int h = 20;
  auto grid = empty_grid(w, h);
  for (int y = 0; y < h; ++y) {block(grid, w, 10, y);}

  GridPlanner p(bare_config());
  p.setMap(grid, w, h, 0.05);
  const PlanResult r = p.plan({2, 10}, {17, 10});
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.failure_reason, "no path exists");
  EXPECT_TRUE(r.path.empty());
  EXPECT_GT(r.nodes_expanded, 0) << "it must actually have searched";
}

TEST(Planner, RejectsStartOrGoalInsideAnObstacle)
{
  const int w = 20;
  const int h = 20;
  auto grid = empty_grid(w, h);
  block(grid, w, 5, 5);
  block(grid, w, 15, 15);

  GridPlanner p(bare_config());
  p.setMap(grid, w, h, 0.05);
  EXPECT_EQ(p.plan({5, 5}, {10, 10}).failure_reason, "start is in an obstacle");
  EXPECT_EQ(p.plan({10, 10}, {15, 15}).failure_reason, "goal is in an obstacle");
  EXPECT_EQ(p.plan({-1, 0}, {10, 10}).failure_reason, "start outside the map");
  EXPECT_EQ(p.plan({10, 10}, {0, 999}).failure_reason, "goal outside the map");
}

TEST(Planner, HandlesAStartEqualToTheGoal)
{
  GridPlanner p(bare_config());
  p.setMap(empty_grid(20, 20), 20, 20, 0.05);
  const PlanResult r = p.plan({7, 7}, {7, 7});
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.path.size(), 1u);
  EXPECT_NEAR(r.cost, 0.0, 1e-12);
}

TEST(Planner, ReportsNoMapRatherThanCrashing)
{
  GridPlanner p(bare_config());
  EXPECT_EQ(p.plan({0, 0}, {1, 1}).failure_reason, "no map");
}

// =====================================================================
//  COSTMAP
// =====================================================================

TEST(Costmap, InflatesObstaclesByTheRobotRadius)
{
  // Inflating lets the planner treat the robot as a POINT, which is exactly
  // equivalent to planning for a disc in the real world and enormously cheaper
  // than checking a footprint at every cell.
  PlannerConfig c = bare_config();
  c.inflation_radius_m = 0.15;        // 3 cells at 5 cm
  GridPlanner p(c);
  auto grid = empty_grid(31, 31);
  block(grid, 31, 15, 15);
  p.setMap(grid, 31, 31, 0.05);

  EXPECT_TRUE(p.isLethal({15, 15}));
  EXPECT_TRUE(p.isLethal({15, 17})) << "2 cells away must still be lethal";
  EXPECT_FALSE(p.isLethal({15, 22})) << "7 cells away must be free";
}

TEST(Costmap, SoftCostDecaysWithDistanceSoPathsPreferTheMiddle)
{
  // Without it the shortest path hugs every wall, leaving the local planner no
  // room and turning any control error into a collision.
  PlannerConfig c;
  c.inflation_radius_m = 0.10;
  c.cost_decay_radius_m = 0.50;
  GridPlanner p(c);
  auto grid = empty_grid(41, 41);
  for (int x = 0; x < 41; ++x) {block(grid, 41, x, 0);}
  p.setMap(grid, 41, 41, 0.05);

  double previous = p.softCost({20, 3});
  EXPECT_GT(previous, 0.0);
  for (int y = 4; y < 12; ++y) {
    const double cost = p.softCost({20, y});
    EXPECT_LE(cost, previous + 1e-9) << "cost rose at y=" << y;
    previous = cost;
  }
  EXPECT_NEAR(p.softCost({20, 20}), 0.0, 1e-9) << "open space must be free";
}

TEST(Costmap, APathThroughACorridorStaysAwayFromTheWalls)
{
  PlannerConfig c;
  c.inflation_radius_m = 0.10;
  c.cost_decay_radius_m = 0.40;
  c.cost_scaling = 8.0;
  GridPlanner p(c);

  const int w = 60;
  const int h = 21;
  auto grid = empty_grid(w, h);
  for (int x = 0; x < w; ++x) {
    block(grid, w, x, 0);
    block(grid, w, x, h - 1);
  }
  p.setMap(grid, w, h, 0.05);

  const PlanResult r = p.plan({5, 10}, {54, 10});
  ASSERT_TRUE(r.success) << r.failure_reason;
  int worst = 0;
  for (const auto & cell : r.path) {
    worst = std::max(worst, std::abs(cell.y - 10));
  }
  EXPECT_LE(worst, 2) << "the path wandered " << worst
                      << " cells off the corridor centreline";
}

TEST(Costmap, UnknownCellsAreLethalButDoNotRadiateCost)
{
  // Unknown space must be impassable without pushing soft cost into the known
  // free space beside it, or the frontier of the map becomes a wall and the
  // robot refuses to approach anywhere it has not already been.
  PlannerConfig c;
  c.unknown_is_lethal = true;
  c.inflation_radius_m = 0.10;
  c.cost_decay_radius_m = 0.50;
  GridPlanner p(c);

  std::vector<std::int8_t> grid(31 * 31, 0);
  for (int y = 0; y < 31; ++y) {
    for (int x = 20; x < 31; ++x) {grid[static_cast<std::size_t>(y) * 31 + x] = -1;}
  }
  p.setMap(grid, 31, 31, 0.05);

  EXPECT_TRUE(p.isLethal({25, 15}));
  EXPECT_FALSE(p.isLethal({18, 15})) << "known free space beside the frontier";
  EXPECT_NEAR(p.softCost({18, 15}), 0.0, 1e-9)
    << "the map frontier must not become a wall of soft cost";
}

TEST(Costmap, UnknownCanBeMadeTraversable)
{
  PlannerConfig c = bare_config();
  c.unknown_is_lethal = false;
  GridPlanner p(c);
  std::vector<std::int8_t> grid(21 * 21, -1);
  p.setMap(grid, 21, 21, 0.05);
  EXPECT_FALSE(p.isLethal({10, 10}));
  EXPECT_TRUE(p.plan({1, 1}, {19, 19}).success);
}

// =====================================================================
//  ACCOUNTING
// =====================================================================

TEST(Planner, ReportsConsistentSearchStatistics)
{
  GridPlanner p(bare_config());
  p.setMap(empty_grid(60, 60), 60, 60, 0.05);
  const PlanResult r = p.plan({2, 2}, {57, 57});
  ASSERT_TRUE(r.success);
  EXPECT_GT(r.nodes_expanded, 0);
  EXPECT_GE(r.nodes_generated, r.nodes_expanded)
    << "a node cannot be expanded without first being generated";
  EXPECT_GT(r.open_set_peak, 0);
  EXPECT_GT(r.planning_time_ms, 0.0);
  EXPECT_LE(r.nodes_expanded, 60 * 60)
    << "a consistent heuristic must not expand a cell twice";
}

TEST(Planner, WeightedAStarTradesOptimalityForSpeed)
{
  // Weighted A* returns a path at most `weight` times optimal and gets there
  // faster. Worth measuring rather than asserting.
  const int w = 70;
  const int h = 70;
  auto grid = empty_grid(w, h);
  for (int y = 10; y < 60; ++y) {block(grid, w, 35, y);}

  PlannerConfig c = bare_config();
  GridPlanner optimal(c);
  optimal.setMap(grid, w, h, 0.05);
  GridPlanner weighted(c);
  weighted.setMap(grid, w, h, 0.05);
  weighted.mutableConfig().heuristic_weight = 3.0;

  const PlanResult a = optimal.plan({3, 35}, {66, 5});
  const PlanResult b = weighted.plan({3, 35}, {66, 5});
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  EXPECT_LE(b.cost, 3.0 * a.cost + 1e-6) << "the weight bounds the suboptimality";
  EXPECT_LT(b.nodes_expanded, a.nodes_expanded);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
