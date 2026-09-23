// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_planning/grid_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace arb_planning
{
namespace
{
constexpr double kSqrt2 = 1.41421356237309504880;
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

std::string toString(Heuristic h)
{
  switch (h) {
    case Heuristic::kOctile: return "octile";
    case Heuristic::kEuclidean: return "euclidean";
    case Heuristic::kManhattan: return "manhattan";
    case Heuristic::kZero: return "none";
  }
  return "unknown";
}

GridPlanner::GridPlanner(const PlannerConfig & config)
: config_(config) {}

double GridPlanner::heuristic(Heuristic type, const GridIndex & a, const GridIndex & b)
{
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  switch (type) {
    case Heuristic::kOctile:
      // (sqrt(2) - 1) * min + max: take min(dx,dy) diagonal steps, the rest
      // straight. Exactly the true cost on an empty 8-connected grid, so it is
      // both admissible and as tight as a heuristic can be.
      return (kSqrt2 - 1.0) * std::min(dx, dy) + std::max(dx, dy);
    case Heuristic::kEuclidean:
      return std::hypot(dx, dy);
    case Heuristic::kManhattan:
      // INADMISSIBLE with diagonal moves: claims 2d for a pure diagonal whose
      // true cost is 1.41d. Kept so the benchmark can measure what that costs.
      return dx + dy;
    case Heuristic::kZero:
      return 0.0;
  }
  return 0.0;
}

double GridPlanner::pathLength(const std::vector<GridIndex> & path)
{
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const int dx = std::abs(path[i].x - path[i - 1].x);
    const int dy = std::abs(path[i].y - path[i - 1].y);
    total += (dx != 0 && dy != 0) ? kSqrt2 : 1.0;
  }
  return total;
}

void GridPlanner::setMap(
  const std::vector<std::int8_t> & occupancy, int width, int height, double resolution)
{
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  const std::size_t n = static_cast<std::size_t>(width) * height;
  lethal_.assign(n, 0);
  cost_.assign(n, 0.0f);
  if (n == 0 || occupancy.size() < n || resolution <= 0.0) {return;}

  // --- seed: cells that are obstacles before inflation --------------------
  std::vector<GridIndex> seeds;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const std::int8_t v = occupancy[static_cast<std::size_t>(y) * width_ + x];
      const bool blocked = (v >= config_.lethal_threshold) ||
        (v < 0 && config_.unknown_is_lethal);
      if (blocked) {
        lethal_[static_cast<std::size_t>(y) * width_ + x] = 1;
        // Only genuine obstacles radiate inflation. Unknown cells are
        // impassable but must NOT push cost into the known free space beside
        // them, or the frontier of the map becomes a wall of soft cost and the
        // robot refuses to approach anywhere it has not already been.
        if (v >= config_.lethal_threshold) {seeds.push_back({x, y});}
      }
    }
  }

  // --- distance transform by BFS from every obstacle ----------------------
  //
  // A multi-source BFS over the 8-connected grid gives each free cell its
  // distance to the nearest obstacle in one pass over the map, rather than
  // the O(obstacles * radius^2) splat a naive implementation uses. On a
  // 480 x 360 grid with thousands of obstacle cells that is the difference
  // between milliseconds and a second per map update.
  const double max_radius = std::max(config_.inflation_radius_m,
                                     config_.cost_decay_radius_m);
  std::vector<float> distance(n, std::numeric_limits<float>::infinity());
  std::queue<GridIndex> q;
  for (const auto & s : seeds) {
    distance[index(s)] = 0.0f;
    q.push(s);
  }

  const int max_cells = static_cast<int>(std::ceil(max_radius / resolution_)) + 1;
  while (!q.empty()) {
    const GridIndex c = q.front();
    q.pop();
    const float d = distance[index(c)];
    if (d >= max_cells) {continue;}
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {continue;}
        const GridIndex nb{c.x + dx, c.y + dy};
        if (!inBounds(nb)) {continue;}
        // Chebyshev-style BFS: a diagonal neighbour is one step. This slightly
        // over-estimates the true Euclidean distance on diagonals, which makes
        // the inflation marginally conservative -- the safe direction.
        const float nd = d + 1.0f;
        if (nd < distance[index(nb)]) {
          distance[index(nb)] = nd;
          q.push(nb);
        }
      }
    }
  }

  // --- inflation and soft cost -------------------------------------------
  for (std::size_t i = 0; i < n; ++i) {
    if (lethal_[i]) {
      cost_[i] = static_cast<float>(config_.max_soft_cost);
      continue;
    }
    const double metres = distance[i] * resolution_;
    if (metres <= config_.inflation_radius_m) {
      // Within the robot's own radius of an obstacle: the robot cannot be
      // here, whatever the grid says about the cell itself.
      lethal_[i] = 1;
      cost_[i] = static_cast<float>(config_.max_soft_cost);
    } else if (metres < config_.cost_decay_radius_m) {
      // Exponential decay with distance. The planner still CAN pass close to a
      // wall, it just prefers not to, which is what keeps a corridor path in
      // the middle instead of scraping along one side.
      const double excess = metres - config_.inflation_radius_m;
      cost_[i] = static_cast<float>(
        config_.max_soft_cost * std::exp(-config_.cost_scaling * excess));
    }
  }
}

bool GridPlanner::isLethal(const GridIndex & c) const
{
  if (!inBounds(c)) {return true;}
  return lethal_[index(c)] != 0;
}

double GridPlanner::softCost(const GridIndex & c) const
{
  if (!inBounds(c)) {return 0.0;}
  return cost_[index(c)];
}

PlanResult GridPlanner::plan(const GridIndex & start, const GridIndex & goal) const
{
  const auto t0 = std::chrono::steady_clock::now();
  PlanResult result;

  auto finish = [&](const std::string & reason) {
      result.failure_reason = reason;
      result.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return result;
    };

  if (width_ <= 0 || height_ <= 0) {return finish("no map");}
  if (!inBounds(start)) {return finish("start outside the map");}
  if (!inBounds(goal)) {return finish("goal outside the map");}
  if (isLethal(start)) {return finish("start is in an obstacle");}
  if (isLethal(goal)) {return finish("goal is in an obstacle");}

  const std::size_t n = static_cast<std::size_t>(width_) * height_;
  std::vector<double> g(n, kInf);
  std::vector<int> parent(n, -1);
  std::vector<std::uint8_t> closed(n, 0);

  struct Node
  {
    double f;
    double g;
    int index;
  };
  // Tie-break on LARGER g when f is equal. Two nodes with the same f are
  // equally promising, but the one further from the start is closer to the
  // goal, so expanding it first drives towards the goal instead of fanning out
  // through the ring of equal-f cells. On an open grid this is the difference
  // between expanding a corridor and expanding a disc.
  const auto worse = [](const Node & a, const Node & b) {
      if (a.f != b.f) {return a.f > b.f;}
      return a.g < b.g;
    };
  std::priority_queue<Node, std::vector<Node>, decltype(worse)> open(worse);

  const std::size_t start_i = index(start);
  g[start_i] = 0.0;
  open.push({config_.heuristic_weight *
      heuristic(config_.heuristic, start, goal), 0.0, static_cast<int>(start_i)});
  result.nodes_generated = 1;

  const std::size_t goal_i = index(goal);
  bool found = false;

  while (!open.empty()) {
    result.open_set_peak = std::max(result.open_set_peak,
                                    static_cast<int>(open.size()));
    const Node current = open.top();
    open.pop();
    const std::size_t ci = static_cast<std::size_t>(current.index);

    // A stale entry: the same cell was pushed again with a better g before
    // this copy came off the queue. std::priority_queue has no decrease-key,
    // so the standard approach is to push duplicates and skip the outdated
    // ones here. Checking `closed` alone is not enough -- the check on g is
    // what discards a duplicate that has not been expanded yet.
    if (closed[ci]) {continue;}
    if (current.g > g[ci] + 1e-12) {continue;}

    closed[ci] = 1;
    ++result.nodes_expanded;

    if (ci == goal_i) {found = true; break;}
    if (result.nodes_expanded >= config_.max_expansions) {
      return finish("expansion limit reached");
    }

    const GridIndex c{static_cast<int>(ci % width_), static_cast<int>(ci / width_)};
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {continue;}
        const bool diagonal = (dx != 0 && dy != 0);
        if (diagonal && !config_.allow_diagonal) {continue;}

        const GridIndex nb{c.x + dx, c.y + dy};
        if (!inBounds(nb) || isLethal(nb)) {continue;}

        // A diagonal move must not CUT A CORNER between two obstacles. The
        // robot is a disc, and a gap that is diagonal-only is not a gap it can
        // fit through; allowing it produces paths that are geometrically
        // shortest and physically impossible.
        if (diagonal &&
            (isLethal({c.x + dx, c.y}) || isLethal({c.x, c.y + dy})))
        {
          continue;
        }

        const std::size_t ni = index(nb);
        if (closed[ni]) {continue;}

        const double step = diagonal ? kSqrt2 : 1.0;
        const double tentative = g[ci] + step + softCost(nb);
        if (tentative + 1e-12 >= g[ni]) {continue;}

        g[ni] = tentative;
        parent[ni] = static_cast<int>(ci);
        const double h = config_.heuristic_weight *
          heuristic(config_.heuristic, nb, goal);
        open.push({tentative + h, tentative, static_cast<int>(ni)});
        ++result.nodes_generated;
      }
    }
  }

  if (!found) {return finish("no path exists");}

  for (int i = static_cast<int>(goal_i); i != -1; i = parent[i]) {
    result.path.push_back({i % width_, i / width_});
  }
  std::reverse(result.path.begin(), result.path.end());
  result.cost = g[goal_i];
  result.length_cells = pathLength(result.path);
  result.success = true;
  return finish("");
}

}  // namespace arb_planning
