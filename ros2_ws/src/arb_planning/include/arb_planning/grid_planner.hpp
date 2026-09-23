// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_PLANNING__GRID_PLANNER_HPP_
#define ARB_PLANNING__GRID_PLANNER_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace arb_planning
{

/// One cell of the planning costmap.
struct GridIndex
{
  int x{0};
  int y{0};
  bool operator==(const GridIndex & o) const {return x == o.x && y == o.y;}
  bool operator!=(const GridIndex & o) const {return !(*this == o);}
};

/// Which distance function to use as the heuristic.
///
/// ===================== ADMISSIBILITY IS NOT OPTIONAL =====================
///
/// A* returns an OPTIMAL path if and only if the heuristic never overestimates
/// the true remaining cost. That property is called admissibility, and losing
/// it does not produce an error -- it produces a plausible, slightly wrong
/// path, which is far harder to notice.
///
/// On an 8-connected grid a diagonal step costs sqrt(2) and moves one cell in
/// both axes. So to cover dx and dy cells the true minimum cost is
///
///     octile(dx, dy) = (sqrt(2) - 1) * min(dx, dy) + max(dx, dy)
///
/// take min(dx,dy) diagonal steps and the rest straight.
///
///   * kOctile    -- exactly that. Admissible AND tight: it equals the true
///                   cost on an empty grid, so A* expands almost nothing.
///   * kEuclidean -- sqrt(dx^2 + dy^2). Admissible, because the straight line
///                   is never longer than any grid path, but LOOSE: on a pure
///                   diagonal it returns 1.41*d where the truth is 1.41*d, and
///                   on a pure straight line it returns d where the truth is
///                   d -- but in between it underestimates, so A* explores
///                   more than it needs to.
///   * kManhattan -- |dx| + |dy|. INADMISSIBLE on an 8-connected grid: for a
///                   pure diagonal it claims 2d against a true cost of 1.41d.
///                   A* with it is fast and returns paths that are not
///                   shortest. Included precisely so the benchmark can show
///                   that, because it is the single most common planning bug.
///   * kZero      -- no heuristic at all, which turns A* into Dijkstra.
enum class Heuristic
{
  kOctile,
  kEuclidean,
  kManhattan,
  kZero,
};

std::string toString(Heuristic h);

struct PlannerConfig
{
  /// Cells at or above this occupancy value are impassable.
  int lethal_threshold{65};

  /// Unknown cells (-1). Treating them as free lets the planner route through
  /// space it has never seen; treating them as lethal stops it leaving the
  /// region it has mapped. Neither is right in general, which is why it is a
  /// parameter rather than a decision buried in the code.
  bool unknown_is_lethal{true};

  /// Obstacles are grown by the robot's radius so the planner can treat the
  /// robot as a POINT. Planning for a point in an inflated world is exactly
  /// equivalent to planning for a disc in the real one, and enormously
  /// cheaper than checking the footprint at every cell.
  double inflation_radius_m{0.28};

  /// Beyond the lethal radius, a decaying cost that makes the planner prefer
  /// the middle of a corridor. Without it the shortest path hugs every wall,
  /// which leaves no room for the local planner to avoid anything and turns
  /// every control error into a collision.
  double cost_decay_radius_m{0.55};
  double cost_scaling{12.0};
  double max_soft_cost{2.5};

  /// Allow diagonal moves. With them the path is shorter and smoother; the
  /// cost is that a diagonal must not cut a corner between two obstacles.
  bool allow_diagonal{true};

  Heuristic heuristic{Heuristic::kOctile};

  /// Multiplier on the heuristic. 1.0 is admissible; above that A* becomes
  /// "weighted A*", which is faster and returns paths at most `weight` times
  /// optimal. Exposed so the benchmark can measure that trade rather than
  /// assert it.
  double heuristic_weight{1.0};

  /// Hard cap on expansions, so a planner asked for an unreachable goal fails
  /// in bounded time instead of exploring the whole map.
  int max_expansions{2000000};
};

struct PlanResult
{
  bool success{false};
  std::string failure_reason;
  std::vector<GridIndex> path;     ///< start .. goal inclusive, in cells
  double cost{0.0};                ///< total path cost including soft costs
  double length_cells{0.0};        ///< geometric length, diagonals as sqrt(2)
  int nodes_expanded{0};           ///< popped from the open set
  int nodes_generated{0};          ///< pushed into the open set
  int open_set_peak{0};
  double planning_time_ms{0.0};
};

/// A* / Dijkstra on an occupancy grid.
///
/// ========================== HOW A* WORKS ==========================
///
/// Every cell carries two numbers:
///
///     g(n)  -- the best cost found so far to REACH n from the start
///     h(n)  -- an estimate of the remaining cost from n to the goal
///     f(n)  = g(n) + h(n), the estimated cost of the best path THROUGH n
///
/// The OPEN SET holds cells discovered but not yet examined, ordered by f.
/// The CLOSED SET holds cells already examined. The loop is:
///
///     pop the lowest-f cell from open
///     if it is the goal, done
///     move it to closed
///     for each neighbour, compute a tentative g; if that beats the
///     neighbour's current g, record it and push the neighbour into open
///
/// Setting h = 0 makes f = g, the open set becomes ordered by distance from
/// the start, and the algorithm is exactly Dijkstra. A* is therefore not a
/// different algorithm -- it is Dijkstra told which direction the goal is in.
///
/// The closed set is what makes it terminate, and with a CONSISTENT heuristic
/// (one satisfying h(n) <= cost(n, m) + h(m)) a closed cell never needs
/// revisiting. Octile and Euclidean are both consistent here.
class GridPlanner
{
public:
  explicit GridPlanner(const PlannerConfig & config = PlannerConfig());

  /// Load an occupancy grid and build the inflated costmap from it.
  ///
  /// \param occupancy  row-major, -1 unknown, 0..100 percent occupied
  void setMap(const std::vector<std::int8_t> & occupancy,
              int width, int height, double resolution);

  PlanResult plan(const GridIndex & start, const GridIndex & goal) const;

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}

  bool inBounds(const GridIndex & c) const
  {
    return c.x >= 0 && c.y >= 0 && c.x < width_ && c.y < height_;
  }

  /// True when the cell is impassable after inflation.
  bool isLethal(const GridIndex & c) const;

  /// Extra cost of traversing a cell, 0 in open space.
  double softCost(const GridIndex & c) const;

  /// The inflated costmap, for visualisation and tests.
  const std::vector<float> & costmap() const {return cost_; }
  const std::vector<std::uint8_t> & lethal() const {return lethal_; }

  /// Straight-line-equivalent distance under the configured heuristic.
  static double heuristic(Heuristic type, const GridIndex & a, const GridIndex & b);

  /// Geometric length of a cell path, diagonals counted as sqrt(2).
  static double pathLength(const std::vector<GridIndex> & path);

  const PlannerConfig & config() const {return config_;}
  PlannerConfig & mutableConfig() {return config_;}

private:
  std::size_t index(const GridIndex & c) const
  {
    return static_cast<std::size_t>(c.y) * width_ + c.x;
  }

  PlannerConfig config_;
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  std::vector<std::uint8_t> lethal_;
  std::vector<float> cost_;
};

}  // namespace arb_planning

#endif  // ARB_PLANNING__GRID_PLANNER_HPP_
