// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_MAPPING__OCCUPANCY_GRID_HPP_
#define ARB_MAPPING__OCCUPANCY_GRID_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "arb_core/pose2d.hpp"

namespace arb_mapping
{

/// Configuration for the occupancy grid and its sensor model.
struct GridConfig
{
  double resolution_m{0.05};   ///< side of one cell
  double width_m{24.0};        ///< extent in x
  double height_m{18.0};       ///< extent in y
  double origin_x_m{-12.0};    ///< world coordinate of cell (0, 0)'s corner
  double origin_y_m{-9.0};

  // --- inverse sensor model, in PROBABILITY -----------------------------
  //
  // These are read as: "given that a beam passed through this cell, how likely
  // is the cell to be occupied?" Neither is 0 or 1, and that is the point:
  // a single measurement can never prove a cell empty or full, and encoding
  // certainty makes the map unable to ever change its mind.
  double p_hit{0.75};          ///< cell at the beam's endpoint
  double p_miss{0.35};         ///< cells the beam passed through
  double p_prior{0.5};         ///< before any measurement: no information

  // --- clamping ---------------------------------------------------------
  //
  // Log-odds are clamped so a cell can always be revised. Without clamping, a
  // wall seen 500 times reaches a log-odds of +250, and a door opened in it
  // would need 500 contradicting observations to register. Clamping bounds the
  // evidence any cell can accumulate and therefore bounds how long it takes to
  // forget -- which is what makes the map able to track a changing world.
  double log_odds_min{-2.0};
  double log_odds_max{3.5};

  /// Cells at or beyond this |log-odds| are reported as occupied / free;
  /// anything between stays unknown.
  double occupied_threshold{0.85};   ///< ~0.7 probability
  double free_threshold{-0.4};       ///< ~0.4 probability

  /// Beams longer than this are treated as "no return": the endpoint is not
  /// marked occupied, but the ray still clears the cells it passed through.
  /// A max-range beam carries real information -- there was nothing in the way
  /// -- and discarding it entirely leaves open space permanently unknown.
  double max_usable_range_m{10.0};

  /// Beams shorter than this are discarded. Returns at the very edge of the
  /// sensor's minimum range are usually the robot's own structure.
  double min_usable_range_m{0.15};
};

/// Three-valued view of a cell, which is what "occupancy grid" actually means.
///
/// UNKNOWN is not a shade of free. A planner that treats unknown as free will
/// drive confidently through a wall it has never seen; one that treats it as
/// occupied will refuse to explore. They are genuinely three different states
/// and collapsing them is the most common mapping bug there is.
enum class CellState : std::uint8_t
{
  kUnknown = 0,
  kFree = 1,
  kOccupied = 2,
};

/// A 2D occupancy grid maintained in LOG-ODDS.
///
/// ======================== WHY LOG-ODDS ========================
///
/// The quantity we want per cell is P(occupied | all measurements so far).
/// Applying Bayes' rule directly for each new measurement gives
///
///     P(m | z_1:t) =  P(m|z_t) P(m|z_1:t-1) / ...
///
/// with a normalising denominator that has to be recomputed every time. Two
/// problems follow: the arithmetic is a product of many numbers near 0 or 1,
/// which underflows, and every update needs a division.
///
/// Define the log-odds
///
///     l(m) = log( P(m) / (1 - P(m)) )
///
/// Under the standard independence assumption, Bayes' rule becomes ADDITION:
///
///     l_t(m) = l_{t-1}(m) + l_sensor(m | z_t) - l_0
///
/// where l_0 is the prior in log-odds (zero for an uninformative prior of 0.5).
/// That is the whole reason log-odds are used: the update is one add and one
/// clamp, it cannot underflow, and it is symmetric -- evidence for and against
/// cancel exactly, which is what lets a map correct itself.
///
/// Recovering the probability is p = 1 - 1/(1 + exp(l)).
///
/// =================== THE INVERSE SENSOR MODEL ===================
///
/// "Inverse" because it answers P(map | measurement), not P(measurement | map).
/// For a range finder it is deliberately crude:
///
///   * every cell the beam passed THROUGH gets evidence for being free
///     (p_miss), because the beam reached further
///   * the cell at the beam's ENDPOINT gets evidence for being occupied
///     (p_hit), because something stopped the beam there
///   * every other cell is untouched
///
/// The crudeness is intentional. A physically detailed model of a laser return
/// buys very little once hundreds of beams per second are being integrated,
/// and it costs the one property that makes the whole scheme work: that the
/// update is a constant added per cell.
class OccupancyGrid
{
public:
  explicit OccupancyGrid(const GridConfig & config);

  /// Integrate one beam, given the sensor pose in the grid's frame.
  ///
  /// \param sensor  pose of the range finder, not of base_link
  /// \param bearing world-frame bearing of the beam
  /// \param range   measured range; non-finite or beyond max_usable_range is
  ///                treated as "no return"
  /// \return false if the beam was discarded entirely
  bool integrateBeam(const arb_core::Pose2D & sensor, double bearing, double range);

  /// Integrate a whole scan. Returns the number of beams used.
  int integrateScan(const arb_core::Pose2D & sensor,
                    const std::vector<float> & ranges,
                    double angle_min, double angle_increment,
                    double range_min, double range_max);

  // --- indexing ---------------------------------------------------------
  int width() const {return width_;}
  int height() const {return height_;}
  const GridConfig & config() const {return config_;}

  bool inBounds(int cx, int cy) const
  {
    return cx >= 0 && cy >= 0 && cx < width_ && cy < height_;
  }

  /// World point to cell indices. Returns false if outside the grid.
  bool worldToCell(double x, double y, int & cx, int & cy) const;

  /// Centre of a cell in world coordinates.
  void cellToWorld(int cx, int cy, double & x, double & y) const;

  double logOdds(int cx, int cy) const;
  double probability(int cx, int cy) const;
  CellState state(int cx, int cy) const;

  /// Raw log-odds buffer, row-major with y as the slow axis, matching
  /// nav_msgs/OccupancyGrid's layout.
  const std::vector<float> & data() const {return log_odds_;}

  /// Occupancy as nav_msgs/OccupancyGrid expects it: -1 unknown, 0..100
  /// probability in percent.
  std::vector<std::int8_t> toOccupancyBytes() const;

  void clear();

  /// Number of cells currently in each state, for diagnostics and tests.
  struct Counts { int unknown{0}; int free{0}; int occupied{0}; };
  Counts counts() const;

  /// Bresenham's line algorithm between two cells, endpoint EXCLUDED.
  ///
  /// Excluding the endpoint is not a detail: the endpoint is the cell the beam
  /// hit, and marking it free before marking it occupied would have the two
  /// updates fight each other. Every cell this returns is a cell the beam
  /// passed THROUGH.
  ///
  /// Bresenham rather than stepping along the ray in small increments because
  /// it is exact integer arithmetic: it visits each cell exactly once, never
  /// skips one, and never revisits one. Sub-stepping with a float either
  /// double-counts cells (biasing the evidence) or leaves diagonal gaps that
  /// let a planner path straight through a wall.
  static std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1);

  /// probability -> log-odds and back.
  static double probabilityToLogOdds(double p);
  static double logOddsToProbability(double l);

private:
  void updateCell(int cx, int cy, double delta);

  GridConfig config_;
  int width_{0};
  int height_{0};
  std::vector<float> log_odds_;

  double l_hit_{0.0};
  double l_miss_{0.0};
  double l_prior_{0.0};
};

}  // namespace arb_mapping

#endif  // ARB_MAPPING__OCCUPANCY_GRID_HPP_
