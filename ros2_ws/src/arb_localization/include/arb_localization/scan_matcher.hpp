// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_LOCALIZATION__SCAN_MATCHER_HPP_
#define ARB_LOCALIZATION__SCAN_MATCHER_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <vector>

#include "arb_core/pose2d.hpp"

namespace arb_localization
{

/// A likelihood field built from an occupancy grid.
///
/// ========================= WHY A LIKELIHOOD FIELD =========================
///
/// Scoring a candidate pose by asking "is the cell under each beam endpoint
/// occupied?" gives a score surface that is flat almost everywhere and jumps
/// at cell boundaries. There is no gradient to follow and no way to tell a
/// pose that is 2 cm out from one that is 2 m out: both score zero.
///
/// The likelihood field replaces the binary question with a smooth one: how
/// FAR is the nearest obstacle from this endpoint? Scoring
///
///     exp( -d^2 / (2 sigma^2) )
///
/// makes the surface continuous, peaked at the correct pose, and monotone in
/// the neighbourhood of it -- which is what makes any search work at all.
///
/// sigma is the width of that basin. Too small and the field is a set of
/// needles the search can step over; too large and every pose scores about the
/// same and the match is mush. A few cells is the usual answer.
class LikelihoodField
{
public:
  /// \param occupancy  row-major occupancy bytes: -1 unknown, 0..100 percent
  /// \param width,height  grid size in cells
  /// \param resolution  metres per cell
  /// \param origin_x,origin_y  world coordinates of cell (0,0)'s corner
  /// \param occupied_threshold  percent at or above which a cell is an obstacle
  /// \param sigma_m  width of the Gaussian basin around each obstacle
  /// \param max_distance_m  radius the field is splatted out to
  LikelihoodField(
    const std::vector<std::int8_t> & occupancy,
    int width, int height, double resolution,
    double origin_x, double origin_y,
    int occupied_threshold = 65,
    double sigma_m = 0.15,
    double max_distance_m = 0.75);

  /// Likelihood in [0, 1] at a world point; 0 outside the field.
  double at(double x, double y) const;

  bool empty() const {return obstacle_count_ == 0;}
  int obstacleCount() const {return obstacle_count_;}
  int width() const {return width_;}
  int height() const {return height_;}

private:
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  int obstacle_count_{0};
  std::vector<float> field_;
};

/// One beam endpoint in the SENSOR frame.
struct ScanPoint
{
  double x{0.0};
  double y{0.0};
};

struct MatchResult
{
  bool valid{false};
  arb_core::Pose2D pose;        ///< corrected sensor pose in the map frame
  double score{0.0};            ///< mean likelihood per point, in [0, 1]
  double initial_score{0.0};    ///< the same for the pose it started from
  int points_used{0};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Identity()};
};

struct MatcherConfig
{
  /// Coarse-to-fine search half-widths and steps. Two passes rather than one
  /// fine pass: a single 1 cm search over +-0.4 m and +-10 degrees is 80 x 80 x
  /// 40 = 256 000 candidates, while coarse-then-fine costs about 3 000 for the
  /// same reach and resolution.
  double coarse_linear_window_m{0.30};
  double coarse_linear_step_m{0.05};
  double coarse_angular_window_rad{0.12};   // ~7 degrees
  double coarse_angular_step_rad{0.02};     // ~1.1 degrees

  double fine_linear_window_m{0.06};
  double fine_linear_step_m{0.01};
  double fine_angular_window_rad{0.025};
  double fine_angular_step_rad{0.004};

  /// Beams are subsampled: neighbouring beams on the same wall are nearly
  /// redundant, and the cost is linear in the number of points.
  int point_stride{2};
  int min_points{60};

  /// Reject a match whose mean likelihood is below this. A low score means the
  /// scan did not settle onto the map, and feeding that pose to the filter is
  /// worse than feeding nothing.
  double min_score{0.35};

  /// Reject a match that did not improve on where it started. A match that
  /// makes the fit worse has found a different minimum, not a better one.
  double min_improvement{-0.02};

  /// Measurement covariance handed to the filter, scaled down as the score
  /// rises. Even a perfect match is given a floor: scan matching against a map
  /// the robot built itself cannot be trusted like an external reference.
  double position_sigma_m{0.05};
  double heading_sigma_rad{0.02};
};

/// Correlative scan matching against a likelihood field.
///
/// An exhaustive coarse-to-fine search rather than Gauss-Newton or ICP.
/// The reason is robustness: gradient methods converge to whatever basin they
/// start in, and in a corridor the basin along the corridor axis is almost
/// flat, so they slide. An exhaustive search over a bounded window either
/// finds the global optimum inside that window or reports a low score, and it
/// has no failure mode where it confidently returns the wrong answer.
class ScanMatcher
{
public:
  explicit ScanMatcher(const MatcherConfig & config = MatcherConfig());

  MatchResult match(const LikelihoodField & field,
                    const arb_core::Pose2D & initial,
                    const std::vector<ScanPoint> & points) const;

  /// Mean likelihood of the points placed by `pose`.
  static double score(const LikelihoodField & field,
                      const arb_core::Pose2D & pose,
                      const std::vector<ScanPoint> & points);

  const MatcherConfig & config() const {return config_;}

private:
  arb_core::Pose2D searchAround(
    const LikelihoodField & field, const arb_core::Pose2D & centre,
    const std::vector<ScanPoint> & points,
    double linear_window, double linear_step,
    double angular_window, double angular_step,
    double & best_score) const;

  MatcherConfig config_;
};

}  // namespace arb_localization

#endif  // ARB_LOCALIZATION__SCAN_MATCHER_HPP_
