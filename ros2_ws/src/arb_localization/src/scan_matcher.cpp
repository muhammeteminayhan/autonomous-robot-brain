// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_localization/scan_matcher.hpp"

#include <algorithm>
#include <cmath>

#include "arb_core/angles.hpp"

namespace arb_localization
{

LikelihoodField::LikelihoodField(
  const std::vector<std::int8_t> & occupancy,
  int width, int height, double resolution,
  double origin_x, double origin_y,
  int occupied_threshold, double sigma_m, double max_distance_m)
: width_(width), height_(height), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y)
{
  field_.assign(static_cast<std::size_t>(width_) * height_, 0.0f);
  if (width_ <= 0 || height_ <= 0 || resolution_ <= 0.0) {return;}

  const int radius = std::max(1, static_cast<int>(std::ceil(max_distance_m / resolution_)));
  const double two_sigma_sq = 2.0 * sigma_m * sigma_m;

  // Splat a Gaussian around every obstacle, keeping the MAXIMUM rather than
  // summing. Summing would make a thick wall score higher than a thin one, so
  // the matcher would prefer poses that pile points onto clutter instead of
  // poses that fit the geometry.
  for (int cy = 0; cy < height_; ++cy) {
    for (int cx = 0; cx < width_; ++cx) {
      const std::int8_t v = occupancy[static_cast<std::size_t>(cy) * width_ + cx];
      if (v < occupied_threshold) {continue;}
      ++obstacle_count_;
      for (int dy = -radius; dy <= radius; ++dy) {
        const int ny = cy + dy;
        if (ny < 0 || ny >= height_) {continue;}
        for (int dx = -radius; dx <= radius; ++dx) {
          const int nx = cx + dx;
          if (nx < 0 || nx >= width_) {continue;}
          const double d2 = (static_cast<double>(dx) * dx + static_cast<double>(dy) * dy)
            * resolution_ * resolution_;
          const float value = static_cast<float>(std::exp(-d2 / two_sigma_sq));
          float & slot = field_[static_cast<std::size_t>(ny) * width_ + nx];
          slot = std::max(slot, value);
        }
      }
    }
  }
}

double LikelihoodField::at(double x, double y) const
{
  if (field_.empty()) {return 0.0;}

  // BILINEAR interpolation, not nearest-cell lookup.
  //
  // Sampling the nearest cell makes the score surface a staircase whose steps
  // are one cell wide, so the best-scoring pose is only ever accurate to about
  // half a cell -- 2.5 cm here -- and the matcher reports a correction even
  // when the pose is already exactly right. Measured with nearest-cell
  // lookup: a perfectly aligned scan was "corrected" by 4.2 cm.
  //
  // Interpolating makes the surface continuous, so the optimum sits where the
  // geometry actually is rather than on the grid, and the search can resolve
  // well below one cell. It costs three extra multiplies per point.
  const double fx = (x - origin_x_) / resolution_ - 0.5;
  const double fy = (y - origin_y_) / resolution_ - 0.5;
  const int x0 = static_cast<int>(std::floor(fx));
  const int y0 = static_cast<int>(std::floor(fy));
  const double tx = fx - x0;
  const double ty = fy - y0;

  auto sample = [this](int cx, int cy) -> double {
      if (cx < 0 || cy < 0 || cx >= width_ || cy >= height_) {return 0.0;}
      return field_[static_cast<std::size_t>(cy) * width_ + cx];
    };

  const double v00 = sample(x0, y0);
  const double v10 = sample(x0 + 1, y0);
  const double v01 = sample(x0, y0 + 1);
  const double v11 = sample(x0 + 1, y0 + 1);
  return (1.0 - tx) * (1.0 - ty) * v00 + tx * (1.0 - ty) * v10 +
         (1.0 - tx) * ty * v01 + tx * ty * v11;
}

ScanMatcher::ScanMatcher(const MatcherConfig & config)
: config_(config) {}

double ScanMatcher::score(
  const LikelihoodField & field, const arb_core::Pose2D & pose,
  const std::vector<ScanPoint> & points)
{
  if (points.empty()) {return 0.0;}
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  double total = 0.0;
  for (const auto & p : points) {
    total += field.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y);
  }
  return total / static_cast<double>(points.size());
}

arb_core::Pose2D ScanMatcher::searchAround(
  const LikelihoodField & field, const arb_core::Pose2D & centre,
  const std::vector<ScanPoint> & points,
  double linear_window, double linear_step,
  double angular_window, double angular_step,
  double & best_score) const
{
  arb_core::Pose2D best = centre;
  best_score = score(field, centre, points);

  const int n_lin = std::max(0, static_cast<int>(std::round(linear_window / linear_step)));
  const int n_ang = std::max(0, static_cast<int>(std::round(angular_window / angular_step)));

  for (int ia = -n_ang; ia <= n_ang; ++ia) {
    const double theta = arb_core::normalizeAngle(centre.theta + ia * angular_step);
    for (int iy = -n_lin; iy <= n_lin; ++iy) {
      const double y = centre.y + iy * linear_step;
      for (int ix = -n_lin; ix <= n_lin; ++ix) {
        const arb_core::Pose2D candidate(centre.x + ix * linear_step, y, theta);
        const double s = score(field, candidate, points);
        if (s > best_score) {
          best_score = s;
          best = candidate;
        }
      }
    }
  }
  return best;
}

MatchResult ScanMatcher::match(
  const LikelihoodField & field, const arb_core::Pose2D & initial,
  const std::vector<ScanPoint> & points) const
{
  MatchResult result;
  result.pose = initial;
  if (field.empty() || static_cast<int>(points.size()) < config_.min_points) {
    return result;
  }
  result.points_used = static_cast<int>(points.size());
  result.initial_score = score(field, initial, points);

  double coarse_score = 0.0;
  const arb_core::Pose2D coarse = searchAround(
    field, initial, points,
    config_.coarse_linear_window_m, config_.coarse_linear_step_m,
    config_.coarse_angular_window_rad, config_.coarse_angular_step_rad,
    coarse_score);

  double fine_score = 0.0;
  const arb_core::Pose2D fine = searchAround(
    field, coarse, points,
    config_.fine_linear_window_m, config_.fine_linear_step_m,
    config_.fine_angular_window_rad, config_.fine_angular_step_rad,
    fine_score);

  result.pose = fine;
  result.score = fine_score;

  if (fine_score < config_.min_score) {return result;}
  if (fine_score - result.initial_score < config_.min_improvement) {return result;}

  // Covariance shrinks as the score rises, with a floor. The floor matters:
  // the map was built from this same estimate, so a perfect match proves the
  // scan agrees with the robot's own beliefs, not with the world. Handing the
  // filter a tiny covariance would let it drive its uncertainty towards zero
  // on entirely self-referential evidence.
  const double quality = std::clamp(fine_score, 0.05, 1.0);
  const double pos = config_.position_sigma_m / quality;
  const double head = config_.heading_sigma_rad / quality;
  result.covariance = Eigen::Matrix3d::Zero();
  result.covariance(0, 0) = pos * pos;
  result.covariance(1, 1) = pos * pos;
  result.covariance(2, 2) = head * head;
  result.valid = true;
  return result;
}

}  // namespace arb_localization
