// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_CORE__POSE2D_HPP_
#define ARB_CORE__POSE2D_HPP_

#include <Eigen/Dense>
#include <cmath>

#include "arb_core/angles.hpp"

namespace arb_core
{

/// A rigid-body pose on the plane: an element of the Lie group SE(2).
///
/// A planar pose is *not* a 3-vector in a vector space, even though we store it
/// as (x, y, theta). Composition is matrix multiplication, not addition:
///
///     T = | cos(th)  -sin(th)   x |
///         | sin(th)   cos(th)   y |
///         |   0          0      1 |
///
/// Keeping that distinction explicit in a type is what stops the classic bug of
/// averaging or subtracting poses component-wise and silently breaking the
/// heading. Every operation below wraps theta through normalizeAngle().
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};   ///< heading in radians, always kept in [-pi, pi)

  Pose2D() = default;
  Pose2D(double x_in, double y_in, double theta_in)
  : x(x_in), y(y_in), theta(normalizeAngle(theta_in)) {}

  /// 3x3 homogeneous transform of this pose.
  Eigen::Matrix3d matrix() const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    Eigen::Matrix3d T;
    T << c, -s, x,
         s,  c, y,
         0,  0, 1;
    return T;
  }

  /// Group composition: this pose followed by `rhs` expressed in this frame.
  /// Equivalent to matrix() * rhs.matrix().
  Pose2D compose(const Pose2D & rhs) const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return Pose2D(
      x + c * rhs.x - s * rhs.y,
      y + s * rhs.x + c * rhs.y,
      theta + rhs.theta);
  }

  /// Group inverse: the transform that undoes this one.
  Pose2D inverse() const
  {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    return Pose2D(-(c * x + s * y), -(-s * x + c * y), -theta);
  }

  /// Relative pose of `other` as seen from this pose: inverse().compose(other).
  /// This is the correct way to express "where is the goal, in robot frame?".
  Pose2D relativeTo(const Pose2D & other) const
  {
    return inverse().compose(other);
  }

  /// Euclidean distance between the translation parts.
  double distanceTo(const Pose2D & other) const
  {
    return std::hypot(other.x - x, other.y - y);
  }

  /// Shortest signed heading difference to `other`, in [-pi, pi).
  double headingErrorTo(const Pose2D & other) const
  {
    return angleDiff(other.theta, theta);
  }

  Eigen::Vector3d vector() const { return Eigen::Vector3d(x, y, theta); }
};

}  // namespace arb_core

#endif  // ARB_CORE__POSE2D_HPP_
