// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_CORE__ANGLES_HPP_
#define ARB_CORE__ANGLES_HPP_

#include <cmath>
#include <limits>

namespace arb_core
{

/// Pi as a compile-time constant of the requested floating point type.
/// Using M_PI directly is non-portable (it is a POSIX extension, not ISO C++).
template<typename T = double>
inline constexpr T pi() noexcept
{
  return static_cast<T>(3.14159265358979323846264338327950288L);
}

template<typename T = double>
inline constexpr T two_pi() noexcept
{
  return static_cast<T>(2) * pi<T>();
}

/// Wrap an angle into the half-open interval [-pi, pi).
///
/// WHY THIS EXISTS
/// ---------------
/// Heading (yaw) lives on the circle S^1, not on the real line R. Two numbers
/// that differ by 2*pi describe the *same* physical orientation. Every place we
/// subtract two angles -- odometry increments, EKF innovation, controller
/// heading error -- we must project the result back onto S^1, otherwise:
///   * a robot at +179 deg and a target at -179 deg produce an error of
///     358 deg instead of the true -2 deg, and the controller spins the long
///     way round;
///   * the EKF innovation explodes, the Kalman gain reacts to a fictitious
///     huge error and the filter diverges.
///
/// IMPLEMENTATION NOTE
/// -------------------
/// std::remainder(x, 2*pi) already returns a value in [-pi, pi] using the
/// IEEE-754 "round to nearest even" rule, and -- unlike the classic
/// `while (a > pi) a -= 2*pi;` loop -- it is O(1), exact for large inputs and
/// does not hang on NaN/Inf. The extra branch below collapses the +pi endpoint
/// onto -pi so the interval is consistently half-open, which makes equality
/// comparisons in tests deterministic.
template<typename T>
inline T normalizeAngle(T angle) noexcept
{
  static_assert(std::is_floating_point<T>::value, "normalizeAngle requires a floating point type");
  if (!std::isfinite(angle)) {
    return std::numeric_limits<T>::quiet_NaN();
  }
  T wrapped = std::remainder(angle, two_pi<T>());
  if (wrapped >= pi<T>()) {
    wrapped -= two_pi<T>();
  }
  return wrapped;
}

/// Smallest signed rotation that takes `from` to `to`, in [-pi, pi).
/// Positive result means counter-clockwise.
///
/// Always use this instead of a raw `to - from` when either side is an angle.
template<typename T>
inline T angleDiff(T to, T from) noexcept
{
  return normalizeAngle<T>(to - from);
}

/// Wrap an angle into [0, 2*pi). Useful for LiDAR beam indexing, where the scan
/// is naturally parameterised from angle_min upwards rather than around zero.
template<typename T>
inline T normalizeAnglePositive(T angle) noexcept
{
  if (!std::isfinite(angle)) {
    return std::numeric_limits<T>::quiet_NaN();
  }
  T wrapped = std::fmod(angle, two_pi<T>());
  if (wrapped < static_cast<T>(0)) {
    wrapped += two_pi<T>();
  }
  // fmod on a tiny negative input can round up to exactly 2*pi; clamp it.
  if (wrapped >= two_pi<T>()) {
    wrapped = static_cast<T>(0);
  }
  return wrapped;
}

/// Circular mean of two angles, weighted by `alpha` in [0, 1].
/// alpha = 0 returns `a`, alpha = 1 returns `b`.
///
/// Naive linear interpolation between 179 deg and -179 deg gives 0 deg, which is
/// the exact opposite of the correct answer (180 deg). Interpolating along the
/// shortest arc is the only correct behaviour on S^1.
template<typename T>
inline T slerpAngle(T a, T b, T alpha) noexcept
{
  return normalizeAngle<T>(a + alpha * angleDiff<T>(b, a));
}

template<typename T>
inline constexpr T degToRad(T deg) noexcept
{
  return deg * pi<T>() / static_cast<T>(180);
}

template<typename T>
inline constexpr T radToDeg(T rad) noexcept
{
  return rad * static_cast<T>(180) / pi<T>();
}

}  // namespace arb_core

#endif  // ARB_CORE__ANGLES_HPP_
