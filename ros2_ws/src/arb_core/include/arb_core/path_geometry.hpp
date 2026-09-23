// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_CORE__PATH_GEOMETRY_HPP_
#define ARB_CORE__PATH_GEOMETRY_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "arb_core/pose2d.hpp"

namespace arb_core
{

/// ================== WHERE AM I ON THIS PATH, AND WHERE DO I AIM ==============
///
/// Every path-following controller needs the same two answers, and both of them
/// are measured from the robot's PROJECTION onto the path -- the nearest point
/// on it -- never from the path's first vertex:
///
///   1. the cross-track error: how far off the path the robot is, and which side
///   2. the carrot: the point a fixed arc length FURTHER ALONG than that
///      projection, which is what the controller actually steers at
///
/// Measuring the carrot from vertex 0 instead is the single most expensive bug
/// in this project's history, because it is invisible for the first second of
/// every run and then silently fatal. It shipped twice, in two independent
/// controllers:
///
///   * the PID path follower spun on the spot at its maximum turn rate for
///     150 s, having driven to a carrot 0.35 m from the path's start, passed it,
///     and then aimed at a point behind itself forever
///   * the DWA local planner covered 1.99 m of a 15.60 m course in 140 s, its
///     best-scoring command every cycle being the slowest one available
///
/// Both looked correct in review and both had a comment saying "walk the path
/// from the robot". They work as long as something republishes the path from the
/// robot's current position several times a second, which the global planner
/// does -- so the defect hides behind the replanner and appears the moment a
/// path is held still.
///
/// That is why this lives in arb_core rather than in either controller: one
/// implementation, one set of tests, and a dependency direction that does not
/// make the planner depend on the controller.
struct PathProjection
{
  std::size_t segment{0};        ///< index i of the segment path[i] -> path[i+1]
  double t{0.0};                 ///< position along that segment, in [0, 1]
  double arc{0.0};               ///< arc length from path[0] to the nearest point
  double signed_distance{0.0};   ///< positive when the path is to the robot's left
  Pose2D point;                  ///< the nearest point, in the same frame as `path`
  bool valid{false};
};

/// Total arc length of a polyline.
inline double pathLength(const std::vector<Pose2D> & path)
{
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    total += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  }
  return total;
}

/// Mean spacing between consecutive vertices, for converting a distance in
/// metres into a number of segments.
inline double pathSpacing(const std::vector<Pose2D> & path)
{
  if (path.size() < 2) {return 1.0;}
  return pathLength(path) / static_cast<double>(path.size() - 1);
}

/// Project the ORIGIN of the path's frame onto the path.
///
/// The path is expected in the ROBOT frame, so the robot is at the origin facing
/// +x and the sign convention follows from that directly: the nearest point's y
/// coordinate is positive when the path is to the robot's left.
///
/// \param from  lowest segment index to consider. Progress along a path is
///   monotonic, so a path that comes back near itself -- a lap, a corridor
///   walked twice -- cannot make the controller snap to a leg it finished ten
///   metres ago and report a cross-track error with the wrong sign. Pass 0 for
///   an unrestricted search.
inline PathProjection projectOntoPath(const std::vector<Pose2D> & path,
                                     std::size_t from = 0)
{
  PathProjection best;
  if (path.empty()) {return best;}
  if (path.size() < 2) {
    best.point = path.front();
    best.signed_distance = path.front().y;
    best.valid = true;
    return best;
  }

  // Arc length is accumulated while the segments are walked, so the carrot --
  // which is a DISTANCE along the path -- has something to measure from.
  // Recomputing it afterwards would mean walking the path twice.
  double arc_to_segment_start = 0.0;
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double ax = path[i].x;
    const double ay = path[i].y;
    const double dx = path[i + 1].x - ax;
    const double dy = path[i + 1].y - ay;
    const double len2 = dx * dx + dy * dy;
    const double len = std::sqrt(len2);
    const double segment_start_arc = arc_to_segment_start;
    arc_to_segment_start += len;
    if (len2 < 1e-12 || i < from) {continue;}

    const double t = std::clamp(-(ax * dx + ay * dy) / len2, 0.0, 1.0);
    const double px = ax + t * dx;
    const double py = ay + t * dy;
    const double d = std::hypot(px, py);
    if (d < best_distance) {
      best_distance = d;
      best.valid = true;
      best.segment = i;
      best.t = t;
      best.arc = segment_start_arc + t * len;
      best.point = Pose2D(px, py, 0.0);
      // The sign says which way to STEER, so the convention has to be "the path
      // is to my left", not "I am to the left of the path" -- they are
      // opposites, and confusing them produces a controller that drives away
      // from its path smoothly and confidently.
      best.signed_distance = (py >= 0.0) ? d : -d;
    }
  }
  if (!best.valid && from > 0) {
    // Every segment was behind the progress window: the robot has run off the
    // end. Fall back to an unrestricted search rather than reporting a zero
    // error it has not earned.
    return projectOntoPath(path, 0);
  }
  return best;
}

/// The point `lookahead` metres further along the path than `arc`.
///
/// Interpolated within the segment it falls in, not snapped to the next vertex:
/// on a path densified to 5 cm, snapping quantises the carrot's bearing and the
/// steering command inherits the steps.
inline Pose2D carrotAt(const std::vector<Pose2D> & path, double arc,
                       double lookahead)
{
  if (path.empty()) {return Pose2D();}
  const double target = arc + lookahead;
  double walked = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double dx = path[i + 1].x - path[i].x;
    const double dy = path[i + 1].y - path[i].y;
    const double len = std::hypot(dx, dy);
    if (walked + len >= target) {
      const double t = (len > 1e-12) ? (target - walked) / len : 0.0;
      return Pose2D(path[i].x + t * dx, path[i].y + t * dy, 0.0);
    }
    walked += len;
  }
  return path.back();
}

}  // namespace arb_core

#endif  // ARB_CORE__PATH_GEOMETRY_HPP_
