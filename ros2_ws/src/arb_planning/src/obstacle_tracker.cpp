// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_planning/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arb_planning
{

std::vector<std::vector<ScanPoint>> ObstacleTracker::cluster(
  const std::vector<ScanPoint> & points, double gap_m, std::size_t min_points)
{
  std::vector<std::vector<ScanPoint>> out;
  if (points.empty()) {return out;}

  std::vector<ScanPoint> current{points.front()};
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double d = std::hypot(points[i].x - points[i - 1].x,
                                points[i].y - points[i - 1].y);
    if (d > gap_m) {
      if (current.size() >= min_points) {out.push_back(current);}
      current.clear();
    }
    current.push_back(points[i]);
  }
  if (current.size() >= min_points) {out.push_back(current);}
  return out;
}

void ObstacleTracker::describe(const std::vector<ScanPoint> & c,
                               double * cx, double * cy, double * extent)
{
  *cx = *cy = *extent = 0.0;
  if (c.empty()) {return;}
  for (const auto & p : c) {*cx += p.x; *cy += p.y;}
  *cx /= static_cast<double>(c.size());
  *cy /= static_cast<double>(c.size());
  // Extent as the span of the endpoints rather than a full diameter search:
  // the points arrive in scan order, so the first and last are the two
  // angular extremes, and an O(n^2) diameter would buy nothing at this scale.
  *extent = std::hypot(c.back().x - c.front().x, c.back().y - c.front().y);
}

void ObstacleTracker::update(const std::vector<ScanPoint> & points,
                             const arb_core::Pose2D & robot, double stamp_s)
{
  rejected_as_structure_ = 0;

  const double dt = have_previous_ ? (stamp_s - previous_stamp_s_) : 0.0;
  previous_stamp_s_ = stamp_s;
  // A non-advancing or backwards clock cannot produce a velocity. Returning
  // early rather than dividing by it keeps a replayed bag or a paused
  // simulator from manufacturing infinities.
  const bool can_differentiate = have_previous_ && dt > 1e-4;
  have_previous_ = true;

  const double c = std::cos(robot.theta);
  const double s = std::sin(robot.theta);

  struct Candidate { double x, y, extent; };
  std::vector<Candidate> candidates;
  for (const auto & cl : cluster(points, config_.cluster_gap_m,
                                 config_.min_cluster_points))
  {
    double cx = 0.0, cy = 0.0, extent = 0.0;
    describe(cl, &cx, &cy, &extent);
    if (extent > config_.max_cluster_extent_m) {
      // Structure, not an object. A wall's centroid slides along it as the
      // robot moves and would manufacture a velocity out of pure geometry.
      ++rejected_as_structure_;
      continue;
    }
    // Robot frame -> fixed frame. Differencing in the robot frame would
    // report every static object as moving at the robot's own speed.
    candidates.push_back({robot.x + c * cx - s * cy,
                          robot.y + s * cx + c * cy, extent});
  }

  std::vector<bool> claimed(tracks_.size(), false);
  std::vector<ObstacleTrack> updated;
  updated.reserve(candidates.size() + tracks_.size());

  for (const auto & cand : candidates) {
    // Nearest unclaimed track inside the gate.
    std::size_t best = tracks_.size();
    double best_d = config_.association_gate_m;
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (claimed[i]) {continue;}
      const double d = std::hypot(tracks_[i].x - cand.x, tracks_[i].y - cand.y);
      if (d < best_d) {best_d = d; best = i;}
    }

    if (best == tracks_.size()) {
      ObstacleTrack t;
      t.id = next_id_++;
      t.x = cand.x;
      t.y = cand.y;
      t.extent_m = cand.extent;
      t.hits = 1;
      t.last_seen_s = stamp_s;
      updated.push_back(t);
      continue;
    }

    claimed[best] = true;
    ObstacleTrack t = tracks_[best];
    if (can_differentiate) {
      const double raw_vx = (cand.x - t.x) / dt;
      const double raw_vy = (cand.y - t.y) / dt;
      const double speed = std::hypot(raw_vx, raw_vy);
      if (speed <= config_.max_tracked_speed_m_s) {
        // First-order low pass. alpha = dt / (tau + dt) is the exact
        // discretisation and stays stable for any dt.
        const double alpha = (config_.velocity_filter_tau_s > 0.0)
          ? dt / (config_.velocity_filter_tau_s + dt) : 1.0;
        t.vx += alpha * (raw_vx - t.vx);
        t.vy += alpha * (raw_vy - t.vy);
      }
      // Above max_tracked_speed_m_s the association is wrong, not the world:
      // two different objects were matched. Keep the previous velocity rather
      // than believing a jump.
    }
    t.x = cand.x;
    t.y = cand.y;
    t.extent_m = cand.extent;
    t.hits += 1;
    t.last_seen_s = stamp_s;
    t.trusted = t.hits >= config_.min_hits_before_trusted;
    updated.push_back(t);
  }

  // Carry unmatched tracks forward briefly: an obstacle occluded for one scan
  // must not become a new object with no velocity the moment it reappears.
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    if (claimed[i]) {continue;}
    if (stamp_s - tracks_[i].last_seen_s <= config_.max_track_age_s) {
      ObstacleTrack t = tracks_[i];
      // Coast on the last known velocity so the prediction stays usable while
      // the object is hidden.
      if (can_differentiate) {t.x += t.vx * dt; t.y += t.vy * dt;}
      updated.push_back(t);
    }
  }

  tracks_ = std::move(updated);
}

std::vector<ObstacleTrack> ObstacleTracker::movingInRobotFrame(
  const arb_core::Pose2D & robot) const
{
  std::vector<ObstacleTrack> out;
  const double c = std::cos(-robot.theta);
  const double s = std::sin(-robot.theta);
  for (const auto & t : tracks_) {
    if (!t.trusted) {continue;}
    if (std::hypot(t.vx, t.vy) < config_.min_tracked_speed_m_s) {continue;}
    ObstacleTrack r = t;
    const double dx = t.x - robot.x;
    const double dy = t.y - robot.y;
    r.x = c * dx - s * dy;
    r.y = s * dx + c * dy;
    r.vx = c * t.vx - s * t.vy;
    r.vy = s * t.vx + c * t.vy;
    out.push_back(r);
  }
  return out;
}

}  // namespace arb_planning
