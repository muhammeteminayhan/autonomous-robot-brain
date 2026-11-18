// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_planning/obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arb_planning
{

std::vector<ObstacleTracker::ClusterSpan> ObstacleTracker::clusterSpans(
  const std::vector<ScanPoint> & points, double gap_m, std::size_t min_points)
{
  std::vector<ClusterSpan> out;
  if (points.empty()) {return out;}

  std::size_t begin = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double d = std::hypot(points[i].x - points[i - 1].x,
                                points[i].y - points[i - 1].y);
    if (d > gap_m) {
      if (i - begin >= min_points) {out.push_back({begin, i});}
      begin = i;
    }
  }
  if (points.size() - begin >= min_points) {
    out.push_back({begin, points.size()});
  }
  return out;
}

std::vector<std::vector<ScanPoint>> ObstacleTracker::cluster(
  const std::vector<ScanPoint> & points, double gap_m, std::size_t min_points)
{
  std::vector<std::vector<ScanPoint>> out;
  for (const auto & span : clusterSpans(points, gap_m, min_points)) {
    out.emplace_back(points.begin() + static_cast<std::ptrdiff_t>(span.begin),
                     points.begin() + static_cast<std::ptrdiff_t>(span.end));
  }
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

namespace
{
double rangeOf(const ScanPoint & p) {return std::hypot(p.x, p.y);}
}  // namespace

bool ObstacleTracker::isFlatSurface(
  const std::vector<ScanPoint> & cluster) const
{
  // The smaller principal component of the cluster IS its deviation from the
  // best-fit line -- that is what principal components are -- so there is no
  // need to fit the line, only to find the smaller eigenvalue of a 2x2
  // covariance. Closed form, no iteration, no library.
  if (cluster.size() < config_.min_points_for_shape) {return false;}

  double mx = 0.0, my = 0.0;
  for (const auto & p : cluster) {mx += p.x; my += p.y;}
  mx /= static_cast<double>(cluster.size());
  my /= static_cast<double>(cluster.size());

  double sxx = 0.0, syy = 0.0, sxy = 0.0;
  for (const auto & p : cluster) {
    const double dx = p.x - mx, dy = p.y - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  const double n = static_cast<double>(cluster.size());
  sxx /= n; syy /= n; sxy /= n;

  // Smaller eigenvalue of [[sxx, sxy], [sxy, syy]].
  const double half_trace = 0.5 * (sxx + syy);
  const double gap = std::sqrt(std::max(
    0.0, half_trace * half_trace - (sxx * syy - sxy * sxy)));
  const double smaller = half_trace - gap;
  const double deviation = std::sqrt(std::max(0.0, smaller));

  // Per unit length, so the verdict does not depend on how much of the
  // object happens to be visible.
  const double length = std::hypot(cluster.back().x - cluster.front().x,
                                   cluster.back().y - cluster.front().y);
  if (length < 1e-6) {return false;}
  return deviation / length < config_.min_bend_ratio;
}

bool ObstacleTracker::seenThroughAGap(const std::vector<ScanPoint> & points,
                                      const ClusterSpan & span) const
{
  // What BOUNDS a cluster says more about it than its width does.
  //
  // An object standing in the open is bounded by returns from further away
  // (the background) or by no return at all. A view through a doorway is
  // bounded on BOTH sides by nearer returns -- the thing being looked past.
  // Only the second case makes the extent meaningless, so both sides must
  // be nearer before the cluster is dropped; one near side is an ordinary
  // partial occlusion and the object is still an object.
  //
  // The scan wraps, so the neighbour of the first point is the last. A
  // cluster that straddles the wrap is split in two and each half is judged
  // on its own -- a known limitation, and a conservative one: the halves
  // keep their outer neighbours and are less likely to be dropped, not more.
  const std::size_t n = points.size();
  if (n < 3 || span.end - span.begin >= n) {return false;}

  double near = std::numeric_limits<double>::max();
  for (std::size_t i = span.begin; i < span.end; ++i) {
    near = std::min(near, rangeOf(points[i]));
  }
  const double threshold = near - config_.aperture_margin_m;

  const std::size_t left = (span.begin == 0) ? n - 1 : span.begin - 1;
  const std::size_t right = (span.end == n) ? 0 : span.end;
  return rangeOf(points[left]) < threshold &&
         rangeOf(points[right]) < threshold;
}

void ObstacleTracker::update(const std::vector<ScanPoint> & points,
                             const arb_core::Pose2D & robot, double stamp_s)
{
  rejected_as_structure_ = 0;
  rejected_as_occluded_ = 0;
  rejected_as_flat_ = 0;

  const double dt = have_previous_ ? (stamp_s - previous_stamp_s_) : 0.0;
  previous_stamp_s_ = stamp_s;
  latest_stamp_s_ = stamp_s;
  // A non-advancing or backwards clock cannot produce a velocity. Returning
  // early rather than dividing by it keeps a replayed bag or a paused
  // simulator from manufacturing infinities.
  const bool can_differentiate = have_previous_ && dt > 1e-4;
  have_previous_ = true;

  const double c = std::cos(robot.theta);
  const double s = std::sin(robot.theta);

  struct Candidate { double x, y, extent; };
  std::vector<Candidate> candidates;
  const auto spans = clusterSpans(points, config_.cluster_gap_m,
                                  config_.min_cluster_points);
  for (const auto & span : spans) {
    const std::vector<ScanPoint> cl(
      points.begin() + static_cast<std::ptrdiff_t>(span.begin),
      points.begin() + static_cast<std::ptrdiff_t>(span.end));
    double cx = 0.0, cy = 0.0, extent = 0.0;
    describe(cl, &cx, &cy, &extent);
    if (extent > config_.max_cluster_extent_m) {
      // Structure, not an object. A wall's centroid slides along it as the
      // robot moves and would manufacture a velocity out of pure geometry.
      ++rejected_as_structure_;
      continue;
    }
    if (seenThroughAGap(points, span)) {
      // Small, but only because most of it is hidden. Its extent is a lower
      // bound on the object behind the gap, not a measurement of it, and
      // its centroid tracks the moving window rather than any object.
      ++rejected_as_occluded_;
      continue;
    }
    if (isFlatSurface(cl)) {
      // Straight, so it is a piece of surface rather than an object, however
      // short it happens to be. Extent cannot make this distinction: a wall
      // is locally straight at every width, and the 0.4 m of it visible
      // between two doorframes is well under the extent limit while still
      // being a wall -- whose centroid slides as the robot moves, which is
      // exactly how a stationary building acquires a velocity.
      ++rejected_as_flat_;
      continue;
    }
    // Robot frame -> fixed frame. Differencing in the robot frame would
    // report every static object as moving at the robot's own speed.
    candidates.push_back({robot.x + c * cx - s * cy,
                          robot.y + s * cx + c * cy, extent});
  }

  // Count split and merge BEFORE associating, because the greedy loop below
  // consumes the evidence: once a candidate claims a track, the fact that a
  // second candidate was also inside that track's gate is gone. Diagnosis
  // only -- nothing here changes which candidate wins.
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    std::size_t in_gate = 0;
    for (const auto & cand : candidates) {
      if (std::hypot(tracks_[i].x - cand.x, tracks_[i].y - cand.y)
          < config_.association_gate_m) {++in_gate;}
    }
    if (in_gate > 1) {++stats_.split_events;}
  }
  for (const auto & cand : candidates) {
    std::size_t in_gate = 0;
    for (const auto & tr : tracks_) {
      if (std::hypot(tr.x - cand.x, tr.y - cand.y)
          < config_.association_gate_m) {++in_gate;}
    }
    if (in_gate > 1) {++stats_.merge_events;}
  }

  // Which candidates and tracks are AMBIGUOUS. A track with two candidates
  // in its gate came apart; a candidate with two tracks in its gate is two
  // things that just became one. In both cases the winning pair is not the
  // same physical object twice, so differencing its centroids measures the
  // regrouping rather than any motion. The association still happens -- the
  // obstacle must stay tracked -- but no velocity is taken from it, which is
  // the same choice the speed cap already makes for a different reason.
  std::vector<bool> track_ambiguous(tracks_.size(), false);
  std::vector<bool> cand_ambiguous(candidates.size(), false);
  for (std::size_t i = 0; i < tracks_.size(); ++i) {
    std::size_t n = 0;
    for (const auto & cand : candidates) {
      if (std::hypot(tracks_[i].x - cand.x, tracks_[i].y - cand.y)
          < config_.association_gate_m) {++n;}
    }
    if (n > 1) {track_ambiguous[i] = true;}
  }
  for (std::size_t j = 0; j < candidates.size(); ++j) {
    std::size_t n = 0;
    for (const auto & tr : tracks_) {
      if (std::hypot(tr.x - candidates[j].x, tr.y - candidates[j].y)
          < config_.association_gate_m) {++n;}
    }
    if (n > 1) {cand_ambiguous[j] = true;}
  }

  // Assign globally nearest pair first, rather than walking the candidates in
  // SCAN ORDER and letting each take the nearest track still free. Scan order
  // is an accident of where the laser started: the first candidate could take
  // a track that fitted the second one far better, and the displaced track
  // then either coasts or attaches to something else and invents a velocity.
  // Sorting every in-gate pair by distance costs nothing at these sizes and
  // removes the dependence on where the scan begins.
  struct Pair { double d; std::size_t track, cand; };
  std::vector<Pair> pairs;
  for (std::size_t j = 0; j < candidates.size(); ++j) {
    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      const double d = std::hypot(tracks_[i].x - candidates[j].x,
                                  tracks_[i].y - candidates[j].y);
      if (d < config_.association_gate_m) {pairs.push_back({d, i, j});}
    }
  }
  std::sort(pairs.begin(), pairs.end(),
            [](const Pair & a, const Pair & b) {
              if (a.d != b.d) {return a.d < b.d;}
              if (a.track != b.track) {return a.track < b.track;}
              return a.cand < b.cand;        // total order, so it is stable
            });

  std::vector<std::size_t> match_of(candidates.size(), tracks_.size());
  std::vector<bool> claimed(tracks_.size(), false);
  std::vector<bool> taken(candidates.size(), false);
  for (const auto & pr : pairs) {
    if (claimed[pr.track] || taken[pr.cand]) {continue;}
    claimed[pr.track] = true;
    taken[pr.cand] = true;
    match_of[pr.cand] = pr.track;
  }

  std::vector<ObstacleTrack> updated;
  updated.reserve(candidates.size() + tracks_.size());

  for (std::size_t j = 0; j < candidates.size(); ++j) {
    const auto & cand = candidates[j];
    const std::size_t best = match_of[j];

    if (best == tracks_.size()) {
      ObstacleTrack t;
      t.id = next_id_++;
      t.x = cand.x;
      t.y = cand.y;
      t.extent_m = cand.extent;
      t.hits = 1;
      t.last_seen_s = stamp_s;
      t.velocity_updated_s = stamp_s;
      ++stats_.tracks_created;
      updated.push_back(t);
      continue;
    }

    ObstacleTrack t = tracks_[best];
    const bool ambiguous = track_ambiguous[best] || cand_ambiguous[j];
    if (ambiguous) {++stats_.ambiguous_associations;}
    if (can_differentiate && !ambiguous) {
      const double raw_vx = (cand.x - t.x) / dt;
      const double raw_vy = (cand.y - t.y) / dt;
      const double speed = std::hypot(raw_vx, raw_vy);
      if (speed > config_.max_tracked_speed_m_s) {
        ++stats_.speed_rejections;
      }
      if (speed <= config_.max_tracked_speed_m_s) {
        // First-order low pass. alpha = dt / (tau + dt) is the exact
        // discretisation and stays stable for any dt.
        const double alpha = (config_.velocity_filter_tau_s > 0.0)
          ? dt / (config_.velocity_filter_tau_s + dt) : 1.0;
        t.vx += alpha * (raw_vx - t.vx);
        t.vy += alpha * (raw_vy - t.vy);
        t.velocity_updated_s = stamp_s;
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
    ++stats_.tracks_matched;
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
    } else {
      // Died of old age. How long it lived and whether it was ever believed
      // separates a phantom from an object that went behind something: a
      // split produces tracks that expire young and untrusted.
      ++stats_.tracks_aged_out;
      stats_.dead_hits_total += static_cast<std::size_t>(tracks_[i].hits);
      if (tracks_[i].trusted) {++stats_.died_trusted;}
      else {++stats_.died_untrusted;}
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
    // A velocity nobody has been able to confirm lately is not evidence of
    // motion. Without this the first spurious velocity a track acquires
    // outlives every chance to disprove it.
    if (latest_stamp_s_ - t.velocity_updated_s > config_.max_velocity_age_s) {
      continue;
    }
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
