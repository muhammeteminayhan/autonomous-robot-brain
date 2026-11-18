// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_PLANNING__OBSTACLE_TRACKER_HPP_
#define ARB_PLANNING__OBSTACLE_TRACKER_HPP_

#include <cstddef>
#include <vector>

#include "arb_core/pose2d.hpp"

namespace arb_planning
{

/// Estimates how fast the things in a laser scan are moving.
///
/// ======================= WHY THIS EXISTS =======================
///
/// Phase 11 measured the Dynamic Window Approach driving into a crossing
/// obstacle in three runs out of three, overlapping it by 0.23 m every time.
/// The trace showed DWA commanding its first stop with 0.027 m of clearance
/// while travelling 0.437 m/s and needing 0.096 m to stop.
///
/// It was not carelessness. DWA rolls a trajectory forward through the
/// obstacle positions it can see RIGHT NOW and rejects anything that
/// intersects them. A crossing obstacle is not in the swept volume when the
/// rollout is computed; it walks into that volume afterwards. Every 50 ms
/// cycle the planner correctly re-established that the path ahead was clear --
/// of where the obstacle had been.
///
/// The admissible-velocity condition `v <= sqrt(2 d a)` was working exactly as
/// designed. `d` is the distance to collision ASSUMING THE WORLD HOLDS STILL,
/// and that assumption is what failed. Fixing it needs obstacle velocity, and
/// a laser scan does not come with any: it is a set of points with no
/// correspondence between one frame and the next. This class manufactures that
/// correspondence.
///
/// ========================= HOW =========================
///
///   1. SEGMENT the scan into clusters, breaking wherever consecutive returns
///      jump further apart than `cluster_gap_m`.
///   2. REJECT clusters wider than `max_cluster_extent_m`. Those are walls,
///      and a wall's centroid slides along it as the robot moves, which would
///      manufacture a velocity out of pure geometry.
///   3. TRANSFORM each centroid into the FIXED frame. This is the step that
///      is easy to skip and fatal to skip -- see below.
///   4. ASSOCIATE each cluster with the nearest track from the previous cycle,
///      within `association_gate_m`. Nearest-neighbour, not Hungarian: with a
///      handful of clusters a metres apart the assignment is unambiguous, and
///      an optimal assignment over an ambiguous one is false precision.
///   5. DIFFERENTIATE the matched centroid and low-pass the result.
///
/// =============== THE STEP THAT MUST NOT BE SKIPPED ===============
///
/// The scan arrives in the ROBOT frame, and the robot is moving. In that frame
/// a stationary wall sweeps past at the robot's own speed. Differencing
/// centroids there would report every static object as moving at -v, the
/// planner would predict the whole world sliding towards it, and it would
/// refuse to move at all.
///
/// So centroids are converted to a fixed frame before differencing, using the
/// robot pose handed in with the scan, and the resulting world velocity is
/// rotated back into the robot frame for the planner to use. A static obstacle
/// then measures zero however fast the robot travels, which is the property
/// `StaticObstaclesMeasureZeroVelocityWhileTheRobotMoves` asserts.
struct TrackerConfig
{
  /// Consecutive returns further apart than this start a new cluster. Must
  /// exceed the beam spacing at the ranges that matter: at 3 m with a 1 degree
  /// increment, neighbouring beams on one flat surface are 5 cm apart.
  double cluster_gap_m{0.18};

  /// Clusters wider than this are structure, not objects, and are not tracked.
  /// The crossing obstacle is 0.40 m across; a corridor wall is metres.
  double max_cluster_extent_m{1.00};
  /// How much nearer a cluster's angular neighbours must be before the
  /// cluster counts as something seen THROUGH a gap rather than an object
  /// standing in the open.
  ///
  /// Extent alone cannot tell "a small object" from "a small piece of a big
  /// one". A wall seen through a doorway returns a section well under
  /// `max_cluster_extent_m`, so the structure filter has nothing to reject,
  /// and the visible window slides along the wall as the robot moves --
  /// carrying the centroid with it at roughly the robot's own speed. That
  /// was the dominant source of phantom movers in this stack.
  ///
  /// The discriminator is what BOUNDS the cluster. An object standing in
  /// the open is bounded by returns from further away, or by no return at
  /// all. A view through an aperture is bounded on both sides by returns
  /// that are NEARER -- the thing being looked past.
  double aperture_margin_m{0.10};
  /// How much a cluster must bend, per metre of its own length, before it
  /// counts as an object rather than a piece of flat surface.
  ///
  /// Extent says how BIG a cluster is; this says what SHAPE it is, and shape
  /// is what separates the two. A wall is locally straight at any width, so
  /// a piece of one deviates from a fitted line only by the sensor noise. A
  /// cylinder cannot be straight: its visible arc bends.
  ///
  /// It is a RATIO, not a distance, and that is the whole point. A short
  /// cluster bends less in absolute terms, so a fixed distance threshold
  /// quietly means "objects must be large". Measured here: a half-occluded
  /// 0.20 m cylinder deviates 0.024 m, which is under any threshold that
  /// still catches walls -- and dropping a half-occluded obstacle is the
  /// dangerous direction to be wrong in. Divided by its own length the same
  /// cluster scores 0.078, well clear.
  ///
  /// Measured over ranges 1.5-7 m and occlusions down to 40 % visible: the
  /// worst cylinder scores 0.063, the worst wall 0.020. The threshold sits
  /// between them with about 1.8x of margin on each side.
  double min_bend_ratio{0.035};
  std::size_t min_points_for_shape{5};

  std::size_t min_cluster_points{2};

  /// Largest centroid motion between cycles that still counts as the same
  /// object. Generous: a 1.5 m/s obstacle at 10 Hz moves 0.15 m, and a
  /// partially occluded cluster's centroid shifts further than its object does.
  double association_gate_m{0.45};

  /// Low-pass time constant on the velocity estimate. Differentiating a
  /// centroid that jitters by a centimetre at 10 Hz yields 0.1 m/s of noise,
  /// and an unfiltered estimate makes the planner swerve around nothing.
  double velocity_filter_tau_s{0.30};

  /// Estimates below this are reported as zero. A static object whose centroid
  /// shimmers must not be predicted into the robot's path.
  double min_tracked_speed_m_s{0.12};

  /// Speeds above this are rejected as association failures rather than
  /// believed. Nothing in this world moves at 3 m/s, so such a reading means
  /// two different objects were matched to each other.
  double max_tracked_speed_m_s{3.00};

  /// A track not seen for this long is forgotten.
  double max_track_age_s{0.60};
  /// How long a velocity may go unconfirmed before the track stops being
  /// reported as moving.
  ///
  /// Ambiguous associations do not produce a velocity, which is right --
  /// but leaving the old one in place is not. A spurious velocity, once
  /// acquired, then outlives every chance to disprove it, keeps the track
  /// above the moving threshold indefinitely, and the planner spends the
  /// run dodging it. Measured: one run timed out 15.8 m from its goal that
  /// way, scoring a flawless 6.03 m of obstacle clearance by never going
  /// anywhere near it.
  double max_velocity_age_s{0.40};

  /// Tracks must survive this many consecutive cycles before their velocity is
  /// reported. One association can be a coincidence; three in a row is an
  /// object.
  int min_hits_before_trusted{3};
};

/// One tracked cluster, in the FIXED frame.
/// A census of what the tracker did, for diagnosis rather than for control.
///
/// The phantom rate alone says "the tracker sees 1.7 movers in a world with
/// one". It cannot say WHY, and the three plausible causes want three
/// different fixes: clusters splitting between scans, two objects associated
/// to each other, or short-lived noise that never should have been trusted.
/// Counting them separately is what makes the next change measurable instead
/// of hopeful.
struct TrackerStats
{
  std::size_t tracks_created{0};
  std::size_t tracks_matched{0};
  std::size_t tracks_aged_out{0};
  /// One track with two or more candidates inside its gate: the cluster it
  /// was following came apart, and whichever half wins moves the centroid.
  std::size_t split_events{0};
  /// One candidate with two or more tracks inside its gate: two things the
  /// tracker believed were separate are now one cluster.
  std::size_t merge_events{0};
  /// Associations whose implied speed exceeded the cap, which means the
  /// match was wrong rather than the world fast.
  std::size_t speed_rejections{0};
  /// Associations made while the pairing was ambiguous -- a track with two
  /// candidates in its gate, or a candidate with two tracks. The match is
  /// kept so the obstacle stays tracked; the velocity is not, because the
  /// two centroids are not the same physical object twice.
  std::size_t ambiguous_associations{0};
  /// How dead tracks ended: a phantom from a split is usually short-lived
  /// and never earns trust, so the split between these two says how much of
  /// the problem is transient.
  std::size_t died_untrusted{0};
  std::size_t died_trusted{0};
  std::size_t dead_hits_total{0};

  double meanLifetimeHits() const
  {
    const std::size_t dead = died_untrusted + died_trusted;
    return dead ? static_cast<double>(dead_hits_total) / dead : 0.0;
  }
};

struct ObstacleTrack
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  double extent_m{0.0};
  int hits{0};
  double last_seen_s{0.0};
  /// When the velocity was last computed from an unambiguous association.
  /// A velocity nobody has confirmed lately is not evidence of motion.
  double velocity_updated_s{0.0};
  bool trusted{false};         ///< enough hits for the velocity to be believed
};

/// A scan point in the robot frame, as handed to the tracker.
struct ScanPoint
{
  double x{0.0};
  double y{0.0};
};

class ObstacleTracker
{
public:
  ObstacleTracker() = default;
  explicit ObstacleTracker(const TrackerConfig & config) : config_(config) {}

  /// Feed one scan.
  ///
  /// \param points    scan returns in the ROBOT frame
  /// \param robot     the robot's pose in the FIXED frame at this instant
  /// \param stamp_s   the scan's timestamp, in seconds, monotonic
  void update(const std::vector<ScanPoint> & points,
              const arb_core::Pose2D & robot, double stamp_s);

  /// Current tracks, in the FIXED frame.
  const std::vector<ObstacleTrack> & tracks() const {return tracks_;}

  /// Tracks whose velocity is trusted and non-negligible, expressed in the
  /// ROBOT frame: position relative to the robot, and the obstacle's world
  /// velocity rotated into the robot's axes.
  ///
  /// The velocity is the obstacle's own, NOT its velocity relative to the
  /// robot, and subtracting the robot's would be a bug. A DWA rollout starts
  /// at the origin and integrates the robot forward, so the frame it works
  /// in is "the robot frame as it was when planning began" -- which stands
  /// still while the rollout runs. Ego-motion is already accounted for by
  /// the robot moving inside that frame; removing it here would count it
  /// twice and make an obstacle appear to chase the robot.
  std::vector<ObstacleTrack> movingInRobotFrame(const arb_core::Pose2D & robot) const;

  /// Segment a scan into clusters. Exposed for testing; the gap rule is the
  /// part most likely to be wrong and the easiest to check in isolation.
  /// A half-open range of indices into the scan the clusters came from.
  /// Needed because what bounds a cluster matters as much as its contents,
  /// and a vector of points has thrown that away.
  struct ClusterSpan
  {
    std::size_t begin{0};
    std::size_t end{0};
  };

  /// Clusters as index ranges, in scan order.
  static std::vector<ClusterSpan> clusterSpans(
    const std::vector<ScanPoint> & points, double gap_m,
    std::size_t min_points);

  static std::vector<std::vector<ScanPoint>> cluster(
    const std::vector<ScanPoint> & points, double gap_m,
    std::size_t min_points);

  /// Centroid and greatest point-to-point extent of a cluster.
  static void describe(const std::vector<ScanPoint> & cluster,
                       double * cx, double * cy, double * extent);

  const TrackerConfig & config() const {return config_;}
  void reset() {tracks_.clear(); next_id_ = 1; have_previous_ = false;}

  /// How many clusters the last update discarded for being too wide. A useful
  /// diagnostic: if it is zero in a corridor, the extent limit is too loose
  /// and walls are being tracked as objects.
  std::size_t lastRejectedAsStructure() const {return rejected_as_structure_;}

  /// Clusters dropped in the last update for being a view through a gap.
  std::size_t lastRejectedAsOccluded() const {return rejected_as_occluded_;}

  /// Clusters dropped in the last update for being flat.
  std::size_t lastRejectedAsFlat() const {return rejected_as_flat_;}

  /// Cumulative census since construction or the last resetStats().
  const TrackerStats & stats() const {return stats_;}
  void resetStats() {stats_ = TrackerStats{};}

private:
  TrackerConfig config_;
  std::vector<ObstacleTrack> tracks_;
  int next_id_{1};
  double previous_stamp_s_{0.0};
  bool have_previous_{false};
  /// True when the cluster is bounded on both sides by nearer returns,
  /// which means it is a piece of something seen through a gap.
  bool seenThroughAGap(const std::vector<ScanPoint> & points,
                       const ClusterSpan & span) const;

  /// True when the cluster's points lie on a straight line to within the
  /// sensor's noise, which makes it a piece of surface rather than an object.
  bool isFlatSurface(const std::vector<ScanPoint> & cluster) const;

  std::size_t rejected_as_structure_{0};
  std::size_t rejected_as_occluded_{0};
  std::size_t rejected_as_flat_{0};
  TrackerStats stats_;
  double latest_stamp_s_{0.0};
};

}  // namespace arb_planning

#endif  // ARB_PLANNING__OBSTACLE_TRACKER_HPP_
