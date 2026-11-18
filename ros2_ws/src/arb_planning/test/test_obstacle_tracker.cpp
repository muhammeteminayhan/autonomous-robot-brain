// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_planning/obstacle_tracker.hpp"

using arb_core::Pose2D;
using arb_planning::ObstacleTrack;
using arb_planning::ObstacleTracker;
using arb_planning::ScanPoint;
using arb_planning::TrackerConfig;

namespace
{
/// A small round object, sampled as an arc of scan returns in the ROBOT frame.
std::vector<ScanPoint> blob(double cx, double cy, double radius = 0.20,
                            int points = 7)
{
  std::vector<ScanPoint> out;
  const double span = 0.9;            // radians of arc actually visible
  for (int i = 0; i < points; ++i) {
    const double a = std::atan2(cy, cx) + span * (i / double(points - 1) - 0.5);
    const double r = std::hypot(cx, cy) - radius;
    out.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return out;
}

/// Express world-frame points in the frame of a robot at `robot`.
std::vector<ScanPoint> toRobot(const std::vector<ScanPoint> & world,
                               const Pose2D & robot)
{
  std::vector<ScanPoint> out;
  const double c = std::cos(-robot.theta), s = std::sin(-robot.theta);
  for (const auto & p : world) {
    const double dx = p.x - robot.x, dy = p.y - robot.y;
    out.push_back({c * dx - s * dy, s * dx + c * dy});
  }
  return out;
}

std::vector<ScanPoint> worldBlob(double cx, double cy, int points = 7)
{
  std::vector<ScanPoint> out;
  for (int i = 0; i < points; ++i) {
    const double a = 2.0 * M_PI * i / points;
    out.push_back({cx + 0.18 * std::cos(a), cy + 0.18 * std::sin(a)});
  }
  return out;
}
}  // namespace

// ============================================================ clustering ====
TEST(Cluster, SplitsOnAGapAndKeepsAdjacentPointsTogether)
{
  std::vector<ScanPoint> pts = {
    {1.0, 0.00}, {1.0, 0.05}, {1.0, 0.10},      // one object
    {1.0, 0.90}, {1.0, 0.95},                   // another, 0.80 m away
  };
  const auto c = ObstacleTracker::cluster(pts, 0.18, 2);
  ASSERT_EQ(c.size(), 2u);
  EXPECT_EQ(c[0].size(), 3u);
  EXPECT_EQ(c[1].size(), 2u);
}

TEST(Cluster, DropsClustersBelowTheMinimumSize)
{
  std::vector<ScanPoint> pts = {{1.0, 0.0}, {1.0, 0.9}, {1.0, 0.95}};
  const auto c = ObstacleTracker::cluster(pts, 0.18, 2);
  ASSERT_EQ(c.size(), 1u) << "the lone point should not have become a cluster";
  EXPECT_EQ(c[0].size(), 2u);
}

TEST(Cluster, HandlesAnEmptyScan)
{
  EXPECT_TRUE(ObstacleTracker::cluster({}, 0.18, 2).empty());
}

TEST(Describe, ReportsCentroidAndExtent)
{
  std::vector<ScanPoint> c = {{1.0, -0.2}, {1.0, 0.0}, {1.0, 0.2}};
  double cx = 0, cy = 0, extent = 0;
  ObstacleTracker::describe(c, &cx, &cy, &extent);
  EXPECT_NEAR(cx, 1.0, 1e-12);
  EXPECT_NEAR(cy, 0.0, 1e-12);
  EXPECT_NEAR(extent, 0.4, 1e-12);
}

// ====================================================== the critical test ====
TEST(Tracker, StaticObstaclesMeasureZeroVelocityWhileTheRobotMoves)
{
  // THE failure this class exists to avoid. The scan arrives in the ROBOT
  // frame, so a stationary object sweeps past at the robot's own speed.
  // Differencing centroids there reports every static object as moving at -v,
  // the planner predicts the world sliding towards it, and it refuses to move.
  ObstacleTracker tracker{TrackerConfig{}};
  const auto world = worldBlob(4.0, 0.0);

  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 12; ++i) {
    const double t = i * 0.1;
    robot.x = 0.5 * t;                       // the ROBOT moves at 0.5 m/s
    tracker.update(toRobot(world, robot), robot, t);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  const auto & tr = tracker.tracks().front();
  EXPECT_NEAR(std::hypot(tr.vx, tr.vy), 0.0, 0.05)
    << "a static object measured " << std::hypot(tr.vx, tr.vy)
    << " m/s while the robot drove at 0.5 m/s";
  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "a static object was reported as moving";
}

TEST(Tracker, MeasuresTheVelocityOfACrossingObstacle)
{
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  const double speed = 0.45;
  for (int i = 0; i < 20; ++i) {
    const double t = i * 0.1;
    // Crossing left to right in front of a stationary robot.
    const auto world = worldBlob(3.0, -1.5 + speed * t);
    tracker.update(toRobot(world, robot), robot, t);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  const auto & tr = tracker.tracks().front();
  EXPECT_NEAR(tr.vx, 0.0, 0.08);
  EXPECT_NEAR(tr.vy, speed, 0.10)
    << "measured " << tr.vy << " m/s against a true " << speed;

  const auto moving = tracker.movingInRobotFrame(robot);
  ASSERT_EQ(moving.size(), 1u);
  EXPECT_NEAR(moving.front().x, 3.0, 0.3);
  EXPECT_NEAR(moving.front().vy, speed, 0.10);
}

TEST(Tracker, VelocityIsRotatedIntoTheRobotFrame)
{
  // Same crossing obstacle, but the robot faces 90 degrees away. What was
  // motion in +y of the world must come out as motion in -x of the robot.
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, M_PI / 2.0);
  const double speed = 0.5;
  for (int i = 0; i < 20; ++i) {
    const double t = i * 0.1;
    const auto world = worldBlob(0.0, 2.0 + speed * t);
    tracker.update(toRobot(world, robot), robot, t);
  }
  const auto moving = tracker.movingInRobotFrame(robot);
  ASSERT_EQ(moving.size(), 1u);
  // World +y, robot facing +y => straight ahead in the robot frame.
  EXPECT_NEAR(moving.front().vx, speed, 0.10);
  EXPECT_NEAR(moving.front().vy, 0.0, 0.08);
}

TEST(Tracker, DoesNotTrackWallsAsObjects)
{
  // A wall is wide, and its centroid slides along it as the robot passes,
  // which would manufacture a velocity out of pure geometry.
  TrackerConfig cfg;
  ObstacleTracker tracker{cfg};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 10; ++i) {
    const double t = i * 0.1;
    robot.x = 0.5 * t;
    std::vector<ScanPoint> wall;              // 4 m of flat wall at y = 2
    for (double x = -2.0; x <= 2.0; x += 0.10) {wall.push_back({x, 2.0});}
    tracker.update(toRobot(wall, robot), robot, t);
  }
  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "a wall was reported as a moving object";
  EXPECT_GT(tracker.lastRejectedAsStructure(), 0u)
    << "the wall was never rejected for being too wide";
}

TEST(Tracker, NeedsSeveralHitsBeforeItsVelocityIsTrusted)
{
  // One association can be a coincidence. A planner that swerves on the first
  // frame of a new blob swerves at noise.
  TrackerConfig cfg;
  ObstacleTracker tracker{cfg};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < cfg.min_hits_before_trusted - 1; ++i) {
    const double t = i * 0.1;
    tracker.update(toRobot(worldBlob(3.0, -1.0 + 0.5 * t), robot), robot, t);
  }
  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "velocity was trusted after fewer than "
    << cfg.min_hits_before_trusted << " hits";
}

TEST(Tracker, RejectsAnImplausibleSpeedAsAnAssociationFailure)
{
  // Two different objects matched to each other produce a huge apparent
  // speed. That is a tracking failure, not a fast obstacle, and believing it
  // would have the planner dodge something that does not exist.
  TrackerConfig cfg;
  cfg.max_tracked_speed_m_s = 1.0;
  cfg.association_gate_m = 10.0;        // deliberately wide, to force the jump
  ObstacleTracker tracker{cfg};
  Pose2D robot(0.0, 0.0, 0.0);
  tracker.update(toRobot(worldBlob(3.0, 0.0), robot), robot, 0.0);
  tracker.update(toRobot(worldBlob(3.0, 0.1), robot), robot, 0.1);
  const double before = std::hypot(tracker.tracks().front().vx,
                                   tracker.tracks().front().vy);
  // Now teleport it 5 m in one 0.1 s step: 50 m/s.
  tracker.update(toRobot(worldBlob(3.0, 5.1), robot), robot, 0.2);
  const double after = std::hypot(tracker.tracks().front().vx,
                                  tracker.tracks().front().vy);
  EXPECT_LT(after, cfg.max_tracked_speed_m_s)
    << "believed a " << after << " m/s obstacle";
  EXPECT_NEAR(after, before, 0.3) << "the implausible jump should be ignored";
}

TEST(Tracker, ForgetsATrackThatStopsBeingSeen)
{
  TrackerConfig cfg;
  cfg.max_track_age_s = 0.3;
  ObstacleTracker tracker{cfg};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 5; ++i) {
    tracker.update(toRobot(worldBlob(3.0, 0.0), robot), robot, i * 0.1);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  for (int i = 5; i < 15; ++i) {
    tracker.update({}, robot, i * 0.1);       // nothing in view any more
  }
  EXPECT_TRUE(tracker.tracks().empty()) << "a vanished track was never aged out";
}

TEST(Tracker, SurvivesOneMissedFrameWithoutLosingItsVelocity)
{
  // An obstacle occluded for a single scan must not come back as a brand-new
  // object with no velocity -- that is exactly when the prediction is needed.
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  int id = 0;
  for (int i = 0; i < 10; ++i) {
    const double t = i * 0.1;
    tracker.update(toRobot(worldBlob(3.0, -1.0 + 0.5 * t), robot), robot, t);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  id = tracker.tracks().front().id;
  const double v_before = tracker.tracks().front().vy;

  tracker.update({}, robot, 1.0);             // one frame fully occluded
  ASSERT_FALSE(tracker.tracks().empty()) << "dropped after a single miss";
  EXPECT_EQ(tracker.tracks().front().id, id) << "came back as a new object";
  EXPECT_NEAR(tracker.tracks().front().vy, v_before, 1e-9)
    << "lost its velocity while occluded";
}

TEST(Tracker, AClockThatDoesNotAdvanceProducesNoVelocity)
{
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 8; ++i) {
    // Same timestamp every call: a paused simulator or a replayed bag.
    tracker.update(toRobot(worldBlob(3.0, 0.1 * i), robot), robot, 1.0);
  }
  for (const auto & t : tracker.tracks()) {
    EXPECT_TRUE(std::isfinite(t.vx) && std::isfinite(t.vy));
    EXPECT_NEAR(std::hypot(t.vx, t.vy), 0.0, 1e-9);
  }
}

TEST(Tracker, AClockThatRUNSBACKWARDSProducesNoVelocity)
{
  // The gap the zero-dt test missed. A non-advancing clock is already caught
  // by the implausible-speed guard, because dividing by zero gives an
  // infinity that fails the speed check. A clock that goes BACKWARDS is not:
  // the division succeeds, the sign of the velocity inverts, and the filter
  // coefficient dt/(tau+dt) goes negative, which is unstable.
  //
  // Backwards time is not hypothetical in ROS -- a looped bag replay and a
  // simulator reset both do it.
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 8; ++i) {
    tracker.update(toRobot(worldBlob(3.0, -1.0 + 0.5 * i * 0.1), robot),
                   robot, i * 0.1);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  const double before_vx = tracker.tracks().front().vx;
  const double before_vy = tracker.tracks().front().vy;

  // Clock jumps backwards, with the obstacle WHERE THE TRACK EXPECTS IT.
  // Feeding it somewhere else would exceed the association gate and create a
  // second track, and the test would then be measuring association, not the
  // clock. After 8 steps at 0.5 m/s from y = -1.0 the obstacle is at -0.65.
  tracker.update(toRobot(worldBlob(3.0, -0.65), robot), robot, -0.2);
  for (const auto & tr : tracker.tracks()) {
    EXPECT_TRUE(std::isfinite(tr.vx) && std::isfinite(tr.vy))
      << "a backwards clock produced a non-finite velocity";
    EXPECT_LE(std::hypot(tr.vx, tr.vy), TrackerConfig{}.max_tracked_speed_m_s)
      << "a backwards clock produced " << std::hypot(tr.vx, tr.vy) << " m/s";
  }
  EXPECT_NEAR(tracker.tracks().front().vx, before_vx, 1e-9)
    << "velocity changed on a backwards clock step";
  EXPECT_NEAR(tracker.tracks().front().vy, before_vy, 1e-9);
}

TEST(Tracker, APoseThatJUMPSMakesTheStaticWorldLookAlive)
{
  // A CHARACTERISATION test: this documents a limitation, not a feature.
  //
  // The tracker differences centroids in whatever frame it is handed. It
  // cannot tell a pose correction from real motion -- nothing can, from one
  // scan -- so when the frame jumps, the entire static world lurches at once.
  //
  // That is not hypothetical. The first wiring fed it `map`, and REP-105
  // makes map -> odom discontinuous ON PURPOSE: every accepted scan match
  // jumps it. Measured in the simulator, the tracker then reported 5 to 8
  // moving obstacles in a world containing exactly one, the planner swerved
  // away from walls that were not moving, and the run meant to demonstrate
  // obstacle avoidance collided and took 106 s instead of 39 s.
  //
  // The fix is the caller's: hand it `odom`, which drifts but never jumps.
  // This test exists so nobody quietly puts `map` back.
  ObstacleTracker tracker{TrackerConfig{}};
  const auto world = worldBlob(3.0, 0.0);      // never moves, not even once
  const Pose2D truth(0.0, 0.0, 0.0);           // robot never moves either

  // Six honest cycles: the believed pose equals the true pose.
  for (int i = 0; i < 6; ++i) {
    tracker.update(toRobot(world, truth), truth, i * 0.1);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  EXPECT_LT(std::hypot(tracker.tracks().front().vx,
                       tracker.tracks().front().vy),
            TrackerConfig{}.min_tracked_speed_m_s)
    << "a smooth pose should leave a stationary object stationary";

  // Now one accepted scan match relocates the robot 15 cm sideways. The
  // SCAN is unchanged -- the laser still sees the same unmoved object -- but
  // the pose the tracker is handed has jumped.
  const Pose2D jumped(0.0, 0.15, 0.0);
  tracker.update(toRobot(world, truth), jumped, 0.6);

  const double spurious = std::hypot(tracker.tracks().front().vx,
                                     tracker.tracks().front().vy);
  EXPECT_GT(spurious, TrackerConfig{}.min_tracked_speed_m_s)
    << "a 15 cm pose jump produced only " << spurious << " m/s of apparent "
    << "motion, so this test no longer demonstrates the hazard it exists "
    << "for -- check whether the filter or the threshold changed";

  // And it is reported as a MOVING obstacle, which is the damage: the
  // planner will now refuse perfectly good commands to dodge a phantom.
  EXPECT_FALSE(tracker.movingInRobotFrame(jumped).empty())
    << "the spurious velocity should reach the planner, which is the "
    << "entire reason the frame choice matters";
}

/// A wall segment, for the little ray caster below.
struct Segment { double x0, y0, x1, y1; };

/// Distance from `o` along unit direction `d` to a segment, or -1.
double hitDistance(double ox, double oy, double dx, double dy,
                   const Segment & s)
{
  const double ex = s.x1 - s.x0, ey = s.y1 - s.y0;
  const double denom = dx * ey - dy * ex;
  if (std::abs(denom) < 1e-12) {return -1.0;}        // parallel
  const double t = ((s.x0 - ox) * ey - (s.y0 - oy) * ex) / denom;
  const double u = ((s.x0 - ox) * dy - (s.y0 - oy) * dx) / denom;
  return (t > 1e-6 && u >= 0.0 && u <= 1.0) ? t : -1.0;
}

/// A full 360-degree scan of a closed room, in angular order.
///
/// It has to be closed and it has to be 360 degrees. A fan that stops at
/// some angle truncates whatever wall it was crossing, and the leftover
/// sliver is a small cluster bounded by further-away returns -- an object
/// by every test the tracker can apply, and an artefact of the fan rather
/// than of the world. The first version of this test was fooled by exactly
/// that, which is a fair warning about synthetic scans.
std::vector<ScanPoint> scanRoom(double rx, double ry,
                                const std::vector<Segment> & world,
                                int beams = 360)
{
  std::vector<ScanPoint> out;
  for (int i = 0; i < beams; ++i) {
    const double a = 2.0 * M_PI * i / beams;
    const double dx = std::cos(a), dy = std::sin(a);
    double best = -1.0;
    for (const auto & s : world) {
      const double d = hitDistance(rx, ry, dx, dy, s);
      if (d > 0.0 && (best < 0.0 || d < best)) {best = d;}
    }
    if (best > 0.0) {out.push_back({rx + best * dx, ry + best * dy});}
  }
  return out;
}

/// A room 6 x 3 m with a 0.30 m doorway in its north wall, and another wall
/// 1 m beyond the doorway. Nothing in it moves.
std::vector<Segment> roomWithADoorway()
{
  return {
    {-3.0, 1.0, 0.85, 1.0}, {1.15, 1.0, 3.0, 1.0},   // north wall + doorway
    {-3.0, 2.0, 3.0, 2.0},                            // the wall beyond it
    {-3.0, -2.0, 3.0, -2.0},                          // south
    {-3.0, -2.0, -3.0, 1.0}, {3.0, -2.0, 3.0, 1.0},   // west, east
  };
}

TEST(Tracker, AWallSeenThroughADoorwayIsNotAnObject)
{
  // `DoesNotTrackWallsAsObjects` passes because its wall is visible end to
  // end, so the cluster is 4 m wide and the structure filter rejects it.
  // Real walls are not seen end to end. Through a 0.30 m doorway only a
  // short section returns, the cluster sits well under the 1.0 m threshold,
  // and the filter has nothing to reject -- while the visible window slides
  // along the wall as the robot moves, carrying the centroid with it.
  //
  // Nothing here moves. The walls are fixed, the doorway is fixed, the pose
  // is exact. Any velocity reported is manufactured out of geometry.
  //
  // Measured in the simulator before the aperture rule existed, this was
  // the dominant phantom source: in a world with exactly one moving object,
  // 121 of 144 logged cycles reported more than one mover, peaking at 8,
  // and the phantom rate predicted whether the run reached its goal
  // (r = -0.73) far better than the planner setting under test did
  // (r = -0.17).
  ObstacleTracker tracker{TrackerConfig{}};
  const auto world = roomWithADoorway();

  Pose2D robot(-1.0, 0.0, 0.0);
  for (int i = 0; i < 14; ++i) {
    const double t = i * 0.1;
    robot.x = -1.0 + 0.5 * t;                        // a steady 0.5 m/s
    const auto scan = scanRoom(robot.x, robot.y, world);
    ASSERT_GT(scan.size(), 300u) << "the room is not closed at step " << i;
    tracker.update(toRobot(scan, robot), robot, t);
  }

  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "a wall seen through a doorway was reported as a moving object";
  EXPECT_GT(tracker.lastRejectedAsOccluded(), 0u)
    << "the far wall section was never recognised as a view through a gap, "
    << "so if this test passes it passes for some other reason";
}

TEST(Tracker, AnObjectHiddenDownOneSideIsStillAnObject)
{
  // This is why the rule needs BOTH neighbours to be nearer.
  //
  // Half-hidden objects are the normal case, not the exception: anything
  // standing near a corner, a doorframe or another object is clipped on one
  // side. Those clusters are bounded by a nearer return on that side and by
  // the background on the other, and they are still objects. Requiring only
  // one near side would discard them -- and a collision avoider that stops
  // believing in half-occluded obstacles is worse than one with no filter
  // at all, because the things it drops are the ones it is about to hit.
  //
  // Geometry: a stub at y = 1 casts a shadow over x in [0.4, 0.6] at y = 2,
  // clipping the left of an object spanning [0.5, 0.9]. The background is
  // the north wall 1 m behind it.
  const std::vector<Segment> world = {
    {-3.0, 1.0, -0.6, 1.0}, {0.2, 1.0, 0.3, 1.0},    // south-ish wall + stub
    {0.5, 2.0, 0.9, 2.0},                             // the object
    {-3.0, 3.0, 3.0, 3.0},                            // background wall
    {-3.0, -2.0, 3.0, -2.0},                          // south
    {-3.0, -2.0, -3.0, 3.0}, {3.0, -2.0, 3.0, 3.0},   // west, east
  };

  ObstacleTracker tracker{TrackerConfig{}};
  const Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 6; ++i) {
    const auto scan = scanRoom(robot.x, robot.y, world, 720);
    ASSERT_GT(scan.size(), 600u);
    tracker.update(scan, robot, i * 0.1);
  }

  EXPECT_EQ(tracker.lastRejectedAsOccluded(), 0u)
    << "an object clipped on ONE side was mistaken for a view through a "
    << "gap; the rule must require both neighbours to be nearer";

  bool found = false;
  for (const auto & tr : tracker.tracks()) {
    if (std::abs(tr.y - 2.0) < 0.25 && tr.x > 0.3 && tr.x < 1.1) {found = true;}
  }
  EXPECT_TRUE(found) << "the half-occluded object was not tracked at all";
}

TEST(Tracker, AnObjectInTheOpenSurvivesTheApertureRule)
{
  // The complement, and the reason the rule needs BOTH sides to be nearer:
  // an object standing in front of a background is bounded by returns from
  // FURTHER away. Rejecting on one near side would throw away every object
  // standing beside a wall, which is most of them.
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 12; ++i) {
    const double t = i * 0.1;
    std::vector<ScanPoint> scan;
    // Background wall at y = 4, left half.
    for (double x = -3.0; x < 0.6; x += 0.05) {scan.push_back({x, 4.0});}
    // A small object at y = 2, drifting sideways at 0.3 m/s.
    const double ox = 0.8 + 0.3 * t;
    for (double d = -0.15; d <= 0.15; d += 0.05) {scan.push_back({ox + d, 2.0});}
    // Background wall again, right half.
    for (double x = 1.4; x <= 3.0; x += 0.05) {scan.push_back({x, 4.0});}
    tracker.update(toRobot(scan, robot), robot, t);
  }

  EXPECT_EQ(tracker.lastRejectedAsOccluded(), 0u)
    << "an object in front of a background was mistaken for a view "
    << "through a gap";
  const auto moving = tracker.movingInRobotFrame(robot);
  ASSERT_EQ(moving.size(), 1u) << "the real mover was lost";
  EXPECT_NEAR(moving.front().vx, 0.3, 0.08);
}

TEST(Tracker, ResetClearsEverything)
{
  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(0.0, 0.0, 0.0);
  for (int i = 0; i < 5; ++i) {
    tracker.update(toRobot(worldBlob(3.0, 0.0), robot), robot, i * 0.1);
  }
  ASSERT_FALSE(tracker.tracks().empty());
  tracker.reset();
  EXPECT_TRUE(tracker.tracks().empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
