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

/// A cylinder, as the segments a ray caster can hit.
///
/// The crossing obstacle in this project is a 0.20 m cylinder, and once the
/// tracker tests SHAPE a flat stand-in stops standing in for it: a segment
/// is a piece of wall by every measure the tracker has. Two of the tests
/// below used one and had to be corrected.
std::vector<Segment> circle(double cx, double cy, double r, int sides = 24)
{
  std::vector<Segment> out;
  for (int i = 0; i < sides; ++i) {
    const double a0 = 2.0 * M_PI * i / sides;
    const double a1 = 2.0 * M_PI * (i + 1) / sides;
    out.push_back({cx + r * std::cos(a0), cy + r * std::sin(a0),
                   cx + r * std::cos(a1), cy + r * std::sin(a1)});
  }
  return out;
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

TEST(Tracker, AFlatPanelIsNotAnObject)
{
  // The case neither of the other two structure rules can reach.
  //
  // It is 0.60 m wide, so the extent limit (1.00 m) does not fire. It stands
  // in the open with the far wall behind it, so both its angular neighbours
  // are FURTHER away and the aperture rule does not fire either. It is a
  // flat surface all the same, and as the robot drives past, the part of it
  // the LiDAR can see slides along it -- which is how a stationary building
  // acquires a velocity.
  //
  // Only shape separates this from an obstacle, and shape is exactly what
  // the first two rules do not look at.
  std::vector<Segment> world = {
    {-0.3, 2.0, 0.3, 2.0},                            // the panel, 0.60 m
    {-4.0, 4.0, 4.0, 4.0},                            // background, further
    {-4.0, -2.0, 4.0, -2.0},
    {-4.0, -2.0, -4.0, 4.0}, {4.0, -2.0, 4.0, 4.0},
  };

  ObstacleTracker tracker{TrackerConfig{}};
  Pose2D robot(-1.2, 0.0, 0.0);
  for (int i = 0; i < 14; ++i) {
    const double t = i * 0.1;
    robot.x = -1.2 + 0.5 * t;                          // drive past it
    tracker.update(scanRoom(robot.x, robot.y, world, 720), robot, t);
  }

  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "a stationary flat panel was reported as a moving object";
  EXPECT_GT(tracker.lastRejectedAsFlat(), 0u)
    << "the panel was not rejected for being flat, so if this test passes "
    << "it passes for some other reason";
  EXPECT_EQ(tracker.lastRejectedAsOccluded(), 0u)
    << "the panel stands in the open; the aperture rule must not claim it";
}

TEST(Tracker, ACylinderIsNotFlatAtAnyRangeThisRobotCaresAbout)
{
  // The complement, and the one that matters for safety: the shape rule
  // must not discard the obstacle it exists to let through. A 0.20 m
  // cylinder is what crosses this robot's corridor.
  for (const double range : {1.5, 3.0, 5.0}) {
    std::vector<Segment> world = {
      {-5.0, 8.0, 5.0, 8.0}, {-5.0, -2.0, 5.0, -2.0},
      {-5.0, -2.0, -5.0, 8.0}, {5.0, -2.0, 5.0, 8.0},
    };
    for (const auto & s : circle(0.0, range, 0.20)) {world.push_back(s);}

    ObstacleTracker tracker{TrackerConfig{}};
    const Pose2D robot(0.0, 0.0, 0.0);
    tracker.update(scanRoom(robot.x, robot.y, world, 720), robot, 0.0);

    bool found = false;
    for (const auto & tr : tracker.tracks()) {
      if (std::hypot(tr.x, tr.y - range) < 0.3) {found = true;}
    }
    EXPECT_TRUE(found) << "the cylinder at " << range
                       << " m was discarded; a collision avoider that stops "
                       << "believing in obstacles is worse than one with no "
                       << "filter at all";
  }
}

TEST(Tracker, TheShapeRuleAbstainsWhenThereIsNotEnoughToJudge)
{
  // Three points are collinear whatever they came from, so below the
  // minimum the rule must abstain -- and abstaining has to mean KEEPING the
  // cluster. A shape test that guesses on thin evidence guesses away real
  // obstacles.
  TrackerConfig cfg;
  cfg.min_cluster_points = 3;
  ObstacleTracker tracker{cfg};

  const std::vector<ScanPoint> sliver = {
    {0.00, 2.00}, {0.03, 2.00}, {0.06, 2.00},          // dead straight
  };
  tracker.update(sliver, Pose2D(0.0, 0.0, 0.0), 0.0);

  EXPECT_EQ(tracker.lastRejectedAsFlat(), 0u)
    << "the rule judged a three-point cluster, where every cluster is "
    << "straight and the verdict means nothing";
  EXPECT_EQ(tracker.tracks().size(), 1u) << "the cluster was dropped anyway";
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
  const std::vector<Segment> base = {
    {-3.0, 1.0, -0.6, 1.0}, {0.2, 1.0, 0.3, 1.0},    // south-ish wall + stub
    // the object: a cylinder, which is what the world actually contains
    {-3.0, 3.0, 3.0, 3.0},                            // background wall
    {-3.0, -2.0, 3.0, -2.0},                          // south
    {-3.0, -2.0, -3.0, 3.0}, {3.0, -2.0, 3.0, 3.0},   // west, east
  };
  std::vector<Segment> world = base;
  for (const auto & s : circle(0.75, 2.0, 0.20)) {world.push_back(s);}

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
    // A small CYLINDER at y = 2, drifting sideways at 0.3 m/s. It has to be
    // curved: the tracker now tests shape, and a flat strip of points is a
    // piece of wall by every measure it has.
    const double ox = 0.8 + 0.3 * t;
    for (double a = -1.0; a <= 1.0; a += 0.22) {
      scan.push_back({ox + 0.20 * std::sin(a), 2.0 - 0.20 * std::cos(a)});
    }
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

/// A compact curved cluster centred on (cx, cy): an object, not a surface.
std::vector<ScanPoint> pebble(double cx, double cy, double r = 0.12,
                              int points = 9)
{
  std::vector<ScanPoint> out;
  for (int i = 0; i < points; ++i) {
    const double a = -1.1 + 2.2 * i / (points - 1);
    out.push_back({cx + r * std::sin(a), cy - r * std::cos(a)});
  }
  return out;
}

TEST(Tracker, TheNearestPairWinsFirst)
{
  // Walking candidates in SCAN ORDER and giving each the nearest track still
  // free is a different algorithm from matching the closest pair first, and
  // this geometry is where they disagree.
  //
  //   tracks      A at x=0.00            B at x=0.60
  //   candidates           c1 at x=0.35       c2 at x=0.65
  //
  // c1 comes first in scan order. Its nearest free track is B (0.25 away,
  // against 0.35 for A), so greedy gives it B -- and then c2, which is 0.05
  // from B and 0.65 from A, has nothing left inside the 0.45 m gate. It
  // becomes a NEW object, and A is left unmatched to coast.
  //
  // Matching the closest pair first takes B-c2 at 0.05, then A-c1 at 0.35.
  // Two tracks, two candidates, nothing invented. The cost is one sort.
  ObstacleTracker tracker{TrackerConfig{}};
  const Pose2D robot(0.0, 0.0, 0.0);

  std::vector<ScanPoint> first;
  for (const auto & p : pebble(0.00, 2.0, 0.05)) {first.push_back(p);}
  for (const auto & p : pebble(0.60, 2.0, 0.05)) {first.push_back(p);}
  tracker.update(first, robot, 0.0);
  ASSERT_EQ(tracker.tracks().size(), 2u) << "the scene did not start as two";

  const std::size_t born_before = tracker.stats().tracks_created;
  std::vector<ScanPoint> second;
  for (const auto & p : pebble(0.35, 2.0, 0.05)) {second.push_back(p);}
  for (const auto & p : pebble(0.65, 2.0, 0.05)) {second.push_back(p);}
  tracker.update(second, robot, 0.1);

  EXPECT_EQ(tracker.stats().tracks_created - born_before, 0u)
    << "a third object was invented: the nearer pair did not win first, so "
    << "a candidate took the track that fitted the other one better";
  EXPECT_EQ(tracker.tracks().size(), 2u)
    << "the world has two objects in it and the tracker should too";
}

TEST(Tracker, AnAmbiguousAssociationProducesNoVelocity)
{
  // When a track has two candidates inside its gate the cluster it was
  // following came apart, and whichever half wins is not the same physical
  // thing it was following a moment ago. Differencing those two centroids
  // measures the regrouping, not any motion.
  //
  // The match is still made -- an obstacle that splits must stay tracked --
  // but no velocity is taken from it, which is the same choice the speed cap
  // already makes for a different reason.
  ObstacleTracker tracker{TrackerConfig{}};
  const Pose2D robot(0.0, 0.0, 0.0);

  // Four cycles of one object holding still, so a track exists and is calm.
  for (int i = 0; i < 4; ++i) {
    tracker.update(pebble(0.0, 2.0), robot, i * 0.1);
  }
  ASSERT_EQ(tracker.tracks().size(), 1u);
  const double before = std::hypot(tracker.tracks().front().vx,
                                   tracker.tracks().front().vy);

  // Now it appears as two clusters, both inside the 0.45 m gate.
  std::vector<ScanPoint> split;
  for (const auto & p : pebble(-0.18, 2.0, 0.08)) {split.push_back(p);}
  for (const auto & p : pebble(0.18, 2.0, 0.08)) {split.push_back(p);}
  tracker.update(split, robot, 0.4);

  EXPECT_GT(tracker.stats().ambiguous_associations, 0u)
    << "the split was not recognised as ambiguous";
  double worst = 0.0;
  for (const auto & tr : tracker.tracks()) {
    worst = std::max(worst, std::hypot(tr.vx, tr.vy));
  }
  EXPECT_NEAR(worst, before, 1e-9)
    << "a velocity was manufactured out of a cluster coming apart";
}

TEST(Tracker, AVelocityNobodyCanConfirmStopsBeingBelieved)
{
  // Ambiguous associations produce no velocity, which is right. Leaving the
  // last one in place is not: a spurious velocity, once acquired, then
  // outlives every chance to disprove it, holds the track above the moving
  // threshold, and the planner spends the run dodging it.
  //
  // Measured before this rule: one run timed out 15.8 m from its goal and
  // scored a flawless 6.03 m of obstacle clearance by never going near the
  // obstacle at all.
  ObstacleTracker tracker{TrackerConfig{}};
  const Pose2D robot(0.0, 0.0, 0.0);

  // Six cycles of honest motion, enough to be trusted and to be moving.
  for (int i = 0; i < 6; ++i) {
    tracker.update(pebble(0.30 * i * 0.1, 2.0), robot, i * 0.1);
  }
  ASSERT_FALSE(tracker.movingInRobotFrame(robot).empty())
    << "the object was never believed to move, so this test proves nothing";

  // Now it keeps splitting: every association is ambiguous, so no velocity
  // can be confirmed. The object itself holds still.
  for (int i = 6; i < 14; ++i) {
    std::vector<ScanPoint> split;
    for (const auto & q : pebble(0.135, 2.0, 0.07)) {split.push_back(q);}
    for (const auto & q : pebble(0.495, 2.0, 0.07)) {split.push_back(q);}
    tracker.update(split, robot, i * 0.1);
  }

  EXPECT_TRUE(tracker.movingInRobotFrame(robot).empty())
    << "a velocity that has gone unconfirmed for 0.8 s is still being "
    << "reported as motion";
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
