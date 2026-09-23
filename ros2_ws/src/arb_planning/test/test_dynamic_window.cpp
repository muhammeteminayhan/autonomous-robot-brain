// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_core/angles.hpp"
#include "arb_planning/dynamic_window.hpp"

using arb_core::Pose2D;
using arb_planning::DwaConfig;
using arb_planning::DwaResult;
using arb_planning::DynamicWindowPlanner;
using arb_planning::ObstaclePoint;

namespace
{
constexpr double kDt = 0.05;

DwaConfig config()
{
  DwaConfig c;
  c.max_linear_velocity = 0.70;
  c.min_linear_velocity = -0.30;
  c.max_angular_velocity = 2.00;
  c.max_linear_acceleration = 1.00;
  c.max_angular_acceleration = 3.00;
  c.robot_radius = 0.25;
  return c;
}
}  // namespace

// =====================================================================
//  ROLLOUT
// =====================================================================

TEST(Rollout, MatchesTheAnalyticArc)
{
  // The rollout must simulate the SAME robot the estimator believes in, or the
  // planner is choosing commands for a different vehicle.
  DwaConfig c = config();
  c.simulation_time = 2.0;
  c.simulation_dt = 0.01;
  DynamicWindowPlanner p(c);

  const double v = 0.4;
  const double w = 0.6;
  const auto poses = p.rollout(v, w);
  ASSERT_EQ(poses.size(), 201u);

  const double t = c.simulation_time;
  const double R = v / w;
  EXPECT_NEAR(poses.back().x, R * std::sin(w * t), 1e-6);
  EXPECT_NEAR(poses.back().y, R * (1.0 - std::cos(w * t)), 1e-6);
  EXPECT_NEAR(poses.back().theta, arb_core::normalizeAngle(w * t), 1e-9);
}

TEST(Rollout, StraightLineWhenOmegaIsZero)
{
  DwaConfig c = config();
  c.simulation_time = 1.0;
  c.simulation_dt = 0.05;
  DynamicWindowPlanner p(c);
  const auto poses = p.rollout(0.5, 0.0);
  EXPECT_NEAR(poses.back().x, 0.5, 1e-9);
  EXPECT_NEAR(poses.back().y, 0.0, 1e-12);
  EXPECT_NEAR(poses.back().theta, 0.0, 1e-12);
}

TEST(Rollout, StartsAtTheRobotOrigin)
{
  DynamicWindowPlanner p(config());
  const auto poses = p.rollout(0.3, 0.5);
  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(poses.front().x, 0.0, 1e-15);
  EXPECT_NEAR(poses.front().y, 0.0, 1e-15);
  EXPECT_NEAR(poses.front().theta, 0.0, 1e-15);
}

TEST(DistanceToCollision, LooksBeyondTheScoringHorizon)
{
  // The safety check must not depend on the scoring horizon. With a 0.3 s
  // horizon at 0.66 m/s the robot travels 0.199 m, so a wall 0.20 m away is
  // invisible to the rollout and the admissibility check passes a command the
  // robot cannot stop from.
  DwaConfig c = config();
  c.simulation_time = 0.3;
  c.simulation_dt = 0.01;
  c.robot_radius = 0.25;
  DynamicWindowPlanner p(c);

  std::vector<ObstaclePoint> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05) {wall.push_back({0.45, y});}

  const double d = p.distanceToCollision(0.66, 0.0, wall);
  ASSERT_TRUE(std::isfinite(d)) << "the wall must be found beyond the horizon";

  // The answer is deliberately CONSERVATIVE: it rounds down to the last point
  // known to be clear, so it under-reports by up to one check step. Asserting
  // symmetric closeness would be asserting that the safety margin is absent.
  const double truth = 0.45 - c.robot_radius;
  EXPECT_LE(d, truth + 1e-9) << "the distance must never be optimistic";
  EXPECT_GE(d, truth - 2.0 * c.collision_check_step_m)
    << "and not so pessimistic that the robot refuses to move";
}

TEST(DistanceToCollision, IsInfiniteWithNoObstacles)
{
  DynamicWindowPlanner p(config());
  EXPECT_TRUE(std::isinf(p.distanceToCollision(0.5, 0.0, {})));
}

TEST(DistanceToCollision, IsZeroWhenAlreadyTouching)
{
  DwaConfig c = config();
  c.robot_radius = 0.25;
  DynamicWindowPlanner p(c);
  EXPECT_NEAR(p.distanceToCollision(0.4, 0.0, {{0.10, 0.0}}), 0.0, 1e-9);
}

TEST(Clearance, IsTheMinimumOverTheWholeTrajectory)
{
  // A trajectory that grazes an obstacle at step 3 and clears it afterwards is
  // still a collision; taking the final distance instead of the minimum is a
  // classic way to drive through things.
  DwaConfig c = config();
  c.simulation_time = 2.0;
  c.simulation_dt = 0.1;
  c.robot_radius = 0.0;
  DynamicWindowPlanner p(c);

  const auto poses = p.rollout(0.5, 0.0);           // straight, 1 m
  // Obstacle halfway along the path, well clear of the endpoint.
  const std::vector<ObstaclePoint> obstacles = {{0.5, 0.02}};
  EXPECT_NEAR(p.clearance(poses, obstacles), 0.02, 1e-9);
}

TEST(Clearance, IsInfiniteWithNoObstacles)
{
  DynamicWindowPlanner p(config());
  EXPECT_TRUE(std::isinf(p.clearance(p.rollout(0.4, 0.0), {})));
}

TEST(Clearance, AccountsForTheRobotRadius)
{
  DwaConfig c = config();
  c.robot_radius = 0.25;
  DynamicWindowPlanner p(c);
  const auto poses = p.rollout(0.0, 0.0);           // stationary at the origin
  EXPECT_NEAR(p.clearance(poses, {{1.0, 0.0}}), 0.75, 1e-9);
}

// =====================================================================
//  THE DYNAMIC WINDOW
// =====================================================================

TEST(Window, OnlyProducesReachableCommands)
{
  // Every candidate must be reachable within ONE control period. A command
  // outside the window is not risky, it is impossible, and issuing it means
  // the robot does something other than what the planner simulated.
  DwaConfig c = config();
  DynamicWindowPlanner p(c);

  for (const double v0 : {0.0, 0.3, 0.7, -0.2}) {
    for (const double w0 : {0.0, 1.0, -1.5}) {
      const DwaResult r = p.plan(v0, w0, Pose2D(3.0, 0.0, 0.0), {}, {}, kDt);
      ASSERT_TRUE(r.success) << r.failure_reason;
      EXPECT_LE(std::abs(r.v - v0), c.max_linear_acceleration * kDt + 1e-9)
        << "v jumped from " << v0 << " to " << r.v;
      EXPECT_LE(std::abs(r.omega - w0), c.max_angular_acceleration * kDt + 1e-9)
        << "omega jumped from " << w0 << " to " << r.omega;
    }
  }
}

TEST(Window, RespectsTheActuatorLimits)
{
  DwaConfig c = config();
  c.max_linear_acceleration = 100.0;    // window wide open
  c.max_angular_acceleration = 100.0;
  DynamicWindowPlanner p(c);
  const DwaResult r = p.plan(0.0, 0.0, Pose2D(5.0, 0.0, 0.0), {}, {}, kDt);
  ASSERT_TRUE(r.success);
  EXPECT_LE(r.v, c.max_linear_velocity + 1e-9);
  EXPECT_GE(r.v, c.min_linear_velocity - 1e-9);
  EXPECT_LE(std::abs(r.omega), c.max_angular_velocity + 1e-9);
}

// =====================================================================
//  BEHAVIOUR
// =====================================================================

TEST(Dwa, DrivesTowardsAGoalInOpenSpace)
{
  DynamicWindowPlanner p(config());
  const DwaResult r = p.plan(0.3, 0.0, Pose2D(4.0, 0.0, 0.0), {}, {}, kDt);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_GT(r.v, 0.0) << "an empty world straight ahead must produce forward motion";
  EXPECT_NEAR(r.omega, 0.0, 0.5);
}

TEST(Dwa, TurnsTowardsAGoalOffToTheSide)
{
  DynamicWindowPlanner p(config());
  const DwaResult left = p.plan(0.2, 0.0, Pose2D(2.0, 2.0, 0.0), {}, {}, kDt);
  const DwaResult right = p.plan(0.2, 0.0, Pose2D(2.0, -2.0, 0.0), {}, {}, kDt);
  ASSERT_TRUE(left.success);
  ASSERT_TRUE(right.success);
  EXPECT_GT(left.omega, 0.0) << "a goal to the left must produce a left turn";
  EXPECT_LT(right.omega, 0.0);
}

TEST(Dwa, NeverDrivesIntoAWallOverAClosedLoop)
{
  // The requirement is NOT "the chosen two-second rollout is collision free".
  // The robot replans every control period and can slow down on the way, so
  // demanding a clear horizon makes it stop dead with plenty of room -- which
  // an earlier version of this test asserted and an earlier version of the
  // planner obeyed, uselessly.
  //
  // The real requirement is a closed-loop one: run the controller against a
  // simulated robot and check it never touches anything. That is what the
  // admissible-velocity condition promises.
  DwaConfig c = config();
  c.simulation_time = 2.0;
  DynamicWindowPlanner p(c);

  std::vector<ObstaclePoint> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05) {wall.push_back({2.0, y});}

  // Robot starts 2 m from the wall at full speed, with the goal beyond it.
  double x = 0.0, y = 0.0, th = 0.0, v = 0.7, w = 0.0;
  double closest = 1e9;
  for (int step = 0; step < 400; ++step) {          // 20 s at 20 Hz
    // Obstacles and goal in the ROBOT frame.
    std::vector<ObstaclePoint> local;
    local.reserve(wall.size());
    const double ct = std::cos(-th), st = std::sin(-th);
    for (const auto & o : wall) {
      const double dx = o.x - x;
      const double dy = o.y - y;
      local.push_back({ct * dx - st * dy, st * dx + ct * dy});
      closest = std::min(closest, std::hypot(dx, dy));
    }
    const double gdx = 6.0 - x;
    const double gdy = 0.0 - y;
    const Pose2D goal(ct * gdx - st * gdy, st * gdx + ct * gdy, 0.0);

    const DwaResult r = p.plan(v, w, goal, {}, local, kDt);
    v = r.v;
    w = r.omega;

    // Integrate the same constant-twist model the planner simulates.
    const double half = 0.5 * w * kDt;
    const double chord = v * kDt * (std::abs(half) < 1e-9 ? 1.0 :
                                    std::sin(half) / half);
    x += chord * std::cos(th + half);
    y += chord * std::sin(th + half);
    th = arb_core::normalizeAngle(th + w * kDt);
  }
  EXPECT_GT(closest, c.robot_radius)
    << "came within " << closest << " m of the wall, closer than the "
    << c.robot_radius << " m footprint";
}

TEST(Dwa, StopsWhenEveryOptionIsBlockedAndItIsMoving)
{
  DwaConfig c = config();
  c.simulation_time = 2.0;
  DynamicWindowPlanner p(c);

  // Boxed in on every side.
  std::vector<ObstaclePoint> box;
  for (double a = 0.0; a < 2 * M_PI; a += 0.05) {
    box.push_back({0.30 * std::cos(a), 0.30 * std::sin(a)});
  }
  const DwaResult r = p.plan(0.5, 0.0, Pose2D(3.0, 0.0, 0.0), {}, box, kDt);
  EXPECT_EQ(r.candidates_admissible, 0);
  EXPECT_NEAR(r.v, 0.0, 1e-12);
  EXPECT_NEAR(r.omega, 0.0, 1e-12);
}

TEST(Dwa, RotatesInPlaceOutOfADeadEndWhenStopped)
{
  // A robot that only ever stops when blocked never leaves a dead end.
  // Rotating in place is safe when nearly stationary and is the way out.
  DwaConfig c = config();
  c.simulation_time = 2.0;
  DynamicWindowPlanner p(c);

  std::vector<ObstaclePoint> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05) {wall.push_back({0.30, y});}

  const DwaResult r = p.plan(0.0, 0.0, Pose2D(0.0, 3.0, 0.0), {}, wall, kDt);
  ASSERT_TRUE(r.success) << r.failure_reason;
  // The robot may creep forward while turning -- with 0.05 m of clearance it
  // can still stop, so a tiny forward command is admissible and sensible.
  // What matters is that it TURNS towards the goal rather than pressing on.
  EXPECT_LT(std::abs(r.v), 0.08) << "must not drive into the wall";
  EXPECT_GT(r.omega, 0.0) << "must turn towards the goal, which is to the left";
  EXPECT_GE(p.distanceToCollision(r.v, r.omega, wall),
            r.v * r.v / (2.0 * c.max_linear_acceleration) - 1e-9);
}

TEST(Dwa, WillNotOutrunItsStoppingDistance)
{
  // The admissible-velocity condition. At 0.7 m/s and 1.0 m/s^2 the robot needs
  // 0.245 m to stop, so a trajectory whose clearance is under that must be
  // refused even though nothing collides within the horizon.
  DwaConfig c = config();
  c.simulation_time = 0.3;            // short horizon: the wall is beyond it
  c.enforce_stopping_distance = true;
  DynamicWindowPlanner p(c);

  std::vector<ObstaclePoint> wall;
  for (double y = -2.0; y <= 2.0; y += 0.05) {wall.push_back({0.45, y});}

  // At 0.7 m/s with 0.18 m of room, stopping needs 0.245 m. The whole dynamic
  // window is [0.65, 0.70] -- the robot cannot slow below 0.65 in one control
  // period -- so EVERY candidate is inadmissible and the planner must refuse
  // them all and brake. That is the right answer: it is already going too
  // fast, and the only thing left is to stop as hard as it can.
  const DwaResult fast = p.plan(0.7, 0.0, Pose2D(5.0, 0.0, 0.0), {}, wall, kDt);
  EXPECT_FALSE(fast.success) << "a robot that cannot stop must report failure";
  EXPECT_EQ(fast.candidates_admissible, 0);
  EXPECT_NEAR(fast.v, 0.0, 1e-12) << "and command a stop";

  // With the check disabled the same situation is accepted and the robot
  // drives on at full speed. That is exactly what the constraint prevents.
  DwaConfig loose = c;
  loose.enforce_stopping_distance = false;
  DynamicWindowPlanner q(loose);
  const DwaResult reckless = q.plan(0.7, 0.0, Pose2D(5.0, 0.0, 0.0), {}, wall, kDt);
  ASSERT_TRUE(reckless.success);
  EXPECT_GT(reckless.v, 0.6)
    << "without the constraint the planner drives into a wall it cannot stop before";

  // And a slower robot in the same situation IS allowed to proceed, so the
  // constraint is a speed limit rather than a blanket refusal.
  const DwaResult slow = p.plan(0.25, 0.0, Pose2D(5.0, 0.0, 0.0), {}, wall, kDt);
  ASSERT_TRUE(slow.success) << slow.failure_reason;
  const double stopping = slow.v * slow.v / (2.0 * c.max_linear_acceleration);
  EXPECT_LE(stopping, p.distanceToCollision(slow.v, slow.omega, wall) + 1e-9);
}

TEST(Dwa, TheCarrotRewardsProgressRatherThanStandingStill)
{
  // The degeneracy the carrot exists to remove. Scoring the distance to the
  // NEAREST path point rewards not moving, because the path starts at the
  // robot and the robot is already on it. Measured with that version, the
  // best-scoring command every cycle was the slowest the window allowed.
  DwaConfig c = config();
  c.path_weight = 4.0;
  c.path_lookahead_m = 1.0;
  DynamicWindowPlanner p(c);

  std::vector<Pose2D> straight;
  for (double t = 0.0; t <= 3.0; t += 0.05) {straight.emplace_back(t, 0.0, 0.0);}

  const DwaResult r = p.plan(0.3, 0.0, Pose2D(3.0, 0.0, 0.0), straight, {}, kDt);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_GE(r.v, 0.3) << "following a straight path must not choose to slow down";
}

TEST(Dwa, PrefersTheGlobalPathOverCuttingTheCorner)
{
  // Heading alone happily cuts through a room the global planner routed around
  // for a reason. The path term is what keeps the local planner subordinate to
  // the global one.
  DwaConfig c = config();
  c.path_weight = 4.0;
  c.heading_weight = 1.0;
  DynamicWindowPlanner p(c);

  // Goal straight ahead, but the path goes left first.
  const Pose2D goal(3.0, 0.0, 0.0);
  std::vector<Pose2D> path;
  for (double t = 0.0; t <= 1.0; t += 0.05) {path.emplace_back(t * 1.0, t * 1.5, 0.0);}

  const DwaResult with_path = p.plan(0.3, 0.0, goal, path, {}, kDt);
  ASSERT_TRUE(with_path.success);
  const DwaResult without = p.plan(0.3, 0.0, goal, {}, {}, kDt);
  ASSERT_TRUE(without.success);
  EXPECT_GT(with_path.omega, without.omega)
    << "the path term must pull the robot towards the route, not the goal";
}

TEST(Dwa, ScoresAreNormalisedSoWeightsMeanSomething)
{
  // Raw clearance is in metres and raw heading in radians; summing them with
  // fixed weights makes the balance depend on the size of the room. Doubling
  // every distance in the problem must not change the decision.
  DwaConfig c = config();
  DynamicWindowPlanner p(c);

  std::vector<ObstaclePoint> near_obs = {{1.2, 0.6}, {1.2, -0.6}};
  std::vector<ObstaclePoint> far_obs = {{2.4, 1.2}, {2.4, -1.2}};

  const DwaResult a = p.plan(0.3, 0.0, Pose2D(2.0, 0.0, 0.0), {}, near_obs, kDt);
  const DwaResult b = p.plan(0.3, 0.0, Pose2D(4.0, 0.0, 0.0), {}, far_obs, kDt);
  ASSERT_TRUE(a.success);
  ASSERT_TRUE(b.success);
  // Both are symmetric situations with the goal straight ahead, so both should
  // choose to go roughly straight.
  EXPECT_NEAR(a.omega, 0.0, 0.35);
  EXPECT_NEAR(b.omega, 0.0, 0.35);
}

TEST(Dwa, ReportsWhatItEvaluated)
{
  DynamicWindowPlanner p(config());
  const DwaResult r = p.plan(0.2, 0.0, Pose2D(3.0, 0.0, 0.0), {}, {}, kDt);
  ASSERT_TRUE(r.success);
  EXPECT_EQ(r.candidates_evaluated,
            p.config().linear_samples * p.config().angular_samples);
  EXPECT_GT(r.candidates_admissible, 0);
  EXPECT_LE(r.candidates_admissible, r.candidates_evaluated);
  EXPECT_GT(r.planning_time_ms, 0.0);
}

TEST(Dwa, RejectsAnInvalidControlPeriod)
{
  DynamicWindowPlanner p(config());
  EXPECT_FALSE(p.plan(0.0, 0.0, Pose2D(1, 0, 0), {}, {}, 0.0).success);
  EXPECT_FALSE(p.plan(0.0, 0.0, Pose2D(1, 0, 0), {}, {}, -0.1).success);
  EXPECT_FALSE(p.plan(0.0, 0.0, Pose2D(1, 0, 0), {}, {}, std::nan("")).success);
}

// =============================================================================
// The defects a live run exposed. DWA shipped the same carrot bug as the PID
// follower, independently, which is why the geometry is now shared in arb_core
// and tested there; what is tested here is DWA's use of it.
// =============================================================================

namespace
{
/// Densify a polyline of corners, as the global planner's output is.
std::vector<Pose2D> denseLine(const std::vector<Pose2D> & corners, double step)
{
  std::vector<Pose2D> out{corners.front()};
  for (std::size_t i = 0; i + 1 < corners.size(); ++i) {
    const double dx = corners[i + 1].x - corners[i].x;
    const double dy = corners[i + 1].y - corners[i].y;
    const int n = std::max(1, static_cast<int>(std::round(std::hypot(dx, dy) / step)));
    for (int k = 1; k <= n; ++k) {
      const double a = static_cast<double>(k) / n;
      out.emplace_back(corners[i].x + a * dx, corners[i].y + a * dy, 0.0);
    }
  }
  return out;
}

/// The same path seen from (x, y, th).
std::vector<Pose2D> inRobotFrame(const std::vector<Pose2D> & world,
                                 double x, double y, double th)
{
  std::vector<Pose2D> local;
  local.reserve(world.size());
  const double c = std::cos(-th);
  const double s = std::sin(-th);
  for (const auto & p : world) {
    local.emplace_back(c * (p.x - x) - s * (p.y - y), s * (p.x - x) + c * (p.y - y), 0.0);
  }
  return local;
}
}  // namespace

TEST(Dwa, MakesProgressAlongAPathFixedInTheWorld)
{
  // THE BUG: the carrot was walked from path.front(), which is the robot only
  // while the global planner keeps republishing from the robot's position. Held
  // still, the carrot stayed 1.0 m from the course start, the robot drove to it
  // and stopped: 1.99 m covered in 140 s of a 15.60 m course.
  DynamicWindowPlanner planner(config());
  const auto world = denseLine({Pose2D(0.0, 0.0, 0.0), Pose2D(8.0, 0.0, 0.0)}, 0.05);

  double x = 0.0, y = 0.0, th = 0.0, v = 0.0, w = 0.0;
  const double dt = 0.05;
  for (int i = 0; i < 400; ++i) {                     // 20 s at 20 Hz
    const auto local = inRobotFrame(world, x, y, th);
    const auto r = planner.plan(v, w, local.back(), local, {}, dt);
    ASSERT_TRUE(r.success) << r.failure_reason << " at step " << i;
    v = r.v;
    w = r.omega;
    const double half = 0.5 * w * dt;
    const double k = (std::abs(half) < 1e-9) ? 1.0 : std::sin(half) / half;
    x += v * dt * k * std::cos(th + half);
    y += v * dt * k * std::sin(th + half);
    th = arb_core::normalizeAngle(th + w * dt);
  }
  EXPECT_GT(x, 6.0) << "covered only " << x << " m of an 8 m path in 20 s";
  EXPECT_LT(std::abs(y), 0.15) << "wandered " << y << " m off a straight path";
}

TEST(Dwa, AimsAtTheCarrotRatherThanADistantGoal)
{
  // On a path that turns, the bearing to a goal far ahead points through
  // whatever the global planner routed around. The heading term must follow the
  // path's shape, which means aiming at the carrot.
  DwaConfig cfg = config();
  cfg.path_lookahead_m = 1.0;
  DynamicWindowPlanner planner(cfg);

  // Straight ahead 1.5 m, then a hard left. The goal ends up above and ahead;
  // the carrot at 1.0 m is still straight ahead.
  const auto path = denseLine(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(1.5, 0.0, 0.0), Pose2D(1.5, 4.0, 0.0)}, 0.05);
  const auto r = planner.plan(0.3, 0.0, path.back(), path, {}, 0.05);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_NEAR(r.carrot_distance_m, 1.0, 0.05);
  // Aiming at the goal at (1.5, 4.0) would demand a hard left NOW. Aiming at
  // the carrot 1.0 m straight ahead demands almost nothing.
  EXPECT_LT(std::abs(r.omega), 0.6)
    << "turned at " << r.omega << " rad/s while the path ahead is straight";
}

TEST(Dwa, SlowsDownForTheEndOfThePath)
{
  // Nothing in the objective knows the path ends: the velocity term rewards
  // speed all the way in. The admissible-velocity condition already used for
  // obstacles, v <= sqrt(2*d*a), is applied to the goal as well.
  DwaConfig cfg = config();
  DynamicWindowPlanner planner(cfg);

  // 0.20 m of path left: sqrt(2 * 0.20 * 1.0) = 0.63 m/s is the most the robot
  // may be doing and still stop on the goal.
  const auto nearly = denseLine({Pose2D(0.0, 0.0, 0.0), Pose2D(0.20, 0.0, 0.0)}, 0.02);
  const double dt = 0.05;
  const auto close = planner.plan(0.7, 0.0, nearly.back(), nearly, {}, dt);
  ASSERT_TRUE(close.success) << close.failure_reason;
  EXPECT_NEAR(close.path_remaining_m, 0.20, 0.01);
  EXPECT_NEAR(close.approach_speed_cap,
              std::sqrt(2.0 * 0.20 * cfg.max_linear_acceleration), 1e-6);
  // The invariant is "never faster than the cap UNLESS that is physically out of
  // reach this step". At 0.70 m/s with a 1.0 m/s^2 limit the slowest command
  // available is 0.65, above the 0.632 cap, and the honest answer is to brake as
  // hard as possible rather than to pretend the window contains 0.632.
  const double hardest_braking = 0.7 - cfg.max_linear_acceleration * dt;
  EXPECT_LE(close.v, std::max(close.approach_speed_cap, hardest_braking) + 1e-9)
    << "commanded " << close.v << " with only " << close.path_remaining_m
    << " m of path left";
  EXPECT_NEAR(close.v, hardest_braking, 1e-9)
    << "should be braking at the limit, not holding speed";

  // At a speed the cap can actually bind, it binds.
  const auto binding = planner.plan(0.5, 0.0, nearly.back(), nearly, {}, dt);
  ASSERT_TRUE(binding.success) << binding.failure_reason;
  EXPECT_LE(binding.v, binding.approach_speed_cap + 1e-9)
    << "commanded " << binding.v << " against a cap of "
    << binding.approach_speed_cap;

  // The same robot with eight metres of path ahead is not slowed at all.
  const auto far = denseLine({Pose2D(0.0, 0.0, 0.0), Pose2D(8.0, 0.0, 0.0)}, 0.05);
  const auto open = planner.plan(0.7, 0.0, far.back(), far, {}, 0.05);
  ASSERT_TRUE(open.success) << open.failure_reason;
  EXPECT_GT(open.v, 0.6) << "slowed to " << open.v << " with 8 m of clear path";
}

TEST(Dwa, RemainingDistanceIsAlongThePathNotTheStraightLine)
{
  // A path that comes back near its start has a short straight line and a long
  // way to drive. Using the straight line, the approach cap would throttle the
  // robot to a crawl for the whole lap.
  DynamicWindowPlanner planner(config());
  const auto loop = denseLine(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), Pose2D(3.0, 0.3, 0.0),
      Pose2D(0.3, 0.3, 0.0)}, 0.05);
  const auto r = planner.plan(0.5, 0.0, loop.back(), loop, {}, 0.05);
  ASSERT_TRUE(r.success) << r.failure_reason;
  EXPECT_NEAR(r.path_remaining_m, 6.0, 0.05)
    << "measured the straight line (0.42 m) instead of the arc";
  EXPECT_GT(r.v, 0.4) << "throttled to " << r.v << " with 6 m of path left";
}

TEST(Dwa, CommitsToOneDirectionWhenItHasToTurnOnTheSpot)
{
  // A robot that has ended up closer to a wall than its own safety radius has
  // no admissible forward command, and turning on the spot is the only way out.
  // The direction of that turn must be LATCHED.
  //
  // THE BUG: the side was chosen fresh each cycle from the bearing to the goal.
  // The moment the robot turned past that bearing the sign flipped, and it
  // oscillated across the bearing indefinitely. Measured in a live run: 0.3 m of
  // progress in 50 s, omega cycling -0.019, -0.048, -0.106, +0.179 rad/s.
  DwaConfig cfg = config();
  DynamicWindowPlanner planner(cfg);

  // Obstacles all around at 0.2 m -- inside the 0.25 m robot radius, so nothing
  // forward is admissible whatever the robot does.
  std::vector<ObstaclePoint> boxed;
  for (int i = 0; i < 36; ++i) {
    const double a = i * 2.0 * arb_core::pi<double>() / 36.0;
    boxed.push_back({0.2 * std::cos(a), 0.2 * std::sin(a)});
  }
  const auto path = denseLine(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 2.0, 0.0)}, 0.05);   // goal to the LEFT

  double th = 0.0, omega = 0.0;
  const double dt = 0.05;
  int sign_changes = 0;
  double previous = 0.0;
  for (int i = 0; i < 100; ++i) {
    // The path is re-expressed in the robot frame as it turns, exactly as the
    // node does -- so the bearing to the goal sweeps through zero and back.
    std::vector<Pose2D> local;
    const double c = std::cos(-th), s = std::sin(-th);
    for (const auto & q : path) {
      local.emplace_back(c * q.x - s * q.y, s * q.x + c * q.y, 0.0);
    }
    const auto r = planner.plan(0.0, omega, local.back(), local, boxed, dt);
    ASSERT_TRUE(r.success) << r.failure_reason << " at step " << i;
    EXPECT_TRUE(r.rotating_in_place) << "should have reported the fallback";
    if (i > 0 && r.omega * previous < 0.0) {++sign_changes;}
    previous = r.omega;
    omega = r.omega;
    th = arb_core::normalizeAngle(th + omega * dt);
  }
  EXPECT_EQ(sign_changes, 0)
    << "reversed direction " << sign_changes << " times while boxed in";
  EXPECT_GT(std::abs(omega), 1.0)
    << "never spun up: ended at " << omega << " rad/s";
}

TEST(Dwa, ReleasesTheCommittedDirectionOnceItCanDriveAgain)
{
  // The latch must not outlive the situation that created it, or the next
  // escape inherits a direction chosen for a different obstacle.
  DynamicWindowPlanner planner(config());
  std::vector<ObstaclePoint> boxed;
  for (int i = 0; i < 36; ++i) {
    const double a = i * 2.0 * arb_core::pi<double>() / 36.0;
    boxed.push_back({0.2 * std::cos(a), 0.2 * std::sin(a)});
  }
  const auto left = denseLine({Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 2.0, 0.0)}, 0.05);
  const auto blocked = planner.plan(0.0, 0.0, left.back(), left, boxed, 0.05);
  ASSERT_TRUE(blocked.rotating_in_place);
  EXPECT_GT(blocked.omega, 0.0) << "goal is to the left, so turn left";

  // Clear the obstacles: a forward command exists again.
  const auto clear = planner.plan(0.0, 0.0, left.back(), left, {}, 0.05);
  ASSERT_TRUE(clear.success);
  EXPECT_FALSE(clear.rotating_in_place);

  // Boxed in again with the goal now to the RIGHT: the direction must be
  // chosen afresh, not inherited.
  const auto right = denseLine({Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, -2.0, 0.0)}, 0.05);
  const auto again = planner.plan(0.0, 0.0, right.back(), right, boxed, 0.05);
  ASSERT_TRUE(again.rotating_in_place);
  EXPECT_LT(again.omega, 0.0) << "goal is to the right, so turn right";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
