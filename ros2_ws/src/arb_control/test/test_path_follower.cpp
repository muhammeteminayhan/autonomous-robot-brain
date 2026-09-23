// Copyright (c) 2026 Emin Ayhan. MIT License.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arb_control/path_follower.hpp"
#include "arb_core/angles.hpp"

using arb_control::FollowerCommand;
using arb_control::FollowerConfig;
using arb_control::PathFollower;
using arb_control::Pid;
using arb_core::Pose2D;

namespace
{
constexpr double kDt = 0.05;

Pid::Gains gains(double kp, double ki, double kd, double limit = 10.0)
{
  Pid::Gains g;
  g.kp = kp;
  g.ki = ki;
  g.kd = kd;
  g.output_limit = limit;
  g.integral_limit = 100.0;
  g.derivative_filter_tau = 0.0;
  return g;
}

/// A straight path along +x in the robot frame.
std::vector<Pose2D> straight(double length = 4.0, double step = 0.05,
                             double lateral = 0.0)
{
  std::vector<Pose2D> p;
  for (double x = 0.0; x <= length; x += step) {p.emplace_back(x, lateral, 0.0);}
  return p;
}

/// Express a path given in WORLD coordinates in the robot's frame.
///
/// Every test that builds its path relative to the robot's current position is
/// testing a path that slides along with the robot, and a path like that hides
/// any bug in how far along the path the controller thinks it is. A real path
/// is fixed in the world and the robot moves through it, which is what this
/// does.
std::vector<Pose2D> toRobotFrame(const std::vector<Pose2D> & world,
                                 double x, double y, double th)
{
  std::vector<Pose2D> local;
  local.reserve(world.size());
  const double c = std::cos(-th);
  const double s = std::sin(-th);
  for (const auto & p : world) {
    const double dx = p.x - x;
    const double dy = p.y - y;
    local.emplace_back(c * dx - s * dy, s * dx + c * dy, 0.0);
  }
  return local;
}

/// Densify a polyline of corners to a fixed spacing, as a planner would.
std::vector<Pose2D> densify(const std::vector<Pose2D> & corners, double step)
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

/// One step of exact SE(2) integration, the same motion model as Phase 3.
void integrate(double & x, double & y, double & th, double v, double w, double dt)
{
  const double half = 0.5 * w * dt;
  const double k = (std::abs(half) < 1e-9) ? 1.0 : std::sin(half) / half;
  x += v * dt * k * std::cos(th + half);
  y += v * dt * k * std::sin(th + half);
  th = arb_core::normalizeAngle(th + w * dt);
}

/// Hold a path still in the robot frame and step the controller `n` times.
///
/// Every command is rate-limited from the PREVIOUS COMMAND, so one call to
/// follow() from a standing start can only ever return
/// max_linear_acceleration * dt. Any assertion about what the controller wants
/// has to let it get there.
FollowerCommand settle(PathFollower & f, const std::vector<Pose2D> & path,
                       double measured_v, int n = 60)
{
  FollowerCommand c;
  for (int i = 0; i < n; ++i) {c = f.follow(path, measured_v, kDt);}
  return c;
}
}  // namespace

// =====================================================================
//  PID
// =====================================================================

TEST(Pid, ProportionalIsExactlyKpTimesError)
{
  Pid pid(gains(2.0, 0.0, 0.0));
  EXPECT_NEAR(pid.update(0.5, kDt), 1.0, 1e-12);
  EXPECT_NEAR(pid.update(-0.25, kDt), -0.5, 1e-12);
}

TEST(Pid, IntegralAccumulatesAndRemovesSteadyStateError)
{
  // The defining property: P alone commands zero output at zero error, so it
  // cannot hold a constant effort. I can.
  Pid pid(gains(0.0, 1.0, 0.0));
  double out = 0.0;
  for (int i = 0; i < 20; ++i) {out = pid.update(0.1, kDt);}
  EXPECT_NEAR(out, 0.1 * 20 * kDt, 1e-9);
  EXPECT_GT(out, 0.0) << "a constant error must build a non-zero output";
}

TEST(Pid, DerivativeRespondsToTheRateNotTheValue)
{
  Pid pid(gains(0.0, 0.0, 1.0));
  pid.update(0.0, kDt);                    // prime
  const double out = pid.update(0.1, kDt); // error jumped by 0.1 in one step
  EXPECT_NEAR(out, 0.1 / kDt, 1e-9);
  // A constant error has zero rate, so D contributes nothing.
  EXPECT_NEAR(pid.update(0.1, kDt), 0.0, 1e-9);
}

TEST(Pid, DerivativeFilterSuppressesSingleSampleNoise)
{
  // Differentiating a noisy signal multiplies the noise by the sample rate. On
  // a 20 Hz loop with encoder-quantised feedback, an unfiltered D term is
  // mostly amplified quantisation.
  Pid::Gains g = gains(0.0, 0.0, 1.0);
  Pid raw(g);
  g.derivative_filter_tau = 0.1;
  Pid filtered(g);

  raw.update(0.0, kDt);
  filtered.update(0.0, kDt);
  const double spike_raw = raw.update(0.01, kDt);          // one noisy sample
  const double spike_filtered = filtered.update(0.01, kDt);
  EXPECT_LT(std::abs(spike_filtered), std::abs(spike_raw) * 0.5)
    << "raw " << spike_raw << ", filtered " << spike_filtered;
}

TEST(Pid, SaturatesAtTheOutputLimit)
{
  Pid pid(gains(100.0, 0.0, 0.0, 1.5));
  EXPECT_NEAR(pid.update(10.0, kDt), 1.5, 1e-12);
  EXPECT_NEAR(pid.update(-10.0, kDt), -1.5, 1e-12);
  EXPECT_TRUE(pid.saturated());
}

TEST(Pid, AntiWindupStopsTheIntegralGrowingWhileSaturated)
{
  // Classic windup: the integrator accumulates an error the actuator cannot
  // act on, and by the time the error reverses it holds a large value that
  // must be unwound before the output leaves saturation. The symptom is a long
  // overshoot that looks like a tuning problem and is not.
  Pid::Gains g = gains(1.0, 2.0, 0.0, 1.0);
  Pid pid(g);

  // Drive it hard into saturation for several seconds.
  for (int i = 0; i < 200; ++i) {pid.update(5.0, kDt);}
  const double wound = pid.integral();
  EXPECT_LT(wound, 5.0 * 200 * kDt)
    << "the integral grew as if the output were never saturated";

  // Now reverse the error. A wound-up controller stays saturated the wrong way
  // for many steps; a protected one comes off the limit quickly.
  int steps_to_recover = 0;
  while (steps_to_recover < 200 && pid.update(-5.0, kDt) > -g.output_limit + 1e-9) {
    ++steps_to_recover;
  }
  EXPECT_LT(steps_to_recover, 5)
    << "took " << steps_to_recover << " steps to respond to a reversed error";
}

TEST(Pid, IntegralLimitIsExpressedInOutputUnits)
{
  // Clamping the raw sum makes the limit mean different things for different
  // ki. Clamping the TERM makes it mean the same thing always.
  Pid::Gains g = gains(0.0, 4.0, 0.0, 100.0);
  g.integral_limit = 0.5;              // at most 0.5 of output from I
  Pid pid(g);
  for (int i = 0; i < 500; ++i) {pid.update(1.0, kDt);}
  EXPECT_NEAR(pid.update(1.0, kDt), 0.5, 1e-9);
}

TEST(Pid, ResetClearsEverything)
{
  Pid pid(gains(1.0, 1.0, 1.0));
  for (int i = 0; i < 50; ++i) {pid.update(1.0, kDt);}
  ASSERT_GT(pid.integral(), 0.0);
  pid.reset();
  EXPECT_NEAR(pid.integral(), 0.0, 1e-15);
  // After a reset the first call must not see a phantom derivative from the
  // old error.
  EXPECT_NEAR(pid.update(1.0, kDt), 1.0 + 1.0 * 1.0 * kDt, 1e-9);
}

TEST(Pid, RejectsNonPositiveOrNonFiniteInput)
{
  Pid pid(gains(1.0, 1.0, 1.0));
  EXPECT_NEAR(pid.update(1.0, 0.0), 0.0, 1e-15);
  EXPECT_NEAR(pid.update(1.0, -0.1), 0.0, 1e-15);
  EXPECT_NEAR(pid.update(std::nan(""), kDt), 0.0, 1e-15);
  EXPECT_NEAR(pid.integral(), 0.0, 1e-15) << "a rejected call must not integrate";
}

// =====================================================================
//  CROSS-TRACK ERROR
// =====================================================================

TEST(CrossTrack, IsZeroOnThePath)
{
  EXPECT_NEAR(PathFollower::crossTrackError(straight()), 0.0, 1e-12);
}

TEST(CrossTrack, IsSignedWithPositiveMeaningThePathIsToTheLeft)
{
  // The sign is what tells the controller which way to steer. Getting it
  // backwards produces a controller that drives away from its path, smoothly
  // and confidently.
  const auto left = straight(4.0, 0.05, 0.3);    // path 0.3 m to the left
  const auto right = straight(4.0, 0.05, -0.3);
  EXPECT_NEAR(PathFollower::crossTrackError(left), 0.3, 1e-9);
  EXPECT_NEAR(PathFollower::crossTrackError(right), -0.3, 1e-9);
}

TEST(CrossTrack, MeasuresToTheSegmentNotTheNearestVertex)
{
  // With coarse waypoints the nearest VERTEX can be much further than the
  // nearest point on the path, and a controller fed that error oscillates in
  // step with the waypoint spacing.
  std::vector<Pose2D> coarse = {{-1.0, 0.5, 0.0}, {1.0, 0.5, 0.0}};
  EXPECT_NEAR(PathFollower::crossTrackError(coarse), 0.5, 1e-9)
    << "the perpendicular distance is 0.5, the nearest vertex is 1.118 away";
}

TEST(CrossTrack, HandlesADegeneratePath)
{
  EXPECT_NEAR(PathFollower::crossTrackError({}), 0.0, 1e-15);
  EXPECT_NEAR(PathFollower::crossTrackError({{0.0, 0.4, 0.0}}), 0.4, 1e-12);
}

// =====================================================================
//  FOLLOWER
// =====================================================================

TEST(Follower, DrivesForwardOnAStraightPath)
{
  PathFollower f;
  // One step from a standing start can only return one acceleration step --
  // that is the rate limiter doing its job, not the controller being timid.
  const FollowerCommand first = f.follow(straight(), 0.0, kDt);
  EXPECT_NEAR(first.v, FollowerConfig().max_linear_acceleration * kDt, 1e-9);

  const FollowerCommand c = settle(f, straight(), 0.3);
  EXPECT_GT(c.v, 0.3) << "on a straight clear path it must accelerate";
  EXPECT_NEAR(c.omega, 0.0, 1e-6);
  EXPECT_FALSE(c.at_goal);
}

TEST(Follower, SteersTowardsAPathOffToTheSide)
{
  PathFollower f;
  const FollowerCommand left = settle(f, straight(4.0, 0.05, 0.5), 0.3, 10);
  PathFollower g;
  const FollowerCommand right = settle(g, straight(4.0, 0.05, -0.5), 0.3, 10);
  EXPECT_GT(left.omega, 0.0) << "a path to the left must steer left";
  EXPECT_LT(right.omega, 0.0);
  EXPECT_GT(left.cross_track_error_m, 0.0);
  EXPECT_LT(right.cross_track_error_m, 0.0);
}

TEST(Follower, TurnsInPlaceWhenThePathIsBehindIt)
{
  // Arcing onto a path that is behind the robot is slow and wide. A
  // differential drive can simply turn first.
  PathFollower f;
  std::vector<Pose2D> behind;
  for (double x = 0.0; x >= -3.0; x -= 0.05) {behind.emplace_back(x, 0.0, 0.0);}
  const FollowerCommand c = settle(f, behind, 0.0, 10);
  EXPECT_NEAR(c.v, 0.0, 1e-12) << "must not drive off while turning round";
  EXPECT_GT(std::abs(c.omega), 0.1);
}

TEST(Follower, SlowsDownWhenTheHeadingErrorIsLarge)
{
  // Driving at full speed while turning hard is how a differential drive
  // leaves its path: the cross-track error grows faster than the controller
  // can correct it.
  PathFollower f;
  auto turning = straight(4.0, 0.05);
  for (auto & p : turning) {p.y = 0.45 * p.x;}        // a 24-degree path

  const FollowerCommand bent = settle(f, turning, 0.5);
  PathFollower g;
  const FollowerCommand flat = settle(g, straight(), 0.5);
  EXPECT_LT(bent.v, flat.v)
    << "bent " << bent.v << " vs straight " << flat.v;
}

TEST(Follower, ReportsAtGoalAndCommandsAStop)
{
  FollowerConfig cfg;
  cfg.goal_tolerance_m = 0.20;
  PathFollower f(cfg);
  std::vector<Pose2D> nearly = {{0.05, 0.0, 0.0}, {0.10, 0.0, 0.0}};
  const FollowerCommand c = f.follow(nearly, 0.3, kDt);
  EXPECT_TRUE(c.at_goal);
  EXPECT_NEAR(c.v, 0.0, 1e-12);
  EXPECT_NEAR(c.omega, 0.0, 1e-12);
}

TEST(Follower, RespectsTheAccelerationLimits)
{
  // Commanding a step the actuator cannot follow means the robot does
  // something other than what the controller believes it did, and every error
  // downstream inherits that.
  //
  // The limit is on the COMMAND STREAM: consecutive commands, not the gap
  // between a command and the measured velocity. The robot's actual velocity is
  // the plant's business and may lag or overshoot; what this controller
  // guarantees is that it never asks for a step it has no right to ask for.
  FollowerConfig cfg;
  cfg.max_linear_acceleration = 1.0;
  cfg.max_angular_acceleration = 3.0;
  PathFollower f(cfg);

  // 0.15 m of lateral offset, not 0.6: past the in-place-turn threshold the
  // linear command is legitimately zero, and this test needs both channels
  // moving to say anything about either.
  const auto path = straight(4.0, 0.05, 0.15);
  double previous_v = 0.0;
  double previous_omega = 0.0;
  for (int i = 0; i < 100; ++i) {
    // The measured speed is fed in deliberately WRONG -- a plant that lags by
    // half and overshoots in turn -- to prove the command stream is smooth
    // regardless of what the measurement does.
    const double measured = (i % 2 == 0) ? 0.0 : 0.7;
    const FollowerCommand c = f.follow(path, measured, kDt);
    EXPECT_LE(std::abs(c.v - previous_v), cfg.max_linear_acceleration * kDt + 1e-9)
      << "v jumped from " << previous_v << " to " << c.v << " at step " << i;
    EXPECT_LE(std::abs(c.omega - previous_omega),
              cfg.max_angular_acceleration * kDt + 1e-9)
      << "omega jumped from " << previous_omega << " to " << c.omega
      << " at step " << i;
    previous_v = c.v;
    previous_omega = c.omega;
  }
  EXPECT_GT(previous_v, 0.1) << "never got moving at all";
}

TEST(Follower, LookaheadGrowsWithSpeed)
{
  // A fixed lookahead is either too short at speed, so the robot weaves, or
  // too long at rest, so it cuts corners.
  PathFollower f;
  const double slow = settle(f, straight(), 0.05, 5).lookahead_m;
  f.reset();
  const double fast = settle(f, straight(), 0.70, 5).lookahead_m;
  EXPECT_GT(fast, slow);
  EXPECT_LE(fast, f.config().lookahead_max_m + 1e-12);
  EXPECT_GE(slow, f.config().lookahead_min_m - 1e-12);
}

TEST(Follower, ConvergesOntoAnOffsetPathInClosedLoop)
{
  // The test that matters: run the controller against a simulated robot and
  // check it actually gets onto the path and stays there. Single-step
  // assertions can all pass while the loop oscillates or diverges.
  FollowerConfig cfg;
  PathFollower f(cfg);

  // The robot starts 0.5 m to the right of a straight path along y = 0.
  double x = 0.0, y = -0.5, th = 0.0, v = 0.0, w = 0.0;
  double peak_after_settling = 0.0;
  const int steps = 600;                 // 30 s at 20 Hz

  for (int i = 0; i < steps; ++i) {
    // The path in the ROBOT frame: a line along world y = 0 ahead of us.
    std::vector<Pose2D> local;
    const double c = std::cos(-th);
    const double s = std::sin(-th);
    for (double px = x - 1.0; px <= x + 5.0; px += 0.05) {
      const double dx = px - x;
      const double dy = 0.0 - y;
      local.emplace_back(c * dx - s * dy, s * dx + c * dy, 0.0);
    }
    const FollowerCommand cmd = f.follow(local, v, kDt);
    v = cmd.v;
    w = cmd.omega;

    const double half = 0.5 * w * kDt;
    const double chord = v * kDt * (std::abs(half) < 1e-9 ? 1.0
                                    : std::sin(half) / half);
    x += chord * std::cos(th + half);
    y += chord * std::sin(th + half);
    th = arb_core::normalizeAngle(th + w * kDt);

    if (i > steps / 2) {peak_after_settling = std::max(peak_after_settling,
                                                       std::abs(y));}
  }
  EXPECT_LT(peak_after_settling, 0.05)
    << "settled with " << peak_after_settling
    << " m of residual cross-track error";
  EXPECT_GT(v, 0.1) << "and it must still be moving, not stalled on the path";
}

// =============================================================================
// The defects that live runs exposed and the unit tests above did not.
//
// The projection and carrot geometry these depend on lives in
// arb_core/path_geometry.hpp and is tested in arb_core/test/test_path_geometry.cpp
// -- it is shared with the DWA local planner, which shipped the same carrot bug
// independently. What is tested HERE is the controller built on top of it.
// =============================================================================

TEST(Follower, CarrotAdvancesAlongAPathFixedInTheWorld)
{
  // THE BUG: the carrot was found by walking `lookahead` metres from path[0],
  // so on a path that does not move with the robot it stayed put. The robot
  // drove to it, passed it, and then aimed at a point behind itself forever.
  FollowerConfig cfg;
  PathFollower f(cfg);
  const std::vector<Pose2D> world = straight(6.0, 0.05, 0.0);

  double previous_carrot_x = -1.0;
  for (double x = 0.0; x <= 3.0; x += 0.5) {
    const FollowerCommand cmd = f.follow(toRobotFrame(world, x, 0.0, 0.0),
                                         0.4, kDt);
    // The carrot is in the ROBOT frame, so a carrot that keeps up with the
    // robot sits at a roughly constant positive x. A carrot that does not
    // advance marches backwards through zero and becomes negative.
    EXPECT_GT(cmd.carrot.x, 0.0)
      << "carrot fell behind the robot at x = " << x;
    EXPECT_NEAR(cmd.carrot.x, cmd.lookahead_m, 0.1)
      << "carrot should sit one lookahead ahead, at x = " << x;
    EXPECT_LT(std::abs(cmd.heading_error_rad), 0.05)
      << "aiming off-axis on a straight path at x = " << x;
    previous_carrot_x = cmd.carrot.x;
  }
  EXPECT_GT(previous_carrot_x, 0.0);
}

TEST(Pid, DerivativeOfAWrappedAngleUsesTheAngularDifference)
{
  // A heading error that slides from -3.12 rad to +3.10 rad has changed by
  // 0.06 rad. The raw difference calls it +6.22, and at 20 Hz that is a
  // derivative of +124 rad/s^2 -- enough on its own to saturate the output.
  Pid::Gains g = gains(0.0, 0.0, 1.0, 1e6);   // D only, and no output clamp
  g.error_is_angle = true;
  Pid angular(g);
  angular.update(-3.12, kDt);
  const double wrapped = angular.update(3.10, kDt);

  g.error_is_angle = false;
  Pid plain(g);
  plain.update(-3.12, kDt);
  const double raw = plain.update(3.10, kDt);

  // normalizeAngle(3.10 - (-3.12)) = 6.22 - 2*pi = -0.0632 rad.
  EXPECT_NEAR(std::abs(wrapped), 0.0632 / kDt, 0.05)
    << "wrapped derivative should see a 0.06 rad step";
  EXPECT_GT(std::abs(raw), 100.0) << "the unwrapped version must be the big one";
  EXPECT_LT(std::abs(wrapped), 0.05 * std::abs(raw));
}

TEST(Follower, TheAngularChannelIsDeclaredAsAnAngle)
{
  // The caller cannot be relied on to set error_is_angle: the angular channel
  // is an angle by construction, so the follower asserts it itself.
  FollowerConfig cfg;
  cfg.angular.error_is_angle = false;
  PathFollower f(cfg);
  EXPECT_TRUE(f.config().angular.error_is_angle);
}

TEST(Follower, CommandsStayInsideTheVelocityLimits)
{
  // The rate limiter must never move a command back outside the absolute limit.
  FollowerConfig cfg;
  PathFollower f(cfg);
  // A path hard to the left forces the angular output into saturation.
  const std::vector<Pose2D> world = densify(
    {Pose2D(0.0, 2.0, 0.0), Pose2D(4.0, 2.0, 0.0)}, 0.05);
  double x = 0.0, y = 0.0, th = 0.0, v = 0.0, w = 0.0;
  for (int i = 0; i < 200; ++i) {
    const FollowerCommand cmd = f.follow(toRobotFrame(world, x, y, th), v, kDt);
    EXPECT_LE(std::abs(cmd.omega), cfg.max_angular_velocity + 1e-9)
      << "omega " << cmd.omega << " at step " << i;
    EXPECT_LE(cmd.v, cfg.max_linear_velocity + 1e-9);
    EXPECT_GE(cmd.v, cfg.min_linear_velocity - 1e-9);
    EXPECT_LE(std::abs(cmd.omega - w), cfg.max_angular_acceleration * kDt + 1e-9)
      << "slewed " << (cmd.omega - w) << " in one step at " << i;
    v = cmd.v;
    w = cmd.omega;
    integrate(x, y, th, v, w, kDt);
  }
}

TEST(Follower, EscapesSaturationEvenWhenThePlantOvershootsTheLimit)
{
  // THE BUG: the rate limit was measured from the MEASURED velocity. Gazebo's
  // wheel controller settles about 11 % above a commanded 2.0 rad/s, and a
  // measurement of 2.22 makes the admissible band [2.07, 2.37] -- entirely
  // above the 2.0 limit. Every request then clamped to exactly 2.0, including
  // -2.0, and commanding 2.0 kept the measurement at 2.22. Measured: 2.0 rad/s
  // commanded for 117 consecutive seconds on a course the robot was supposed
  // to be tracking.
  //
  // The plant here deliberately overshoots by 11 %, exactly as the simulator
  // does, so the test reproduces the mechanism and not just the symptom.
  FollowerConfig cfg;
  PathFollower f(cfg);
  const std::vector<Pose2D> world = densify(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(8.0, 0.0, 0.0)}, 0.05);

  double x = 0.0, y = 0.0, th = 0.0, v = 0.3, w = 0.0;
  int saturated_steps = 0;
  for (int i = 0; i < 400; ++i) {                 // 20 s at 20 Hz
    const FollowerCommand cmd = f.follow(toRobotFrame(world, x, y, th), v, kDt);
    if (std::abs(cmd.omega) > 0.95 * cfg.max_angular_velocity) {++saturated_steps;}
    v = cmd.v;
    // The plant overshoots its command by 11 %, and the CONTROLLER MUST NOT
    // CARE: it is handed the overshot value as the measurement.
    w = 1.11 * cmd.omega;
    integrate(x, y, th, v, w, kDt);
  }
  EXPECT_LT(saturated_steps, 40)
    << "angular command sat at the limit for " << saturated_steps
    << " of 400 steps on a straight path";
  EXPECT_LT(std::abs(y), 0.10)
    << "ended " << y << " m off a straight path it started on";
}

TEST(Follower, RemainingDistanceIsMeasuredAlongThePathNotTheStraightLine)
{
  // Slowing down for the goal has to use the arc length remaining. On a path
  // that turns back -- an L around a wall, a lap -- the straight line to the
  // last pose is short while there are still metres to drive, and the robot
  // crawls the whole way round at a fraction of its speed.
  FollowerConfig cfg;
  PathFollower f(cfg);
  // Out 3 m, then straight back to 0.3 m from the start.
  const std::vector<Pose2D> world = densify(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 0.0, 0.0), Pose2D(3.0, 0.3, 0.0),
      Pose2D(0.3, 0.3, 0.0)}, 0.05);

  const FollowerCommand cmd = f.follow(toRobotFrame(world, 0.0, 0.0, 0.0),
                                       0.0, kDt);
  EXPECT_NEAR(cmd.distance_to_goal_m, 6.0, 0.05)
    << "measured the straight line (0.42 m) instead of the arc";
  EXPECT_GT(cmd.v, 0.0) << "and it must not treat the goal as already reached";
}

TEST(Follower, DrivesAStaticPathFromStartToFinish)
{
  // The end-to-end test, on a path FIXED IN THE WORLD. The existing
  // closed-loop test rebuilt its path around the robot every step, which meant
  // path[0] was always just behind the robot and the carrot bug could not
  // show. This is the same simulation with the path held still, and it is the
  // test that fails against the old carrot.
  FollowerConfig cfg;
  PathFollower f(cfg);
  const std::vector<Pose2D> world = densify(
    {Pose2D(0.0, 0.0, 0.0), Pose2D(4.0, 0.0, 0.0), Pose2D(4.0, 3.0, 0.0),
      Pose2D(0.0, 3.0, 0.0)}, 0.05);
  const double course = 11.0;

  double x = 0.0, y = -0.6, th = 0.0, v = 0.0, w = 0.0;
  double travelled = 0.0;
  bool at_goal = false;
  double worst_after_settling = 0.0;
  for (int i = 0; i < 1200 && !at_goal; ++i) {      // up to 60 s at 20 Hz
    const FollowerCommand cmd = f.follow(toRobotFrame(world, x, y, th), v, kDt);
    at_goal = cmd.at_goal;
    v = cmd.v;
    w = cmd.omega;
    const double x0 = x, y0 = y;
    integrate(x, y, th, v, w, kDt);
    travelled += std::hypot(x - x0, y - y0);
    if (travelled > 2.0 && !at_goal) {
      worst_after_settling =
        std::max(worst_after_settling, std::abs(cmd.cross_track_error_m));
    }
  }
  EXPECT_TRUE(at_goal) << "gave up after " << travelled << " m of a "
                       << course << " m course";
  EXPECT_LT(travelled, 1.4 * course)
    << "drove " << travelled << " m to cover " << course << " m";
  EXPECT_LT(worst_after_settling, 0.35)
    << "left the path by " << worst_after_settling << " m at a corner";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
