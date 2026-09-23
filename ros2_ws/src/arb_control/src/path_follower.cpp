// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_control/path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "arb_core/angles.hpp"
#include "arb_core/path_geometry.hpp"

namespace arb_control
{

double Pid::update(double error, double dt)
{
  if (!(dt > 0.0) || !std::isfinite(error)) {return 0.0;}

  // --- derivative, on a low-passed error ---------------------------------
  if (has_previous_) {
    const double delta = gains_.error_is_angle
      ? arb_core::normalizeAngle(error - previous_error_)
      : (error - previous_error_);
    const double raw = delta / dt;
    if (gains_.derivative_filter_tau > 0.0) {
      // First-order low pass. alpha = dt / (tau + dt) is the exact
      // discretisation, not an approximation, and it stays stable for any dt.
      const double alpha = dt / (gains_.derivative_filter_tau + dt);
      derivative_ += alpha * (raw - derivative_);
    } else {
      derivative_ = raw;
    }
  } else {
    derivative_ = 0.0;
    has_previous_ = true;
  }
  previous_error_ = error;

  // --- provisional output, before the integral is committed --------------
  const double proportional = gains_.kp * error;
  const double derivative_term = gains_.kd * derivative_;

  // CONDITIONAL INTEGRATION. The integral only accumulates when doing so would
  // not push an already-saturated output further into saturation. Integrating
  // regardless is classic windup: the term grows while the actuator cannot
  // respond, and then has to be unwound before the output comes off the limit,
  // which shows up as a long overshoot that no amount of gain tuning fixes.
  const double unsaturated_without_i = proportional + derivative_term;
  const double candidate_integral = integral_ + error * dt;
  const double candidate_output =
    unsaturated_without_i + gains_.ki * candidate_integral;

  const bool would_saturate_high = candidate_output > gains_.output_limit;
  const bool would_saturate_low = candidate_output < -gains_.output_limit;
  const bool pushing_further =
    (would_saturate_high && error > 0.0) || (would_saturate_low && error < 0.0);

  if (!pushing_further) {
    integral_ = candidate_integral;
    // A second, independent clamp on the integral TERM (not the raw sum), so
    // the limit means the same thing whatever ki is.
    if (gains_.ki > 1e-12) {
      const double limit = gains_.integral_limit / gains_.ki;
      integral_ = std::clamp(integral_, -limit, limit);
    }
  }

  const double output = proportional + gains_.ki * integral_ + derivative_term;
  saturated_ = std::abs(output) > gains_.output_limit;
  return std::clamp(output, -gains_.output_limit, gains_.output_limit);
}

void Pid::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  derivative_ = 0.0;
  has_previous_ = false;
  saturated_ = false;
}

PathFollower::PathFollower(const FollowerConfig & config)
: config_(config), linear_pid_(config.linear), angular_pid_(config.angular)
{
  // The angular channel tracks a heading error, which wraps. The class cannot
  // infer that from the numbers, so it is declared here rather than left to
  // whoever fills in the config.
  Pid::Gains angular = config.angular;
  angular.error_is_angle = true;
  angular_pid_.setGains(angular);
  config_.angular = angular;
}

void PathFollower::reset()
{
  linear_pid_.reset();
  angular_pid_.reset();
  last_v_command_ = 0.0;
  last_omega_command_ = 0.0;
}

double PathFollower::crossTrackError(const std::vector<arb_core::Pose2D> & path)
{
  return arb_core::projectOntoPath(path).signed_distance;
}

FollowerCommand PathFollower::follow(
  const std::vector<arb_core::Pose2D> & path, double current_v, double dt)
{
  FollowerCommand cmd;
  if (path.empty() || !(dt > 0.0)) {
    reset();
    return cmd;
  }

  const arb_core::Pose2D goal = path.back();
  const double goal_distance = std::hypot(goal.x, goal.y);
  if (goal_distance < config_.goal_tolerance_m) {
    cmd.at_goal = true;
    cmd.distance_to_goal_m = goal_distance;
    // Progress is reported even here. Returning zero instead makes the
    // diagnostic jump from "6.3 m along" to "0 m along" in the step the robot
    // arrives, which reads exactly like the controller losing the path.
    cmd.progress_m = arb_core::pathLength(path);
    reset();
    return cmd;
  }

  // --- the carrot ---------------------------------------------------------
  // Lookahead grows with speed: the faster the robot goes, the further ahead
  // it must aim, because it covers more ground before it can react. A fixed
  // lookahead is either too short at speed (weaving) or too long at rest
  // (cutting corners).
  const double lookahead = std::clamp(
    config_.lookahead_min_m + config_.lookahead_gain_s * std::abs(current_v),
    config_.lookahead_min_m, config_.lookahead_max_m);
  cmd.lookahead_m = lookahead;

  // WHERE THE ROBOT IS ON THE PATH comes first. Everything else -- the carrot,
  // the cross-track error, the progress index -- is measured from this point.
  //
  // The backward window exists so a robot that slips or is nudged backwards can
  // still re-acquire the path behind it, while a path that folds back on itself
  // cannot lure the controller into restarting a leg it has already driven.
  const std::size_t window_segments = static_cast<std::size_t>(
    std::max(1.0, kBackwardWindowM / std::max(1e-3, arb_core::pathSpacing(path))));
  const std::size_t from = (!have_progress_ || progress_ < window_segments)
    ? 0u : progress_ - window_segments;
  const arb_core::PathProjection proj = arb_core::projectOntoPath(path, from);
  progress_ = proj.segment;
  have_progress_ = true;

  cmd.cross_track_error_m = proj.signed_distance;
  cmd.progress_m = proj.arc;

  // The carrot is `lookahead` metres FURTHER ALONG THE PATH than the nearest
  // point -- not `lookahead` metres from the path's first vertex. Those are the
  // same thing only at the instant the robot starts, which is why the bug was
  // invisible for the first second of every run.
  const arb_core::Pose2D carrot = arb_core::carrotAt(path, proj.arc, lookahead);
  cmd.carrot = carrot;
  cmd.distance_to_goal_m =
    std::max(0.0, arb_core::pathLength(path) - proj.arc);

  // --- heading error ------------------------------------------------------
  // The robot is at the origin facing +x in its own frame, so the bearing to
  // the carrot IS the heading error. Wrapping is what keeps a carrot just
  // behind the robot from commanding a 359-degree turn.
  const double heading_error =
    arb_core::normalizeAngle(std::atan2(carrot.y, carrot.x));
  cmd.heading_error_rad = heading_error;

  // --- in-place turn ------------------------------------------------------
  // With the path far off to the side, arcing onto it is slow and wide. A
  // differential drive can simply turn first, and doing so keeps the
  // cross-track error from growing while it manoeuvres.
  if (std::abs(heading_error) > config_.in_place_turn_threshold_rad) {
    const double omega = angular_pid_.update(heading_error, dt);
    // The linear channel is not being used, so its error history is stale. Left
    // alone, the moment translation resumes the D term differentiates a jump
    // from an error recorded seconds ago across a single 50 ms step.
    linear_pid_.reset();
    cmd.v = 0.0;
    cmd.omega = rateLimit(omega, last_omega_command_,
                          config_.max_angular_acceleration * dt,
                          -config_.max_angular_velocity,
                          config_.max_angular_velocity);
    last_v_command_ = cmd.v;
    last_omega_command_ = cmd.omega;
    return cmd;
  }

  // --- steering -----------------------------------------------------------
  const double omega_raw = angular_pid_.update(heading_error, dt);

  // --- speed --------------------------------------------------------------
  // Two things reduce it: distance to the goal (so the robot arrives rather
  // than arrives fast), and heading error (driving flat out while turning hard
  // is how a differential drive leaves its path).
  // Slowing down uses the distance REMAINING ALONG THE PATH, not the straight
  // line to the last pose. On any path that curves back -- a lap, an L around a
  // wall -- the straight line shrinks while there are still metres to drive,
  // and the robot crawls the whole way round.
  const double approach =
    std::min(1.0, cmd.distance_to_goal_m / (2.0 * lookahead));
  const double heading_penalty =
    1.0 / (1.0 + config_.heading_slowdown_gain * std::abs(heading_error));
  const double target_v = config_.max_linear_velocity * approach * heading_penalty;

  // The linear PID tracks that target rather than commanding it directly, so a
  // systematic bias -- a dragging caster, a slope -- is integrated out.
  const double v_raw = current_v + linear_pid_.update(target_v - current_v, dt);

  // --- limits -------------------------------------------------------------
  cmd.v = rateLimit(v_raw, last_v_command_, config_.max_linear_acceleration * dt,
                    config_.min_linear_velocity, config_.max_linear_velocity);
  cmd.omega = rateLimit(omega_raw, last_omega_command_,
                        config_.max_angular_acceleration * dt,
                        -config_.max_angular_velocity,
                        config_.max_angular_velocity);
  last_v_command_ = cmd.v;
  last_omega_command_ = cmd.omega;
  return cmd;
}

double PathFollower::rateLimit(double raw, double previous, double max_rate,
                               double lower, double upper) const
{
  // WHAT MATTERS IS THE ANCHOR, not the order of the two clamps.
  //
  // Because `previous` is itself a command, it already satisfies
  // lower <= previous <= upper, so the rate band [previous - max_rate,
  // previous + max_rate] always overlaps [lower, upper] and the two clamps
  // commute. Swapping them is an equivalent mutation; the mutation harness in
  // the scratchpad confirms the test suite cannot tell the difference, and it
  // is right not to.
  //
  // The order DOES matter when the anchor is a measured velocity, which is what
  // this used to be: a measurement of 2.22 rad/s against a 2.0 limit makes the
  // band [2.07, 2.37], which lies entirely above the limit, and then clamping
  // an already-limited command into it returns 2.07 for every input including
  // -2.0. See the header for the full failure.
  const double rate_limited =
    std::clamp(raw, previous - max_rate, previous + max_rate);
  return std::clamp(rate_limited, lower, upper);
}


}  // namespace arb_control
