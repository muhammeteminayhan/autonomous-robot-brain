// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_planning/dynamic_window.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include "arb_core/angles.hpp"
#include "arb_core/path_geometry.hpp"

namespace arb_planning
{
namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

/// sin(u)/u, stable through zero. Same function the odometry and the EKF use:
/// the rollout integrates the identical constant-twist model, and it would be
/// a strange kind of planner that simulated a different robot from the one the
/// estimator believes in.
double sinc(double u)
{
  if (std::abs(u) < 1e-4) {
    const double u2 = u * u;
    return 1.0 - u2 / 6.0 + u2 * u2 / 120.0;
  }
  return std::sin(u) / u;
}

}  // namespace

DynamicWindowPlanner::DynamicWindowPlanner(const DwaConfig & config)
: config_(config) {}

std::vector<arb_core::Pose2D> DynamicWindowPlanner::rollout(
  double v, double omega) const
{
  std::vector<arb_core::Pose2D> poses;
  const int steps = std::max(1, static_cast<int>(
      std::round(config_.simulation_time / config_.simulation_dt)));
  poses.reserve(steps + 1);

  arb_core::Pose2D p;
  poses.push_back(p);
  const double dt = config_.simulation_dt;
  for (int i = 0; i < steps; ++i) {
    const double half = 0.5 * omega * dt;
    const double chord = v * dt * sinc(half);
    const double heading_mid = p.theta + half;
    p.x += chord * std::cos(heading_mid);
    p.y += chord * std::sin(heading_mid);
    p.theta = arb_core::normalizeAngle(p.theta + omega * dt);
    poses.push_back(p);
  }
  return poses;
}

double DynamicWindowPlanner::clearance(
  const std::vector<arb_core::Pose2D> & poses,
  const std::vector<ObstaclePoint> & obstacles) const
{
  if (obstacles.empty()) {return kInf;}
  double worst = kInf;
  // The rollout is generated at a FIXED TIME STEP, so pose i happens at
  // i * simulation_dt and the obstacles can be advanced to meet it.
  //
  // Making only distanceToCollision time-aware is not enough, and the reason
  // is worth stating. That function feeds the admissible-velocity condition,
  // which asks "can the robot STOP before this". With a crossing obstacle the
  // answer is usually yes -- the robot has plenty of room to stop, it simply
  // does not stop, it keeps going and arrives exactly when the obstacle does.
  // The hard constraint is satisfied and the collision happens anyway.
  //
  // What actually changes the decision is the SCORE: a candidate whose
  // rollout passes through where the obstacle will be has to look bad, so a
  // different candidate wins. Measured: with only the constraint made
  // time-aware, the chosen command was byte-identical to the static-world one.
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const double t = config_.predict_obstacle_motion
      ? static_cast<double>(i) * config_.simulation_dt : 0.0;
    for (const auto & o : obstacles) {
      const double d = std::hypot(o.xAt(t) - poses[i].x, o.yAt(t) - poses[i].y)
        - config_.robot_radius;
      worst = std::min(worst, d);
      // No early exit on a negative value: the MINIMUM over the whole
      // trajectory is what the score needs, and a trajectory that grazes an
      // obstacle at step 3 and clears it afterwards is still a collision.
    }
  }
  return worst;
}

double DynamicWindowPlanner::timeToCollision(
  double v, double omega, const std::vector<ObstaclePoint> & obstacles) const
{
  if (obstacles.empty()) {return kInf;}

  // Stepped in TIME, not arc length, because the question is a time one:
  // when does this curvature put the robot where that obstacle will be? The
  // robot may be stationary and still be hit, so there is no speed to divide
  // by and no arc length to step along.
  const double dt = config_.simulation_dt;
  const double horizon = config_.moving_obstacle_horizon_s;

  arb_core::Pose2D p;
  for (double t = 0.0; t <= horizon + 1e-9; t += dt) {
    for (const auto & o : obstacles) {
      if (std::hypot(o.xAt(t) - p.x, o.yAt(t) - p.y)
          <= config_.robot_radius + config_.collision_margin)
      {
        return t;
      }
    }
    const double half = 0.5 * omega * dt;
    const double chord = v * dt * sinc(half);
    const double heading_mid = p.theta + half;
    p.x += chord * std::cos(heading_mid);
    p.y += chord * std::sin(heading_mid);
    p.theta = arb_core::normalizeAngle(p.theta + omega * dt);
  }
  return kInf;
}

double DynamicWindowPlanner::distanceToCollision(
  double v, double omega, const std::vector<ObstaclePoint> & obstacles) const
{
  if (obstacles.empty()) {return kInf;}

  // How far we must look: far enough that a command we could not stop from is
  // always visible. Stopping from v needs v^2/(2a); add the footprint and a
  // margin, and never look less far than the scoring horizon.
  const double stopping =
    (v * v) / (2.0 * std::max(1e-6, config_.max_linear_acceleration));
  const double needed = stopping + config_.robot_radius + 0.2;
  const double horizon = std::max(
    needed, std::abs(v) * config_.simulation_time);

  // Step by ARC LENGTH, not time, so the resolution of the safety check does
  // not depend on how fast the robot happens to be going.
  if (std::abs(v) < 1e-9) {
    // A stationary or rotating-in-place command never advances, so the only
    // question is whether anything REACHES the robot while it sits there.
    // Standing still is not automatically safe when the obstacle is the thing
    // that moves -- which is exactly how Phase 11's collision happened: DWA
    // commanded a stop and the obstacle walked into the stationary robot.
    const double window = config_.predict_obstacle_motion
      ? config_.simulation_time : 0.0;
    for (const auto & o : obstacles) {
      for (double t = 0.0; t <= window + 1e-9; t += config_.simulation_dt) {
        if (std::hypot(o.xAt(t), o.yAt(t)) <= config_.robot_radius) {return 0.0;}
      }
      if (std::hypot(o.x, o.y) <= config_.robot_radius) {return 0.0;}
    }
    return kInf;
  }
  const double dt = config_.collision_check_step_m / std::abs(v);

  arb_core::Pose2D p;
  double travelled = 0.0;
  double last_clear = 0.0;
  double elapsed = 0.0;
  const int max_steps = 5000;
  for (int i = 0; i <= max_steps; ++i) {
    // Each step of the rollout happens at a known TIME, so the obstacle can be
    // advanced to meet it. Stepping by arc length keeps the spatial resolution
    // independent of speed; dividing by |v| recovers the time that step took.
    const double t_ahead = config_.predict_obstacle_motion ? elapsed : 0.0;
    for (const auto & o : obstacles) {
      if (std::hypot(o.xAt(t_ahead) - p.x, o.yAt(t_ahead) - p.y)
          <= config_.robot_radius)
      {
        // Round DOWN to the last point known to be clear. Reporting the
        // distance at which contact was DETECTED overstates the room available
        // by up to one step, and the admissible-velocity condition then passes
        // a command the robot cannot stop from.
        return last_clear;
      }
    }
    last_clear = travelled;
    if (travelled > horizon) {break;}

    elapsed += dt;
    const double half = 0.5 * omega * dt;
    const double chord = v * dt * sinc(half);
    const double heading_mid = p.theta + half;
    p.x += chord * std::cos(heading_mid);
    p.y += chord * std::sin(heading_mid);
    p.theta = arb_core::normalizeAngle(p.theta + omega * dt);
    travelled += std::abs(chord);
  }
  return kInf;
}

DwaResult DynamicWindowPlanner::plan(
  double current_v, double current_omega,
  const arb_core::Pose2D & goal,
  const std::vector<arb_core::Pose2D> & path,
  const std::vector<ObstaclePoint> & obstacles,
  double dt) const
{
  const auto t0 = std::chrono::steady_clock::now();
  DwaResult result;

  auto finish = [&](const std::string & reason) {
      result.failure_reason = reason;
      result.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      return result;
    };

  if (dt <= 0.0 || !std::isfinite(dt)) {return finish("invalid control period");}

  // --- the dynamic window ------------------------------------------------
  // What the robot can reach in one control period, intersected with what the
  // actuators allow at all. Candidates outside this are not slow or risky,
  // they are impossible.
  const double v_lo = std::max(config_.min_linear_velocity,
                               current_v - config_.max_linear_acceleration * dt);
  double v_hi = std::min(config_.max_linear_velocity,
                         current_v + config_.max_linear_acceleration * dt);
  const double w_lo = std::max(-config_.max_angular_velocity,
                               current_omega - config_.max_angular_acceleration * dt);
  const double w_hi = std::min(config_.max_angular_velocity,
                               current_omega + config_.max_angular_acceleration * dt);

  const double goal_distance = std::hypot(goal.x, goal.y);

  // Static and moving obstacles are judged by different rules, so they are
  // separated once rather than re-tested per candidate. An obstacle counts as
  // moving only when prediction is enabled; with it off, everything is static
  // and the behaviour is the Phase 11 behaviour exactly.
  std::vector<ObstaclePoint> static_obstacles, moving_obstacles;
  static_obstacles.reserve(obstacles.size());
  for (const auto & o : obstacles) {
    const bool moving = config_.predict_obstacle_motion &&
      std::hypot(o.vx, o.vy) > 1e-6;
    (moving ? moving_obstacles : static_obstacles).push_back(o);
  }

  // --- the carrot ---------------------------------------------------------
  // `path_lookahead_m` FURTHER ALONG the path than the robot's projection onto
  // it -- not `path_lookahead_m` from the path's first vertex.
  //
  // The first version walked from path.front(), with a comment claiming it
  // walked "from the robot". Those coincide only while something republishes
  // the path from the robot's current position, which the global planner does
  // several times a second -- so the defect hid behind the replanner and
  // appeared the moment a path was held still: 1.99 m covered in 140 s of a
  // 15.60 m course, the best-scoring command every cycle being the slowest the
  // window allowed. The identical bug shipped independently in the PID
  // follower, which is why the geometry now lives in arb_core and is shared.
  const arb_core::PathProjection projection = arb_core::projectOntoPath(path);
  arb_core::Pose2D carrot;
  bool carrot_valid = projection.valid;
  if (carrot_valid) {
    carrot = arb_core::carrotAt(path, projection.arc, config_.path_lookahead_m);
    result.carrot_distance_m = std::hypot(carrot.x, carrot.y);
  }

  // --- slowing down for the goal ------------------------------------------
  // The objective rewards speed and nothing in it knows the path ends. Without
  // this the robot runs at its limit until the node's arrival check fires and
  // then stops dead -- it reaches the goal by overshooting it and being told to
  // halt, which is not the same thing as arriving.
  //
  // The cap is the SAME admissible-velocity condition already applied to
  // obstacles, v <= sqrt(2 * d * a_max), with the goal as the thing that must
  // not be hit. Distance is the arc length REMAINING ALONG THE PATH, not the
  // straight line: on a path that turns back the straight line shrinks while
  // there are still metres to drive, and the robot would crawl the whole way.
  result.path_remaining_m = projection.valid
    ? std::max(0.0, arb_core::pathLength(path) - projection.arc)
    : goal_distance;
  result.approach_speed_cap = std::sqrt(
    2.0 * result.path_remaining_m * config_.max_linear_acceleration);
  // max() with v_lo keeps the window non-empty: if the robot is already going
  // faster than it can stop from, the answer is "decelerate as hard as you
  // can", not "no admissible command exists".
  v_hi = std::max(v_lo, std::min(v_hi, result.approach_speed_cap));

  // First pass: roll out and reject the inadmissible, keeping the raw terms so
  // the second pass can normalise them.
  std::vector<DwaCandidate> candidates;
  candidates.reserve(
    static_cast<std::size_t>(config_.linear_samples * config_.angular_samples));

  const int nv = std::max(1, config_.linear_samples);
  const int nw = std::max(1, config_.angular_samples);

  for (int i = 0; i < nv; ++i) {
    const double v = (nv == 1) ? v_hi
      : v_lo + (v_hi - v_lo) * static_cast<double>(i) / (nv - 1);
    for (int j = 0; j < nw; ++j) {
      const double w = (nw == 1) ? 0.0
        : w_lo + (w_hi - w_lo) * static_cast<double>(j) / (nw - 1);

      DwaCandidate c;
      c.v = v;
      c.omega = w;
      c.rollout = rollout(v, w);
      c.min_clearance = clearance(c.rollout, obstacles);
      // Static obstacles answer "can I stop before it"; moving obstacles
      // answer "will I be there when it is". Mixing them into one distance
      // loses the distinction, which is the whole point.
      c.distance_to_collision = distanceToCollision(v, w, static_obstacles);
      c.time_to_moving_collision = timeToCollision(v, w, moving_obstacles);
      ++result.candidates_evaluated;

      // Already in contact at the robot's CURRENT position: nothing this
      // command does can help, and it is the one case that is a hard
      // rejection rather than a distance question.
      if (c.distance_to_collision <= config_.collision_margin) {
        c.rejection = "already in collision";
        candidates.push_back(std::move(c));
        continue;
      }

      // THE ADMISSIBLE VELOCITY CONDITION
      //
      //     v <= sqrt( 2 * dist * a_max )
      //
      // The robot may choose any velocity it can still stop from before the
      // first obstacle on this curvature. It does NOT have to keep the whole
      // horizon clear: it replans every control period and can slow down on
      // the way. Requiring a clear horizon instead makes a robot with 0.45 m
      // of room at 0.3 m/s find no admissible command at all and stop dead.
      if (config_.enforce_stopping_distance &&
          std::isfinite(c.distance_to_collision))
      {
        const double max_safe = std::sqrt(
          2.0 * c.distance_to_collision * config_.max_linear_acceleration);
        if (v > max_safe) {
          c.rejection = "cannot stop in time";
          candidates.push_back(std::move(c));
          continue;
        }
      }

      // AND THE MOVING-OBSTACLE RULE, which is strict where the one above is
      // permissive. Being able to stop is no help against something coming
      // towards you: it arrives regardless. A command that puts the robot
      // where a moving obstacle will be, at any point inside the horizon, is
      // rejected outright.
      //
      // This is the rule that would be far too strict for static obstacles --
      // it is "keep the whole horizon clear", which strands a robot with
      // 0.45 m of room. It is right here because the obstacle will have moved
      // on: refusing to enter its path for two seconds costs two seconds,
      // while entering it costs a collision.
      if (std::isfinite(c.time_to_moving_collision)) {
        c.rejection = "a moving obstacle gets there first";
        candidates.push_back(std::move(c));
        continue;
      }

      c.admissible = true;
      ++result.candidates_admissible;

      // --- raw objective terms -------------------------------------------
      const arb_core::Pose2D & end = c.rollout.back();

      // Heading: how much of the angle to the AIM POINT remains at the end of
      // the rollout. Measured from the END pose, not the current one, which is
      // the point of simulating at all.
      //
      // The aim point is the carrot when there is a path, and the goal only when
      // there is not. Aiming at the final goal while a path exists throws the
      // path's shape away: on a course that turns, the bearing to a goal 15 m
      // ahead points through the wall the planner routed around, and the heading
      // term then fights the path term every cycle.
      const arb_core::Pose2D & aim = carrot_valid ? carrot : goal;
      const double bearing = std::atan2(aim.y - end.y, aim.x - end.x);
      const double residual =
        std::abs(arb_core::angleDiff(bearing, end.theta));
      c.heading_score = 1.0 - residual / arb_core::pi<double>();

      // Clearance scores the DISTANCE the robot could travel before trouble,
      // saturating: beyond max_useful_clearance there is no reason to seek
      // more open space, and without the cap this term quietly becomes "drive
      // into the middle of the largest room".
      const double clear = std::isfinite(c.distance_to_collision)
        ? std::min(c.distance_to_collision, config_.max_useful_clearance)
        : config_.max_useful_clearance;
      c.clearance_score = clear / config_.max_useful_clearance;

      // Velocity, signed towards the goal: reversing away from it is not
      // rewarded for being fast.
      c.velocity_score = std::max(0.0, v) / std::max(1e-6,
                                                     config_.max_linear_velocity);

      // Path adherence, measured against the CARROT rather than the nearest
      // path point.
      //
      // Scoring the nearest point rewards standing still: the robot is already
      // on the path, so not moving is optimal. Measured with that version, the
      // best command every cycle was the slowest the window allowed. The carrot
      // rewards adherence and progress together, which is the trick pure
      // pursuit uses.
      //
      // Heading alone is not enough either: it will happily cut a corner
      // through a room the global planner routed around for a reason.
      if (carrot_valid) {
        const double d = std::hypot(carrot.x - end.x, carrot.y - end.y);
        c.path_score = 1.0 / (1.0 + d);
      } else {
        c.path_score = 1.0;
      }
      candidates.push_back(std::move(c));
    }
  }

  if (result.candidates_admissible == 0) {
    // EVASION BEFORE SURRENDER.
    //
    // When a moving obstacle has disqualified every forward command, stopping
    // is not a safe default -- it is how the Phase 11 collision happened, and
    // it is how the first version of this rule reproduced it: the planner
    // rejected everything, commanded zero, and the obstacle walked into a
    // stationary robot. Measured, that version fixed none of the collisions
    // in a timing sweep and made two of them slightly worse.
    //
    // So before giving up, take the command that buys the most TIME. Reverse
    // is in the window and is often the answer. It is a best effort, not a
    // guarantee -- but "the latest possible contact" strictly dominates
    // "contact now while standing still".
    if (!moving_obstacles.empty()) {
      const DwaCandidate * best_effort = nullptr;
      for (const auto & c : candidates) {
        if (c.rejection != "a moving obstacle gets there first") {continue;}
        if (best_effort == nullptr ||
            c.time_to_moving_collision > best_effort->time_to_moving_collision ||
            (c.time_to_moving_collision == best_effort->time_to_moving_collision &&
             c.distance_to_collision > best_effort->distance_to_collision))
        {
          best_effort = &c;
        }
      }
      if (best_effort != nullptr) {
        result.success = true;
        result.v = best_effort->v;
        result.omega = best_effort->omega;
        result.best = *best_effort;
        result.best.admissible = true;
        result.evading = true;
        escape_direction_ = 0;
        return finish("evading: no command clears the moving obstacle");
      }
    }

    // Every forward option is blocked by something that is not going to move.
    // Rotating in place is still safe if the robot is nearly stopped, and it
    // is the only way out of a dead end.
    if (std::abs(current_v) < config_.in_place_rotation_speed) {
      // Turn towards the AIM POINT -- the carrot when there is a path, the goal
      // otherwise -- and COMMIT to that direction until a forward command
      // becomes admissible again.
      //
      // Choosing the side fresh each cycle is what makes a boxed-in robot
      // jitter rather than escape: the moment it has turned past the aim point
      // the bearing changes sign, the rotation reverses, and it oscillates
      // across the bearing forever. Measured, before the latch: 0.3 m of
      // progress in 50 s while omega cycled through -0.019, -0.048, -0.106 and
      // +0.179 rad/s.
      if (escape_direction_ == 0) {
        const arb_core::Pose2D & aim = carrot_valid ? carrot : goal;
        escape_direction_ = (aim.y >= 0.0) ? 1 : -1;
      }
      result.success = true;
      result.rotating_in_place = true;
      result.v = 0.0;
      result.omega = escape_direction_ *
        std::min(config_.max_angular_velocity, std::abs(w_hi));
      result.best.v = 0.0;
      result.best.omega = result.omega;
      result.best.admissible = true;
      return finish("rotating in place: no admissible forward trajectory");
    }
    result.v = 0.0;
    result.omega = 0.0;
    return finish("no admissible trajectory; stopping");
  }

  // --- normalise, then combine -------------------------------------------
  //
  // Normalising ACROSS CANDIDATES rather than using raw values is what makes
  // the weights mean anything. Raw clearance is in metres and raw heading in
  // radians; summing them with fixed weights makes the balance depend on the
  // size of the room. Scaling each term to [0, 1] over the current candidate
  // set makes the weights a statement about priorities instead.
  double h_min = kInf, h_max = -kInf;
  double c_min = kInf, c_max = -kInf;
  double v_min = kInf, v_max = -kInf;
  double p_min = kInf, p_max = -kInf;
  for (const auto & c : candidates) {
    if (!c.admissible) {continue;}
    h_min = std::min(h_min, c.heading_score); h_max = std::max(h_max, c.heading_score);
    c_min = std::min(c_min, c.clearance_score); c_max = std::max(c_max, c.clearance_score);
    v_min = std::min(v_min, c.velocity_score); v_max = std::max(v_max, c.velocity_score);
    p_min = std::min(p_min, c.path_score); p_max = std::max(p_max, c.path_score);
  }
  const auto scale = [](double x, double lo, double hi) {
      return (hi - lo > 1e-9) ? (x - lo) / (hi - lo) : 1.0;
    };

  double best_total = -kInf;
  for (auto & c : candidates) {
    if (!c.admissible) {continue;}
    c.total =
      config_.heading_weight * scale(c.heading_score, h_min, h_max) +
      config_.clearance_weight * scale(c.clearance_score, c_min, c_max) +
      config_.velocity_weight * scale(c.velocity_score, v_min, v_max) +
      config_.path_weight * scale(c.path_score, p_min, p_max);
    if (c.total > best_total) {
      best_total = c.total;
      result.best = c;
    }
  }

  // A forward command exists again, so the escape rotation is over and the
  // next one is free to pick its own direction.
  escape_direction_ = 0;
  result.success = true;
  result.v = result.best.v;
  result.omega = result.best.omega;
  return finish("");
}

}  // namespace arb_planning
