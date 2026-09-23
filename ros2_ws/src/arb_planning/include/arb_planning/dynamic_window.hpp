// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_PLANNING__DYNAMIC_WINDOW_HPP_
#define ARB_PLANNING__DYNAMIC_WINDOW_HPP_

#include <string>
#include <vector>

#include "arb_core/pose2d.hpp"

namespace arb_planning
{

/// A point obstacle in the ROBOT frame.
struct ObstaclePoint
{
  double x{0.0};
  double y{0.0};
};

struct DwaConfig
{
  // --- actuator limits, from configs/robot.yaml -------------------------
  double max_linear_velocity{0.70};
  double min_linear_velocity{-0.30};
  double max_angular_velocity{2.00};
  double max_linear_acceleration{1.00};
  double max_angular_acceleration{3.00};

  /// Resolution of the velocity sampling grid.
  int linear_samples{9};
  int angular_samples{21};

  /// How far ahead each candidate is rolled out, and with what step.
  /// The horizon is the single most consequential parameter: too short and the
  /// robot cannot see a wall in time to stop, too long and it refuses
  /// perfectly safe motion because something might be in the way eventually.
  double simulation_time{2.0};
  double simulation_dt{0.1};

  // --- objective weights ------------------------------------------------
  //
  // The DWA objective is a weighted sum of three terms that PULL IN DIFFERENT
  // DIRECTIONS, which is the whole design:
  //
  //   heading   -- go towards the goal
  //   clearance -- stay away from obstacles
  //   velocity  -- do not dawdle
  //
  // Heading alone drives into walls. Clearance alone parks in the middle of
  // the largest open space and never leaves. Velocity alone drives flat out
  // into the first thing it meets. The weights are how much each is allowed to
  // win, and tuning them is tuning the robot's personality.
  double heading_weight{1.0};
  double clearance_weight{0.7};
  double velocity_weight{0.25};
  double path_weight{1.2};

  /// Arc length along the global path at which the "carrot" sits.
  ///
  /// The path term scores how close the rollout ENDS to a point a fixed
  /// distance ALONG the path, not to the path as a whole. That distinction is
  /// not cosmetic: the global path starts at the robot, so "minimise the
  /// distance to the nearest path point" is maximised by NOT MOVING. Measured
  /// with the naive version, the best-scoring command was the slowest one the
  /// window allowed, every cycle.
  ///
  /// A carrot rewards adherence and progress in a single term, and is the same
  /// idea pure pursuit uses. Too short and the robot oscillates chasing a
  /// point it keeps overshooting; too long and it cuts corners towards a
  /// carrot on the far side of a wall.
  double path_lookahead_m{1.0};

  /// Radius of the robot's footprint, used for collision checking.
  double robot_radius{0.25};

  /// Clearance beyond this counts as "completely clear", so the term stops
  /// rewarding open space the robot has no reason to seek out.
  double max_useful_clearance{1.2};

  /// A trajectory already in collision at the robot's current position is
  /// rejected outright, whatever it scores elsewhere. A weighted sum can
  /// always be outvoted; a hard constraint cannot, and collision is not a
  /// preference.
  double collision_margin{0.02};

  /// The ADMISSIBLE VELOCITY condition, which is what DWA actually requires:
  ///
  ///     v <= sqrt( 2 * dist(v, omega) * a_max )
  ///
  /// where dist is the arc length along this curvature to the first collision.
  /// In words: the robot may choose any velocity it can still STOP from before
  /// hitting whatever is in the way.
  ///
  /// This is subtler than "the rollout must be collision-free", and rejecting
  /// every trajectory that touches an obstacle within the horizon is a
  /// mistake. The robot replans every control period; it does not have to
  /// commit to two seconds of constant velocity. Demanding a clear horizon
  /// makes a robot approaching a wall at 0.3 m/s with 0.45 m of clearance find
  /// NO admissible command and stop dead, when it could comfortably slow down
  /// and turn -- measured, and the reason this is a distance check rather than
  /// a collision check.
  bool enforce_stopping_distance{true};

  /// Below this the robot is treated as stopped, and in-place rotation is
  /// allowed even where forward motion is blocked.
  double in_place_rotation_speed{0.05};

  /// Arc-length step for the collision check, in metres.
  ///
  /// Stepping by TIME makes the safety check's resolution depend on speed: at
  /// 0.66 m/s and a 0.1 s step the robot jumps 6.6 cm between checks, so a
  /// wall can be missed by almost that much. Measured consequence: the check
  /// reported 0.265 m of room where there was 0.20 m, which made a command the
  /// robot could not stop from look admissible.
  ///
  /// Stepping by DISTANCE makes the resolution a fixed 2 cm regardless of
  /// speed, and the result is rounded DOWN to the last known-clear point, so
  /// the answer is conservative by construction.
  double collision_check_step_m{0.02};
};

struct DwaCandidate
{
  double v{0.0};
  double omega{0.0};
  bool admissible{false};
  std::string rejection;
  double heading_score{0.0};
  double clearance_score{0.0};
  double velocity_score{0.0};
  double path_score{0.0};
  double total{0.0};
  double min_clearance{0.0};
  double distance_to_collision{0.0};
  std::vector<arb_core::Pose2D> rollout;
};

struct DwaResult
{
  bool success{false};
  std::string failure_reason;
  double v{0.0};
  double omega{0.0};
  int candidates_evaluated{0};
  int candidates_admissible{0};
  DwaCandidate best;
  double planning_time_ms{0.0};

  /// Arc length remaining along the path, and the speed cap that implies.
  ///
  /// Reported because a robot that arrives too fast and a robot that is blocked
  /// both come out as "slow", and only the diagnostics distinguish them.
  double path_remaining_m{0.0};
  double approach_speed_cap{0.0};
  double carrot_distance_m{0.0};
};

/// The Dynamic Window Approach (Fox, Burgard and Thrun, 1997).
///
/// ======================== WHY VELOCITY SPACE ========================
///
/// A planner that searches POSITIONS can produce a path the robot cannot
/// follow: a right-angle turn, a reversal, an instant stop. The global planner
/// in Phase 7 does exactly that, and it is the right thing for it to do --
/// its job is to find a route, not to drive it.
///
/// DWA searches VELOCITIES instead. Every candidate is a (v, omega) pair the
/// robot can actually reach within one control period given its acceleration
/// limits, so every candidate is by construction executable. The "dynamic
/// window" is that reachable set:
///
///     v     in [v_now - a_max*dt,     v_now + a_max*dt]
///     omega in [w_now - alpha_max*dt, w_now + alpha_max*dt]
///
/// intersected with the actuator limits. Each candidate is rolled forward
/// under the constant-twist model for a fixed horizon and scored.
///
/// ======================== WHY NOT A POTENTIAL FIELD ========================
///
/// Potential fields are simpler and have a failure mode DWA does not: local
/// minima. A robot in a U-shaped dead end sits at the bottom of the potential
/// well forever, because every direction increases the potential. DWA can also
/// get stuck, but it is stuck in a way the score function reports -- every
/// candidate scores badly and the caller can see it and replan globally.
/// Silent failure versus visible failure.
class DynamicWindowPlanner
{
public:
  explicit DynamicWindowPlanner(const DwaConfig & config = DwaConfig());

  /// Choose a command.
  ///
  /// \param current_v,current_omega  the robot's twist now, which sets the window
  /// \param goal   target in the ROBOT frame
  /// \param path   the global path in the ROBOT frame; may be empty
  /// \param obstacles  obstacle points in the ROBOT frame
  /// \param dt     control period
  DwaResult plan(double current_v, double current_omega,
                 const arb_core::Pose2D & goal,
                 const std::vector<arb_core::Pose2D> & path,
                 const std::vector<ObstaclePoint> & obstacles,
                 double dt) const;

  /// Roll a constant twist forward, in the robot frame.
  std::vector<arb_core::Pose2D> rollout(double v, double omega) const;

  /// Closest approach of a rollout to any obstacle. Infinite when clear.
  double clearance(const std::vector<arb_core::Pose2D> & poses,
                   const std::vector<ObstaclePoint> & obstacles) const;

  /// Arc length along the (v, omega) curvature before the footprint first
  /// touches an obstacle, or infinity if it never does.
  ///
  /// This, not the minimum clearance, is what the admissible-velocity
  /// condition needs: the question is "how far can I go before I must have
  /// stopped", and a trajectory that curves away and back is safe for exactly
  /// as long as it takes to reach the first contact.
  ///
  /// It rolls out INDEPENDENTLY of `simulation_time`, far enough to cover the
  /// stopping distance from `v`. Reusing the scoring rollout would make the
  /// safety check silently depend on the scoring horizon: with a 0.3 s
  /// horizon at 0.66 m/s the robot travels 0.199 m, so a wall 0.20 m away is
  /// invisible and the check passes a command the robot cannot stop from.
  double distanceToCollision(double v, double omega,
                             const std::vector<ObstaclePoint> & obstacles) const;

  const DwaConfig & config() const {return config_;}
  DwaConfig & mutableConfig() {return config_;}

private:
  DwaConfig config_;
};

}  // namespace arb_planning

#endif  // ARB_PLANNING__DYNAMIC_WINDOW_HPP_
