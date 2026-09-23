// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_CONTROL__PATH_FOLLOWER_HPP_
#define ARB_CONTROL__PATH_FOLLOWER_HPP_

#include <vector>

#include "arb_core/path_geometry.hpp"
#include "arb_core/pose2d.hpp"

namespace arb_control
{

/// A PID controller for one scalar channel.
///
/// ========================= WHAT EACH TERM DOES =========================
///
///   P  reacts to the error NOW. Alone it always leaves a steady-state error
///      whenever a constant effort is needed to hold zero error, because zero
///      error commands zero effort.
///
///   I  accumulates past error, so it can hold a non-zero output at zero
///      error. That is what removes steady-state error -- and it is also the
///      term that destabilises the loop, because the accumulated history keeps
///      pushing after the error has changed sign.
///
///   D  reacts to the RATE of change, which damps the overshoot the other two
///      cause. It is also the term that amplifies measurement noise, since
///      differentiating noise multiplies it by the sample rate.
///
/// ANTI-WINDUP is not optional. If the output saturates -- and it always does,
/// because actuators have limits -- the integrator keeps accumulating an error
/// the robot cannot act on. By the time the error reverses, the integrator
/// holds a large value that must be "unwound" before the output leaves
/// saturation, and the response is a long, ugly overshoot that looks like a
/// tuning problem and is not.
///
/// Two defences here: the integral is clamped, and it stops accumulating
/// entirely while the output is saturated in the direction the error would
/// push it further.
class Pid
{
public:
  struct Gains
  {
    double kp{1.0};
    double ki{0.0};
    double kd{0.0};
    double integral_limit{1.0};   ///< clamp on the integral TERM, not the sum
    double output_limit{1.0};
    /// Low-pass on the derivative, as a time constant in seconds. Zero
    /// disables it. Differentiating a noisy signal multiplies the noise by the
    /// sample rate, so a raw D term on a 20 Hz loop with encoder-quantised
    /// feedback is mostly amplified quantisation.
    double derivative_filter_tau{0.05};

    /// The error is an ANGLE in radians, wrapped to (-pi, pi].
    ///
    /// This changes one thing: the derivative is taken over the WRAPPED
    /// difference. A heading error that slides from -3.12 to +3.10 has changed
    /// by 0.06 rad, not by 6.22, and differentiating the raw difference reports
    /// +124 rad/s^2 where the truth is +1.2.
    ///
    /// That single spike is enough to saturate the output, and because a robot
    /// turning one way always wraps the same way, the spike always has the same
    /// sign. It reinforces the rotation that produced it. Measured: the robot
    /// turned at its maximum rate continuously while the heading error cycled
    /// through every value, and no error signal could stop it.
    bool error_is_angle{false};
  };

  Pid() = default;
  explicit Pid(const Gains & gains) : gains_(gains) {}

  /// One control step. Returns the saturated output.
  double update(double error, double dt);

  void reset();

  double integral() const {return integral_;}
  double lastDerivative() const {return derivative_;}
  bool saturated() const {return saturated_;}

  const Gains & gains() const {return gains_;}
  void setGains(const Gains & g) {gains_ = g;}

private:
  Gains gains_;
  double integral_{0.0};
  double previous_error_{0.0};
  double derivative_{0.0};
  bool has_previous_{false};
  bool saturated_{false};
};

struct FollowerConfig
{
  Pid::Gains linear;
  Pid::Gains angular;

  /// Lookahead distance for the carrot, as in pure pursuit.
  ///
  /// The single most consequential parameter. Too short and the robot chases a
  /// point it keeps overshooting, so it weaves; too long and it cuts every
  /// corner. A common refinement, used here, is to scale it with speed: the
  /// faster the robot goes the further ahead it must look, because it covers
  /// more ground before it can react.
  double lookahead_min_m{0.35};
  double lookahead_gain_s{0.9};       ///< added lookahead per m/s of speed
  double lookahead_max_m{1.2};

  double max_linear_velocity{0.70};
  double min_linear_velocity{0.0};
  double max_angular_velocity{2.00};
  double max_linear_acceleration{1.00};
  double max_angular_acceleration{3.00};

  /// Slow down in proportion to how far off-heading the robot is. Driving at
  /// full speed while turning hard is how a differential drive leaves its
  /// path: the cross-track error grows faster than the controller can correct.
  double heading_slowdown_gain{1.4};

  /// Stop translating and turn on the spot when the heading error exceeds
  /// this. A differential drive can do that, and it is far better than
  /// arcing slowly back onto a path that is behind it.
  double in_place_turn_threshold_rad{1.0};

  double goal_tolerance_m{0.20};
};

struct FollowerCommand
{
  double v{0.0};
  double omega{0.0};
  bool at_goal{false};
  double cross_track_error_m{0.0};   ///< signed, positive to the robot's left
  double heading_error_rad{0.0};
  double lookahead_m{0.0};
  double progress_m{0.0};            ///< arc length from the path's start
  double distance_to_goal_m{0.0};    ///< arc length REMAINING, not straight line
  arb_core::Pose2D carrot;
};

/// A pure-pursuit path follower with PID on the two error channels.
///
/// The geometric part (pure pursuit) picks WHERE to aim; the PID part decides
/// HOW HARD to steer and how fast to go. Pure pursuit alone has no notion of
/// accumulated error, so a systematic bias -- a miscalibrated wheel, a slope,
/// a dragging caster -- leaves the robot permanently offset from its path.
class PathFollower
{
public:
  explicit PathFollower(const FollowerConfig & config = FollowerConfig());

  /// \param path  the path in the ROBOT frame
  /// \param current_v  the robot's measured speed. Used for the speed-scaled
  ///   lookahead and as the linear PID's feedback -- NOT for the rate limiter,
  ///   see rateLimit().
  ///
  /// There is deliberately no `current_omega`. The measured turn rate has no
  /// job here: the angular channel's feedback is the heading error, and putting
  /// the measurement into the rate limiter is what produced a controller that
  /// turned at its limit for 117 s straight.
  FollowerCommand follow(const std::vector<arb_core::Pose2D> & path,
                         double current_v, double dt);

  /// Clear the PID state. Called when there is no path to follow at all.
  void reset();

  /// Forget where the robot was on the path, WITHOUT clearing the PID state.
  ///
  /// Called when a new path arrives. The progress index is meaningless against
  /// a different path, but the integrators are not: a replan every second
  /// would otherwise dump the accumulated bias correction once a second, and
  /// the integral term would never do anything.
  void onNewPath() {progress_ = 0; have_progress_ = false;}

  const FollowerConfig & config() const {return config_;}
  FollowerConfig & mutableConfig() {return config_;}

  /// Signed distance from the robot to the path segment nearest it. Positive
  /// means the path is to the robot's left. A thin wrapper over
  /// arb_core::projectOntoPath, kept because it reads better at call sites.
  static double crossTrackError(const std::vector<arb_core::Pose2D> & path);

private:
  /// Rate-limit then absolute-limit a command, in that order.
  ///
  /// The rate limit is measured from the PREVIOUS COMMAND, not from the
  /// measured velocity. Anchoring it to the measurement feeds the plant's own
  /// overshoot back into the limiter: Gazebo's wheel controller settles about
  /// 11 % above a commanded 2.0 rad/s, and a measurement of 2.22 makes the
  /// admissible band [2.07, 2.37], which lies entirely ABOVE the 2.0 limit. The
  /// command then clamps to exactly 2.0 whatever the controller asks for -- and
  /// commanding 2.0 keeps the measurement at 2.22, so it never lets go.
  /// Measured: 2.0 rad/s commanded for 117 consecutive seconds.
  ///
  /// The previous command is known exactly, is always inside the limits, and
  /// gives a command sequence whose slew rate is what it claims to be.
  double rateLimit(double raw, double previous, double max_rate,
                   double lower, double upper) const;

  /// How far back along the path the projection may look. One robot length: far
  /// enough to recover from a slip, short enough that a path doubling back
  /// 0.6 m away cannot capture it.
  static constexpr double kBackwardWindowM = 0.40;

  FollowerConfig config_;
  Pid linear_pid_;
  Pid angular_pid_;

  std::size_t progress_{0};
  bool have_progress_{false};
  double last_v_command_{0.0};
  double last_omega_command_{0.0};
};

}  // namespace arb_control

#endif  // ARB_CONTROL__PATH_FOLLOWER_HPP_
