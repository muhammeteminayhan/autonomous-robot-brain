// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_LOCALIZATION__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
#define ARB_LOCALIZATION__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_

#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <string>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"

namespace arb_localization
{

/// Physical constants of the platform, read from configs/robot.yaml.
struct OdometryConfig
{
  /// Wheel radius r. Enters every distance the robot believes it travelled, so
  /// an error here is a pure SCALE error on the whole trajectory.
  double wheel_radius_m{0.06};

  /// Track width L, the distance between the wheel contact points. Enters only
  /// the heading, so an error here is a pure ROTATION-RATE error: the robot
  /// thinks it turned more or less than it did, and the position error that
  /// follows grows with distance rather than being proportional to it.
  double wheel_separation_m{0.35};

  /// Encoder resolution, used for the quantisation term of the covariance.
  double encoder_resolution_rad{2.0 * 3.14159265358979323846 / 4096.0};

  /// Noise coefficients for the motion model, in metres of variance per metre
  /// travelled. This is the standard "odometry error grows with distance"
  /// model: a wheel that slips does so in proportion to how far it turned, so
  /// the VARIANCE is linear in |arc length| and the standard deviation grows
  /// as sqrt(distance).
  double k_right{0.002};
  double k_left{0.002};

  /// Floor on the per-step variance, so that a stationary robot still
  /// accumulates a little uncertainty. Without it the covariance stops growing
  /// the moment the wheels stop, which claims the robot knows exactly where it
  /// is while it sits still being pushed, bumped or drifting.
  double min_variance_per_step{1e-9};

  /// Reject a step whose interval is non-positive or absurdly long.
  double max_dt_s{0.5};

  /// Largest believable wheel increment in one step. A jump beyond this is a
  /// dropped sample or a corrupted count, not motion; integrating it teleports
  /// the estimate and there is no recovering from that.
  double max_wheel_step_rad{2.0};

  std::string left_joint{"left_wheel_joint"};
  std::string right_joint{"right_wheel_joint"};
};

/// Why a step was not integrated.
enum class OdometryReject
{
  kNone,
  kFirstSample,
  kNonFinite,
  kNonMonotonicTime,
  kImplausibleStep,
};

/// One integration step, fully described for logging and testing.
struct OdometryStep
{
  bool valid{false};
  OdometryReject reason{OdometryReject::kNone};

  double dt_s{0.0};
  double left_arc_m{0.0};    ///< ground arc length of the left wheel
  double right_arc_m{0.0};
  double linear_velocity{0.0};   ///< v  [m/s]
  double angular_velocity{0.0};  ///< omega  [rad/s]
  double delta_s_m{0.0};         ///< signed distance travelled this step
  double delta_theta_rad{0.0};   ///< heading change this step

  /// Jacobian of the pose update with respect to the PREVIOUS pose.
  /// Exposed because it is the F matrix of the Phase 4 EKF prediction step,
  /// and because a Jacobian that is never compared against a finite difference
  /// of the function it claims to differentiate is an assertion, not a fact.
  Eigen::Matrix3d jacobian_pose{Eigen::Matrix3d::Identity()};

  /// Jacobian with respect to the two measured wheel arc lengths,
  /// ordered (right, left). This is what maps wheel-level noise into pose
  /// covariance.
  Eigen::Matrix<double, 3, 2> jacobian_input{Eigen::Matrix<double, 3, 2>::Zero()};
};

/// Dead reckoning for a differential-drive platform.
///
/// ============================ THE KINEMATICS ============================
///
/// Two wheels of radius r, a distance L apart, each turning at omega_l and
/// omega_r. Contact points roll without slipping, so each wheel's contact
/// point moves at v_l = r*omega_l and v_r = r*omega_r along the ground.
///
/// The body is rigid, so the two contact points share one angular velocity
/// about the instantaneous centre of rotation. Writing the two contact
/// velocities in terms of the body twist (v, omega) at the midpoint:
///
///     v_r = v + omega * L/2
///     v_l = v - omega * L/2
///
/// Solving the pair:
///
///     v     = (v_r + v_l) / 2 = r * (omega_r + omega_l) / 2
///     omega = (v_r - v_l) / L = r * (omega_r - omega_l) / L
///
/// That is the FORWARD kinematics, and the whole of Phase 3 rests on it.
/// Note what it says: the wheels only ever tell you v and omega. They say
/// nothing about position; position only exists after integration, and every
/// error committed in one step is carried forever.
///
/// ======================= THE CONTINUOUS MODEL ===========================
///
/// A differential drive cannot move sideways, which is a NON-HOLONOMIC
/// constraint. The body-frame velocity is always (v, 0, omega), and in the
/// world frame:
///
///     x_dot     = v * cos(theta)
///     y_dot     = v * sin(theta)
///     theta_dot = omega
///
/// ========================= THE INTEGRATION ==============================
///
/// The naive discretisation is forward Euler:
///
///     x += v * cos(theta) * dt          <-- WRONG, and wrong in a biased way
///
/// This assumes the heading is constant over the step, so while turning it
/// always cuts the corner: it integrates the chord of the arc using the
/// STARTING tangent. The error does not average out, because it always points
/// the same way relative to the turn, so a robot driving in a circle spirals
/// inwards a little on every step.
///
/// With v and omega constant over the step, the exact path is a circular arc,
/// and the exact update is available in closed form. Starting from
///
///     x += (v/omega) * [sin(theta + omega*dt) - sin(theta)]
///     y += -(v/omega) * [cos(theta + omega*dt) - cos(theta)]
///
/// and applying the sum-to-product identities with a = omega*dt:
///
///     sin(theta + a) - sin(theta) =  2 * cos(theta + a/2) * sin(a/2)
///     cos(theta + a) - cos(theta) = -2 * sin(theta + a/2) * sin(a/2)
///
/// gives a form with no singularity at omega = 0:
///
///     x     += v*dt * sinc(a/2) * cos(theta + a/2)
///     y     += v*dt * sinc(a/2) * sin(theta + a/2)
///     theta += a
///
/// where sinc(u) = sin(u)/u, and sinc(0) = 1. Three things are worth noticing:
///
///   * At omega = 0 this reduces exactly to a straight line, with no special
///     case and no division by a small number. The classic
///     `if (fabs(omega) < 1e-6)` branch exists only because the (v/omega)
///     form was used; the identity removes the need for it.
///
///   * The heading used is the MIDPOINT of the step, theta + a/2, not the
///     starting heading. That alone is the difference between first-order and
///     second-order accuracy.
///
///   * sinc(a/2) <= 1 always. It is the ratio of the chord to the arc: the
///     straight-line displacement really is shorter than the distance the
///     wheels rolled, and this factor is exactly how much shorter.
///
/// This is the exponential map of SE(2) applied to the body twist, which is
/// why it is exact rather than merely accurate.
class DifferentialDriveOdometry
{
public:
  explicit DifferentialDriveOdometry(const OdometryConfig & config);

  /// Integrate one encoder sample.
  ///
  /// Takes wheel POSITIONS, not velocities, and differences them internally.
  /// A quadrature encoder does not measure velocity; the velocity it reports
  /// is already a difference of counts, and differencing positions here keeps
  /// the integration consistent with what was actually counted. It also means
  /// a dropped sample costs nothing: the next step simply covers a longer
  /// interval, instead of losing the motion that happened during the gap.
  OdometryStep update(double left_wheel_rad, double right_wheel_rad, double stamp_s);

  const arb_core::Pose2D & pose() const {return pose_;}
  const Eigen::Matrix3d & covariance() const {return covariance_;}

  double linear_velocity() const {return last_.linear_velocity;}
  double angular_velocity() const {return last_.angular_velocity;}
  const OdometryStep & last_step() const {return last_;}

  /// Total ground distance rolled, regardless of direction. Drift is usually
  /// quoted as a fraction of this, not of elapsed time.
  double distance_travelled_m() const {return distance_travelled_;}

  /// Force the estimate somewhere, e.g. when an external correction arrives.
  void reset(const arb_core::Pose2D & pose = {},
             const Eigen::Matrix3d & covariance = Eigen::Matrix3d::Zero());

  /// sin(u)/u, evaluated stably through u = 0.
  ///
  /// The Taylor series 1 - u^2/6 + u^4/120 is used for small u because the
  /// direct quotient loses precision catastrophically there: at u = 1e-8 the
  /// subtraction inside sin(u) has already discarded most of the significant
  /// digits, and sin(u)/u returns something noticeably off 1.
  static double sinc(double u);

  /// d/du [sin(u)/u] = (u*cos(u) - sin(u)) / u^2, evaluated stably through
  /// u = 0 where it equals 0. Needed because the integration itself carries a
  /// sinc factor, so the Jacobian of that integration must differentiate it.
  static double sincDerivative(double u);

private:
  OdometryConfig config_;
  arb_core::Pose2D pose_;
  Eigen::Matrix3d covariance_{Eigen::Matrix3d::Zero()};

  bool have_previous_{false};
  double previous_stamp_s_{0.0};
  double previous_left_rad_{0.0};
  double previous_right_rad_{0.0};
  double distance_travelled_{0.0};
  OdometryStep last_;
};

}  // namespace arb_localization

#endif  // ARB_LOCALIZATION__DIFFERENTIAL_DRIVE_ODOMETRY_HPP_
