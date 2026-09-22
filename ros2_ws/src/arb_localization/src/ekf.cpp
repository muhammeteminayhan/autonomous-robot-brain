// Copyright (c) 2026 Emin Ayhan. MIT License.
#include "arb_localization/ekf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace arb_localization
{
namespace
{
/// Smallest LDLT pivot accepted, relative to the largest diagonal entry of S.
/// Below this the innovation covariance is numerically singular.
constexpr double kPivotTolerance = 1e-12;
}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter()
: ExtendedKalmanFilter(Config{}) {}

ExtendedKalmanFilter::ExtendedKalmanFilter(const Config & config)
: config_(config)
{
  covariance_.setZero();
}

double ExtendedKalmanFilter::sinc(double u)
{
  constexpr double kSmall = 1e-4;
  if (std::abs(u) < kSmall) {
    const double u2 = u * u;
    return 1.0 - u2 / 6.0 + u2 * u2 / 120.0;
  }
  return std::sin(u) / u;
}

double ExtendedKalmanFilter::sincDerivative(double u)
{
  constexpr double kSmall = 1e-3;
  if (std::abs(u) < kSmall) {
    const double u2 = u * u;
    return -u / 3.0 + u * u2 / 30.0;
  }
  return (u * std::cos(u) - std::sin(u)) / (u * u);
}

void ExtendedKalmanFilter::initialise(
  const StateVector & state, const StateMatrix & covariance)
{
  state_ = state;
  state_(THETA) = arb_core::normalizeAngle(state_(THETA));
  covariance_ = covariance;
  symmetrise();
  consecutive_rejections_ = 0;
}

void ExtendedKalmanFilter::symmetrise()
{
  covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval();
}

bool ExtendedKalmanFilter::predict(double dt)
{
  if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.max_predict_dt) {
    return false;
  }

  const double theta = state_(THETA);
  const double v = state_(V);
  const double w = state_(OMEGA);

  const double a = w * dt;
  const double half = 0.5 * a;
  const double k = sinc(half);
  const double dk = sincDerivative(half);
  const double heading_mid = theta + half;
  const double c = std::cos(heading_mid);
  const double s = std::sin(heading_mid);
  const double chord = v * dt * k;

  // --- state propagation (SE(2) exponential map, exactly as Phase 3) -------
  state_(X) += chord * c;
  state_(Y) += chord * s;
  state_(THETA) = arb_core::normalizeAngle(theta + a);
  // v, omega and the bias are unchanged by the model; only their uncertainty
  // grows. That is what "constant velocity model" means.

  // --- Jacobian -----------------------------------------------------------
  //
  // d(x')/d(omega) needs the chain rule through BOTH occurrences of omega:
  // the sinc factor and the midpoint heading, each contributing dt/2.
  //
  //   x' = x + v*dt*sinc(w*dt/2)*cos(theta + w*dt/2)
  //   dx'/dw = v*dt*[ sinc'(half)*(dt/2)*cos(.) - sinc(half)*sin(.)*(dt/2) ]
  //          = v*dt^2/2 * [ dk*cos(.) - k*sin(.) ]
  //
  // Dropping either term leaves F inconsistent with f, and the covariance
  // then describes a different filter from the one actually running.
  StateMatrix F = StateMatrix::Identity();
  F(X, THETA) = -chord * s;
  F(X, V) = dt * k * c;
  F(X, OMEGA) = v * dt * dt * 0.5 * (dk * c - k * s);
  F(Y, THETA) = chord * c;
  F(Y, V) = dt * k * s;
  F(Y, OMEGA) = v * dt * dt * 0.5 * (dk * s + k * c);
  F(THETA, OMEGA) = dt;

  // --- process noise ------------------------------------------------------
  //
  // EVERY term scales as dt, not dt^2, and the difference is not cosmetic.
  //
  // A velocity driven by white acceleration noise is a RANDOM WALK. Its
  // variance accumulates as q * T, independent of the step size used to
  // integrate it: splitting one 20 ms step into four 5 ms steps must not
  // change how uncertain the filter is after 20 ms. Writing the per-step
  // variance as (sigma * dt)^2 instead gives sigma^2 * T * dt, which SHRINKS
  // as the step shrinks -- so running the same filter at 200 Hz instead of
  // 50 Hz makes it four times more confident for no physical reason.
  //
  // Measured consequence of getting this wrong, with accel_sigma = 0.8 m/s^2
  // and 5 ms prediction steps: the filter accumulated only 0.008 m/s of
  // velocity uncertainty between 50 Hz odometry updates, while the platform's
  // 1.0 m/s^2 acceleration limit changes v by 0.020 m/s over the same
  // interval. Every real deceleration therefore arrived as a 2.5-sigma
  // measurement, the chi-square gate REJECTED it, and the filter kept
  // believing the robot was still moving. On a rectangle route it traced a
  // half-metre loop at every corner: while the robot pivoted in place, the
  // filter held v = 0.08 m/s against an odometry reading of 0.0001 m/s.
  //
  // Correct scaling: after one second with no measurement, the velocity is
  // uncertain by accel_sigma * 1 s, which is the physical meaning the
  // parameter claims to have.
  StateMatrix Q = StateMatrix::Zero();
  Q(V, V) = config_.accel_sigma * config_.accel_sigma * dt;
  Q(OMEGA, OMEGA) =
    config_.angular_accel_sigma * config_.angular_accel_sigma * dt;
  Q(GYRO_BIAS, GYRO_BIAS) =
    config_.gyro_bias_random_walk * config_.gyro_bias_random_walk * dt;
  Q(X, X) = config_.position_process_sigma * config_.position_process_sigma * dt;
  Q(Y, Y) = Q(X, X);
  Q(THETA, THETA) =
    config_.heading_process_sigma * config_.heading_process_sigma * dt;

  covariance_ = F * covariance_ * F.transpose() + Q;
  symmetrise();
  return true;
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::applyUpdate(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & H,
  const Eigen::MatrixXd & R,
  double gate,
  const std::string & source)
{
  UpdateResult result;
  result.innovation = innovation;
  result.gate = gate;
  result.source = source;

  const Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;

  // The innovation covariance must be invertible. A singular S means the
  // measurement carries no information in some direction, which is a
  // configuration error rather than a numerical accident, so bail out loudly
  // instead of producing an enormous gain.
  //
  // Checking only `ldlt.info()` is NOT enough: Eigen's LDLT is designed for
  // positive SEMI-definite matrices and reports Success for a singular one,
  // after which solve() returns whatever the zero pivots produce. The pivots
  // themselves have to be inspected, and the threshold has to be RELATIVE to
  // the scale of S -- an absolute epsilon would reject a well-conditioned
  // matrix whose entries happen to be small, and accept an ill-conditioned one
  // whose entries happen to be large.
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  const double scale = std::max(1e-300, S.diagonal().cwiseAbs().maxCoeff());
  const double min_pivot = ldlt.vectorD().minCoeff();
  if (ldlt.info() != Eigen::Success || min_pivot <= kPivotTolerance * scale) {
    result.accepted = false;
    result.nis = std::numeric_limits<double>::infinity();
    ++consecutive_rejections_;
    return result;
  }

  // Normalised innovation squared. Under the filter's own assumptions this is
  // chi-square distributed with dim(z) degrees of freedom, so it measures the
  // surprise in units the filter itself defines. That is the useful property:
  // the threshold does not depend on the sensor's units or the robot's speed.
  const Eigen::VectorXd S_inv_y = ldlt.solve(innovation);
  result.nis = innovation.dot(S_inv_y);

  if (config_.enable_gating && result.nis > gate) {
    ++consecutive_rejections_;
    if (consecutive_rejections_ <= config_.max_consecutive_rejections) {
      result.accepted = false;
      return result;
    }
    // Forced open: a filter that rejects everything is lost, and a lost filter
    // cannot recover without accepting something.
    consecutive_rejections_ = 0;
  } else {
    consecutive_rejections_ = 0;
  }

  const Eigen::MatrixXd K = covariance_ * H.transpose() * ldlt.solve(
    Eigen::MatrixXd::Identity(S.rows(), S.cols()));

  state_ += K * innovation;
  state_(THETA) = arb_core::normalizeAngle(state_(THETA));

  // Joseph form: a sum of two positive semi-definite terms, so symmetry and
  // positive-definiteness survive rounding. See the header for why the
  // textbook (I - K H) P form is not good enough.
  const StateMatrix IKH =
    StateMatrix::Identity() - K * H;
  covariance_ = IKH * covariance_ * IKH.transpose() + K * R * K.transpose();
  symmetrise();

  result.accepted = true;
  return result;
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateWheelTwist(
  double v, double omega, double sigma_v, double sigma_omega)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, kStateDim);
  H(0, V) = 1.0;
  H(1, OMEGA) = 1.0;

  Eigen::VectorXd y(2);
  y << v - state_(V), omega - state_(OMEGA);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
  R(0, 0) = sigma_v * sigma_v;
  R(1, 1) = sigma_omega * sigma_omega;

  return applyUpdate(y, H, R, config_.nis_gate_2dof, "wheel_odom");
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateGyro(
  double yaw_rate, double sigma)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, kStateDim);
  H(0, OMEGA) = 1.0;
  H(0, GYRO_BIAS) = 1.0;

  Eigen::VectorXd y(1);
  y << yaw_rate - (state_(OMEGA) + state_(GYRO_BIAS));

  Eigen::MatrixXd R(1, 1);
  R(0, 0) = sigma * sigma;

  return applyUpdate(y, H, R, config_.nis_gate_1dof, "imu");
}

ExtendedKalmanFilter::UpdateResult ExtendedKalmanFilter::updateAbsolutePose(
  const arb_core::Pose2D & pose, const Eigen::Matrix3d & covariance)
{
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kStateDim);
  H(0, X) = 1.0;
  H(1, Y) = 1.0;
  H(2, THETA) = 1.0;

  Eigen::VectorXd y(3);
  // The heading residual MUST be wrapped. An estimate at +179 deg and a
  // measurement at -179 deg differ by 2 deg, not 358; an unwrapped residual
  // hands the filter a fictitious enormous error, the gain reacts to it, and
  // the filter diverges in a single step.
  y << pose.x - state_(X),
    pose.y - state_(Y),
    arb_core::angleDiff(pose.theta, state_(THETA));

  return applyUpdate(y, H, covariance, config_.nis_gate_3dof, "absolute_pose");
}

}  // namespace arb_localization
