// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_LOCALIZATION__EKF_HPP_
#define ARB_LOCALIZATION__EKF_HPP_

#include <Eigen/Dense>
#include <optional>
#include <string>

#include "arb_core/angles.hpp"
#include "arb_core/pose2d.hpp"

namespace arb_localization
{

/// Extended Kalman Filter for a planar differential-drive robot.
///
/// ======================== THE STATE VECTOR ========================
///
///     x = [ x, y, theta, v, omega, b_gyro ]^T          (6 states)
///
/// WHY THESE SIX, AND NOT OTHERS
///
/// * `x, y, theta` -- the pose, which is the answer everything downstream
///   wants.
///
/// * `v, omega` -- the body twist. These are states rather than inputs because
///   TWO independent sensors measure them: the wheel encoders measure both,
///   and the gyroscope measures omega. Making them states is what lets the
///   filter fuse those two opinions instead of picking one. Treating the
///   encoders as a control input instead (the common "odometry as u" setup)
///   throws the gyro's information away, and Phase 3 measured that the entire
///   residual error lives in rotation.
///
/// * `b_gyro` -- the gyroscope's slowly varying zero offset. This is the state
///   that earns its place least obviously and matters most. A gyro bias is not
///   noise: integrated for t seconds it produces b*t of heading error, without
///   bound. Phase 2 modelled it explicitly as an Ornstein-Uhlenbeck process
///   precisely because it cannot be averaged away.
///
///   It is OBSERVABLE here because the encoders measure omega independently.
///   The filter sees two estimates of the same rotation rate, and the
///   persistent difference between them is the bias. With only a gyro it would
///   be unobservable and the filter would wander.
///
/// WHAT IS DELIBERATELY ABSENT
///
/// * `v_y` -- the sideways body velocity. A differential drive is
///   NON-HOLONOMIC: it physically cannot translate sideways, so v_y is
///   identically zero. Adding it would create a state no measurement
///   constrains; the filter would fill that direction with process noise, the
///   covariance would grow without bound along it, and the position estimate
///   would slowly smear sideways. An unobservable state is not free.
///
/// * accelerometer bias -- the accelerometer is not used in the measurement
///   model at all. For a planar robot at walking pace the useful signal is
///   buried under gravity leakage from millidegree chassis tilt, and
///   integrating it twice to get position is strictly worse than the wheel
///   encoders. It is published and logged, and not believed.
///
/// ========================= THE SYSTEM MODEL =========================
///
/// Constant twist between updates, integrated exactly on SE(2), the same
/// exponential map used in Phase 3:
///
///     a     = omega * dt,   half = a/2,   k = sinc(half)
///     x'    = x + v*dt*k*cos(theta + half)
///     y'    = y + v*dt*k*sin(theta + half)
///     theta'= theta + a
///     v'    = v                  driven by acceleration noise
///     omega'= omega              driven by angular acceleration noise
///     b'    = b                  driven by a random walk
///
/// "Constant velocity" is not a claim that the robot never accelerates. It is
/// a statement that the filter does not know the commanded acceleration, so it
/// models the change as noise. The size of that noise is what `accel_sigma`
/// and `angular_accel_sigma` declare.
class ExtendedKalmanFilter
{
public:
  static constexpr int kStateDim = 6;
  enum StateIndex { X = 0, Y = 1, THETA = 2, V = 3, OMEGA = 4, GYRO_BIAS = 5 };

  using StateVector = Eigen::Matrix<double, kStateDim, 1>;
  using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

  struct Config
  {
    /// Process noise. Each is the 1-sigma of the driving disturbance over one
    /// second, and enters the discrete Q scaled by dt as its physics requires.
    double accel_sigma{0.8};            ///< m/s^2, uncertainty in v
    double angular_accel_sigma{2.0};    ///< rad/s^2, uncertainty in omega
    double gyro_bias_random_walk{2e-5}; ///< rad/s per sqrt(s)

    /// Small direct process noise on the pose itself, covering everything the
    /// twist model does not: wheel slip, an uneven floor, a bumped robot.
    /// Without it the pose covariance is driven only through v and omega, and
    /// the filter becomes certain about a pose it has no right to be certain
    /// about while standing still.
    double position_process_sigma{0.002};   ///< m per sqrt(s)
    double heading_process_sigma{0.002};    ///< rad per sqrt(s)

    /// Chi-square gate on the normalised innovation squared. A measurement
    /// whose NIS exceeds the threshold for its dimension is REJECTED.
    ///
    /// This is the filter's immune system. An EKF has no defence against a
    /// wrong measurement: it will happily fuse a scan match that locked onto
    /// the wrong corridor, and once the estimate has moved the next match is
    /// even more wrong. The gate turns divergence into a visible rejection.
    ///
    /// Chi-square 99 % quantiles: 6.63 (1 dof), 9.21 (2 dof), 11.34 (3 dof).
    bool enable_gating{true};
    double nis_gate_1dof{6.63};
    double nis_gate_2dof{9.21};
    double nis_gate_3dof{11.34};

    /// Consecutive rejections after which the gate is forced open once.
    ///
    /// A gate that never yields is a trap: if the filter is genuinely lost,
    /// every correct measurement looks like an outlier and is rejected
    /// forever. Letting one through after a run of rejections lets the filter
    /// recover, at the cost of occasionally accepting a real outlier.
    int max_consecutive_rejections{25};

    /// Largest interval the prediction may advance in one step.
    double max_predict_dt{0.5};
  };

  /// What one update did, for diagnostics and tests.
  struct UpdateResult
  {
    bool accepted{false};
    double nis{0.0};
    double gate{0.0};
    Eigen::VectorXd innovation;
    std::string source;
  };

  /// Two constructors rather than one with `= Config()` as a default argument.
  /// A nested class's default member initializers are not usable until the
  /// ENCLOSING class is complete, so a defaulted `Config()` argument declared
  /// inside this class fails to compile with
  /// "default member initializer required before the end of its enclosing
  /// class". Defining the default in the .cpp sidesteps it entirely.
  ExtendedKalmanFilter();
  explicit ExtendedKalmanFilter(const Config & config);

  /// Set the state and covariance, e.g. at startup or after a reset.
  void initialise(const StateVector & state, const StateMatrix & covariance);

  /// Advance the state by `dt` seconds. Returns false if dt was rejected.
  bool predict(double dt);

  /// Fuse a wheel-odometry twist measurement, z = [v, omega].
  UpdateResult updateWheelTwist(double v, double omega,
                                double sigma_v, double sigma_omega);

  /// Fuse a gyroscope reading of the body yaw rate.
  ///
  /// The measurement model is  z = omega + b_gyro, NOT z = omega. Modelling
  /// the bias inside h() is the whole reason the bias is estimable: the filter
  /// attributes the persistent gap between the gyro and the encoders to b,
  /// rather than repeatedly correcting omega towards a biased reading.
  UpdateResult updateGyro(double yaw_rate, double sigma);

  /// Fuse an absolute pose measurement, z = [x, y, theta].
  ///
  /// Used from Phase 6 onwards, when LiDAR scan matching against a map
  /// provides an absolute reference. Until then the filter has no absolute
  /// measurement at all and its position uncertainty can only grow -- which is
  /// not a flaw in the filter but a property of dead reckoning.
  UpdateResult updateAbsolutePose(const arb_core::Pose2D & pose,
                                  const Eigen::Matrix3d & covariance);

  const StateVector & state() const {return state_;}
  const StateMatrix & covariance() const {return covariance_;}

  arb_core::Pose2D pose() const
  {
    return arb_core::Pose2D(state_(X), state_(Y), state_(THETA));
  }
  Eigen::Matrix3d poseCovariance() const {return covariance_.topLeftCorner<3, 3>();}

  double linearVelocity() const {return state_(V);}
  double angularVelocity() const {return state_(OMEGA);}
  double gyroBias() const {return state_(GYRO_BIAS);}

  int consecutiveRejections() const {return consecutive_rejections_;}
  const Config & config() const {return config_;}

  /// sinc(u) = sin(u)/u and its derivative, stable through u = 0.
  static double sinc(double u);
  static double sincDerivative(double u);

private:
  /// Shared Kalman update in Joseph form.
  ///
  /// The textbook update P <- (I - K H) P is algebraically correct and
  /// numerically fragile: it is a difference of two positive quantities, so
  /// rounding can make P asymmetric and eventually indefinite, at which point
  /// the filter reports negative variances and diverges. Joseph form
  ///
  ///     P <- (I - K H) P (I - K H)^T + K R K^T
  ///
  /// is a sum of two positive semi-definite terms, so it preserves symmetry
  /// and positive-definiteness by construction. It costs one extra matrix
  /// product, which for a 6-state filter is nothing.
  UpdateResult applyUpdate(const Eigen::VectorXd & innovation,
                           const Eigen::MatrixXd & H,
                           const Eigen::MatrixXd & R,
                           double gate,
                           const std::string & source);

  void symmetrise();

  Config config_;
  StateVector state_{StateVector::Zero()};
  StateMatrix covariance_{StateMatrix::Identity()};
  int consecutive_rejections_{0};
};

}  // namespace arb_localization

#endif  // ARB_LOCALIZATION__EKF_HPP_
