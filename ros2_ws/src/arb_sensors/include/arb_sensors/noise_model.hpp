// Copyright (c) 2026 Emin Ayhan. MIT License.
#ifndef ARB_SENSORS__NOISE_MODEL_HPP_
#define ARB_SENSORS__NOISE_MODEL_HPP_

#include <cmath>
#include <cstdint>
#include <random>
#include <string>

namespace arb_sensors
{

/// Deterministic random source.
///
/// WHY A WRAPPER INSTEAD OF A BARE std::mt19937
/// --------------------------------------------
/// Every number this project reports depends on the noise that was injected.
/// If the noise is not reproducible, a benchmark cannot be repeated, a
/// regression cannot be bisected, and "the RMSE got worse" is indistinguishable
/// from "this run drew unlucky samples".
///
/// So: one explicitly seeded generator per sensor, seeded from configuration,
/// never from std::random_device or the clock. Two nodes given the same seed
/// must not share a stream either -- each sensor derives its own seed by
/// hashing its name into the base seed, so changing the LiDAR configuration
/// does not shift the IMU's noise sequence.
class RandomSource
{
public:
  RandomSource() : RandomSource(0u, "") {}

  RandomSource(std::uint32_t base_seed, const std::string & stream_name)
  : engine_(mix(base_seed, stream_name)), normal_(0.0, 1.0), uniform_(0.0, 1.0) {}

  /// Standard normal sample, N(0, 1).
  double gaussian() { return normal_(engine_); }

  /// Sample from N(mean, sigma^2). A non-positive sigma returns the mean
  /// exactly, which makes "disable this noise source" a config change rather
  /// than a code path.
  double gaussian(double mean, double sigma)
  {
    return (sigma > 0.0) ? mean + sigma * normal_(engine_) : mean;
  }

  /// Uniform sample in [0, 1).
  double uniform() { return uniform_(engine_); }

  /// True with probability p. p <= 0 never fires, p >= 1 always does.
  bool bernoulli(double p)
  {
    if (p <= 0.0) {return false;}
    if (p >= 1.0) {return true;}
    return uniform_(engine_) < p;
  }

  /// Re-seed, discarding all state. Used when a scenario restarts so that
  /// repeated runs of the same experiment are bit-identical.
  void reseed(std::uint32_t base_seed, const std::string & stream_name)
  {
    engine_.seed(mix(base_seed, stream_name));
    normal_.reset();
  }

  /// FNV-1a over the stream name, folded into the base seed. Any cheap
  /// avalanche function works; the requirement is only that distinct names
  /// give distinct, well-separated seeds.
  static std::uint32_t mix(std::uint32_t base_seed, const std::string & name)
  {
    std::uint32_t h = 2166136261u;
    for (const char c : name) {
      h ^= static_cast<std::uint8_t>(c);
      h *= 16777619u;
    }
    return base_seed ^ (h + 0x9e3779b9u + (base_seed << 6) + (base_seed >> 2));
  }

private:
  std::mt19937 engine_;
  std::normal_distribution<double> normal_;
  std::uniform_real_distribution<double> uniform_;
};

/// A slowly varying sensor bias, modelled as an Ornstein-Uhlenbeck process.
///
/// THE MODEL
/// ---------
/// Real MEMS gyroscopes and accelerometers do not just add white noise. Their
/// zero point wanders: a bias that is nearly constant over seconds but drifts
/// over minutes. The standard description is a first-order Gauss-Markov
/// (Ornstein-Uhlenbeck) process
///
///     db/dt = -(1/tau) * b + sigma_w * w(t)
///
/// whose exact discrete-time update over a step dt is
///
///     b_{k+1} = b_k * exp(-dt/tau) + sigma_ss * sqrt(1 - exp(-2*dt/tau)) * n
///
/// with n ~ N(0,1) and sigma_ss the steady-state standard deviation (the
/// datasheet's "bias stability"). Using the exact update rather than a naive
/// Euler step matters: it is stable for any dt, and its stationary variance is
/// exactly sigma_ss^2 rather than something that depends on the sample rate.
///
/// WHY IT MATTERS FOR THE ESTIMATOR
/// --------------------------------
/// White noise averages away; a bias does not. Integrating a gyro with a
/// constant bias b for time t gives a heading error of b*t that grows without
/// bound. This is the single largest error source in wheel-plus-IMU dead
/// reckoning, and the reason the EKF in Phase 4 has to be told how much to
/// trust the IMU rather than simply integrating it.
///
/// Setting tau <= 0 degenerates to a pure random walk (no mean reversion),
/// which is the worst case and is what the "encoder drift" fault uses.
class GaussMarkovBias
{
public:
  GaussMarkovBias() = default;

  /// \param steady_state_sigma  1-sigma spread of the bias once settled
  /// \param correlation_time_s  tau; <= 0 means a pure random walk
  /// \param random_walk_sigma   used only when tau <= 0, in units per sqrt(s)
  GaussMarkovBias(double steady_state_sigma, double correlation_time_s,
                  double random_walk_sigma = 0.0)
  : sigma_ss_(steady_state_sigma),
    tau_(correlation_time_s),
    sigma_rw_(random_walk_sigma) {}

  /// Start the bias at a random draw from its stationary distribution rather
  /// than at zero. A sensor that powers on with a perfect zero is not a sensor
  /// anyone has ever owned, and starting at zero makes the first seconds of
  /// every run optimistic in a way that flatters the estimator.
  void initialise(RandomSource & rng)
  {
    value_ = (tau_ > 0.0) ? rng.gaussian(0.0, sigma_ss_) : 0.0;
    initialised_ = true;
  }

  /// Advance by dt and return the new bias value.
  double update(double dt, RandomSource & rng)
  {
    if (!initialised_) {initialise(rng);}
    if (dt <= 0.0) {return value_;}

    if (tau_ > 0.0) {
      const double decay = std::exp(-dt / tau_);
      const double driving = sigma_ss_ * std::sqrt(std::max(0.0, 1.0 - decay * decay));
      value_ = value_ * decay + driving * rng.gaussian();
    } else {
      // Pure random walk: variance grows linearly with time, hence sqrt(dt).
      value_ += sigma_rw_ * std::sqrt(dt) * rng.gaussian();
    }
    return value_;
  }

  double value() const {return value_;}
  void set(double v) {value_ = v; initialised_ = true;}
  void reset() {value_ = 0.0; initialised_ = false;}

private:
  double sigma_ss_{0.0};
  double tau_{0.0};
  double sigma_rw_{0.0};
  double value_{0.0};
  bool initialised_{false};
};

/// Complete corruption model for one scalar measurement channel.
///
///     measured = (1 + scale_error) * truth + bias(t) + white_noise
///
/// The three terms are deliberately separate because they have completely
/// different consequences downstream:
///
///   * white noise     -> averages out; the EKF's R matrix handles it
///   * bias            -> integrates into unbounded drift; must be estimated
///                        or bounded by an independent measurement
///   * scale error     -> proportional to the signal, so it only shows up when
///                        the robot is actually moving, which makes it look
///                        like it appears and disappears
///
/// `noise_scale` multiplies the white-noise sigma at runtime. That is the knob
/// the Phase 10 "high IMU noise" fault turns, and keeping it separate from the
/// configured sigma means the nominal specification is never overwritten.
class ChannelNoise
{
public:
  ChannelNoise() = default;

  /// \param white_sigma  the sensor's nominal 1-sigma white noise
  /// \param input_carries_nominal_noise  true when the incoming sample ALREADY
  ///        contains `white_sigma` of independent noise, as it does when the
  ///        simulator models the datasheet noise itself.
  ChannelNoise(double white_sigma, double bias_sigma, double bias_tau_s,
               double bias_random_walk_sigma, double scale_error,
               bool input_carries_nominal_noise = false)
  : white_sigma_(white_sigma),
    scale_error_(scale_error),
    input_carries_nominal_noise_(input_carries_nominal_noise),
    bias_(bias_sigma, bias_tau_s, bias_random_walk_sigma) {}

  /// Corrupt one sample. `dt` is the interval since the previous call and is
  /// used only to advance the bias process.
  double apply(double truth, double dt, RandomSource & rng)
  {
    const double bias = bias_.update(dt, rng);
    return (1.0 + scale_error_) * truth + bias + rng.gaussian(0.0, added_sigma());
  }

  /// Noise this stage actually injects, which is NOT the same as the total.
  ///
  /// WHY THE DISTINCTION EXISTS
  /// --------------------------
  /// Gazebo already applies the datasheet noise inside the sensor model. If
  /// this stage then adds another full sigma on top, the two independent
  /// sources combine as sqrt(sigma^2 + sigma^2) = 1.41 * sigma. The stream is
  /// 41 % noisier than the part we claim to be simulating, every covariance we
  /// publish is wrong by the same factor, and the Phase 4 filter is mistuned
  /// from the first line of code.
  ///
  /// Independent variances ADD, so to reach a target total of
  /// `noise_scale * sigma` when the input already carries `sigma`, this stage
  /// must inject
  ///
  ///     sqrt((noise_scale * sigma)^2 - sigma^2)
  ///
  /// which is exactly zero at noise_scale = 1: nominal operation adds nothing
  /// and preserves the simulator's own model.
  double added_sigma() const
  {
    const double target = white_sigma_ * noise_scale_;
    if (!input_carries_nominal_noise_) {return target;}
    const double extra_variance = target * target - white_sigma_ * white_sigma_;
    return (extra_variance > 0.0) ? std::sqrt(extra_variance) : 0.0;
  }

  /// TOTAL white-noise standard deviation of the published sample, including
  /// whatever the input already carried and any runtime scaling. This is what
  /// belongs in the measurement covariance handed to the EKF: the filter must
  /// be told when a sensor has become noisier, otherwise it keeps trusting it.
  double effective_sigma() const {return white_sigma_ * noise_scale_; }

  double bias() const {return bias_.value();}

  void set_noise_scale(double s) {noise_scale_ = (s > 0.0) ? s : 0.0;}
  double noise_scale() const {return noise_scale_;}

  void set_bias_offset(double b) {bias_.set(b);}
  void reset() {bias_.reset(); noise_scale_ = 1.0;}

private:
  double white_sigma_{0.0};
  double scale_error_{0.0};
  double noise_scale_{1.0};
  bool input_carries_nominal_noise_{false};
  GaussMarkovBias bias_;
};

}  // namespace arb_sensors

#endif  // ARB_SENSORS__NOISE_MODEL_HPP_
