// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Subscribes to the simulator's raw IMU stream, applies the configured noise,
// bias and fault model, and republishes a message that carries an honest
// measurement covariance.
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "arb_sensors/imu_processor.hpp"

namespace arb_sensors
{

class ImuPreprocessorNode : public rclcpp::Node
{
public:
  ImuPreprocessorNode()
  : rclcpp::Node("imu_preprocessor")
  {
    ImuConfig cfg;
    cfg.rate_hz = declare_parameter("rate_hz", 200.0);

    // Datasheets quote a noise DENSITY in units per sqrt(Hz). Gazebo and this
    // stage both want a discrete-time sigma. Doing the conversion here, once,
    // keeps configs/sensors.yaml readable as a datasheet and stops the
    // sqrt(rate) factor (about 14x at 200 Hz) from being applied twice or not
    // at all.
    const double gyro_density = declare_parameter("gyro.noise_density", 0.00035);
    const double accel_density = declare_parameter("accel.noise_density", 0.0035);
    const double sqrt_rate = std::sqrt(cfg.rate_hz);
    cfg.gyro_white_sigma = gyro_density * sqrt_rate;
    cfg.accel_white_sigma = accel_density * sqrt_rate;

    cfg.gyro_bias_sigma = declare_parameter("gyro.bias_stability", 0.0002);
    cfg.gyro_bias_tau_s = declare_parameter("gyro.bias_correlation_time_s", 300.0);
    cfg.gyro_bias_random_walk = declare_parameter("gyro.bias_random_walk", 0.00002);
    cfg.gyro_scale_error = declare_parameter("gyro.scale_error", 0.002);

    cfg.accel_bias_sigma = declare_parameter("accel.bias_stability", 0.005);
    cfg.accel_bias_tau_s = declare_parameter("accel.bias_correlation_time_s", 300.0);
    cfg.accel_bias_random_walk = declare_parameter("accel.bias_random_walk", 0.0004);
    cfg.accel_scale_error = declare_parameter("accel.scale_error", 0.003);

    cfg.input_carries_nominal_noise =
      declare_parameter("input_carries_nominal_noise", true);
    cfg.calibration_samples = static_cast<int>(
      declare_parameter("calibration_samples", 200));
    cfg.max_dt_s = declare_parameter("max_dt_s", 0.5);
    cfg.seed = static_cast<std::uint32_t>(declare_parameter("seed", 1));

    const std::string input = declare_parameter("input_topic", "/imu/data_raw");
    const std::string output = declare_parameter("output_topic", "/arb/imu");

    // Runtime fault knobs. Declared as parameters so Phase 10 can degrade the
    // sensor with `ros2 param set` and no restart, and so a scenario file can
    // be replayed against a running stack.
    declare_parameter("fault.noise_scale", 1.0);
    declare_parameter("fault.gyro_bias_offset", 0.0);
    declare_parameter("fault.outage", false);

    processor_ = std::make_unique<ImuProcessor>(cfg);
    apply_fault_parameters();

    // Sensor data is BEST_EFFORT: a dropped sample beats a late one, because
    // retransmission delivers stale data into a real-time filter.
    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(*msg);});

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_parameters(params);
      });

    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "IMU preprocessor: %s -> %s | gyro sigma %.6f rad/s, accel sigma %.6f m/s^2, "
      "calibrating over %d samples",
      input.c_str(), output.c_str(), cfg.gyro_white_sigma, cfg.accel_white_sigma,
      cfg.calibration_samples);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const auto result = processor_->process(msg, stamp);
    ++received_;

    if (!result.message) {
      switch (result.reason) {
        case ImuReject::kOutage: ++dropped_outage_; break;
        case ImuReject::kNonFinite: ++dropped_nonfinite_; break;
        case ImuReject::kNonMonotonicTime: ++dropped_time_; break;
        case ImuReject::kCalibrating: ++calibrating_; break;
        default: break;
      }
      if (result.reason == ImuReject::kCalibrating && processor_->calibrated()) {
        const auto & b = processor_->gyro_calibration_bias();
        RCLCPP_INFO(
          get_logger(),
          "IMU calibration complete: removing gyro bias [%.6f %.6f %.6f] rad/s",
          b[0], b[1], b[2]);
      }
      return;
    }
    publisher_->publish(*result.message);
    ++published_;
  }

  void apply_fault_parameters()
  {
    ImuFaultState fault;
    fault.noise_scale = get_parameter("fault.noise_scale").as_double();
    fault.gyro_bias_offset = get_parameter("fault.gyro_bias_offset").as_double();
    fault.outage = get_parameter("fault.outage").as_bool();
    processor_->set_fault(fault);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    ImuFaultState fault = processor_->fault();
    for (const auto & p : params) {
      if (p.get_name() == "fault.noise_scale") {
        if (p.as_double() < 0.0) {
          result.successful = false;
          result.reason = "fault.noise_scale must be >= 0";
          return result;
        }
        fault.noise_scale = p.as_double();
      } else if (p.get_name() == "fault.gyro_bias_offset") {
        fault.gyro_bias_offset = p.as_double();
      } else if (p.get_name() == "fault.outage") {
        fault.outage = p.as_bool();
      } else if (p.get_name().rfind("fault.", 0) == 0) {
        result.successful = false;
        result.reason = "unknown fault parameter: " + p.get_name();
        return result;
      } else {
        // Everything else describes the sensor itself and is read once at
        // construction. Silently accepting a change would leave the parameter
        // server disagreeing with the running model.
        result.successful = false;
        result.reason = p.get_name() + " is not reconfigurable at runtime";
        return result;
      }
    }
    processor_->set_fault(fault);
    RCLCPP_INFO(
      get_logger(), "IMU fault state: noise_scale=%.2f bias_offset=%.5f outage=%s",
      fault.noise_scale, fault.gyro_bias_offset, fault.outage ? "true" : "false");
    return result;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(),
      "IMU: received=%lu published=%lu | dropped: outage=%lu non-finite=%lu "
      "time=%lu calibrating=%lu | sigma_gyro=%.6f",
      received_, published_, dropped_outage_, dropped_nonfinite_, dropped_time_,
      calibrating_, processor_->gyro_sigma());
  }

  std::unique_ptr<ImuProcessor> processor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  unsigned long received_{0}, published_{0};
  unsigned long dropped_outage_{0}, dropped_nonfinite_{0}, dropped_time_{0};
  unsigned long calibrating_{0};
};

}  // namespace arb_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_sensors::ImuPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
