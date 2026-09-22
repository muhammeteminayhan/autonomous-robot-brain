// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Turns the simulator's exact joint states into something a quadrature encoder
// could have produced: quantised positions, differenced velocities, slip and
// injectable per-wheel drift.
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arb_sensors/encoder_processor.hpp"

namespace arb_sensors
{

class EncoderPreprocessorNode : public rclcpp::Node
{
public:
  EncoderPreprocessorNode()
  : rclcpp::Node("encoder_preprocessor")
  {
    EncoderConfig cfg;
    cfg.rate_hz = declare_parameter("rate_hz", 50.0);
    cfg.left_joint = declare_parameter("left_joint", std::string("left_wheel_joint"));
    cfg.right_joint = declare_parameter("right_joint", std::string("right_wheel_joint"));
    cfg.counts_per_revolution =
      static_cast<int>(declare_parameter("counts_per_revolution", 1024));
    cfg.quadrature_multiplier =
      static_cast<int>(declare_parameter("quadrature_multiplier", 4));
    cfg.velocity_noise_sigma = declare_parameter("velocity_noise_sigma", 0.01);
    cfg.slip_mean = declare_parameter("slip_ratio_mean", 0.0);
    cfg.slip_sigma = declare_parameter("slip_ratio_sigma", 0.01);
    cfg.max_dt_s = declare_parameter("max_dt_s", 0.5);
    cfg.seed = static_cast<std::uint32_t>(declare_parameter("seed", 2));

    const std::string input = declare_parameter("input_topic", "/joint_states");
    const std::string output = declare_parameter("output_topic", "/arb/wheel_states");

    declare_parameter("fault.left_bias_rad_s", 0.0);
    declare_parameter("fault.right_bias_rad_s", 0.0);
    declare_parameter("fault.noise_scale", 1.0);
    declare_parameter("fault.outage", false);

    processor_ = std::make_unique<EncoderProcessor>(cfg);
    apply_fault_parameters();

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::JointState>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input, qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joints(*msg);});

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_parameters(params);
      });
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "Encoder preprocessor: %s -> %s | %d x %d counts/rev -> %.6f rad resolution "
      "(%.3f mm arc at a 60 mm wheel), velocity quantisation at %.0f Hz = %.4f rad/s",
      input.c_str(), output.c_str(), cfg.counts_per_revolution,
      cfg.quadrature_multiplier, processor_->resolution_rad(),
      processor_->resolution_rad() * 0.06 * 1000.0,
      cfg.rate_hz, processor_->resolution_rad() * cfg.rate_hz);
  }

private:
  void on_joints(const sensor_msgs::msg::JointState & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const auto result = processor_->process(msg, stamp);
    ++received_;
    if (!result.message) {
      switch (result.reason) {
        case EncoderReject::kOutage: ++dropped_outage_; break;
        case EncoderReject::kMissingJoint: ++dropped_missing_; break;
        case EncoderReject::kNonFinite: ++dropped_nonfinite_; break;
        case EncoderReject::kNonMonotonicTime: ++dropped_time_; break;
        default: break;
      }
      if (result.reason == EncoderReject::kMissingJoint && dropped_missing_ == 1) {
        RCLCPP_WARN(
          get_logger(),
          "joint names not found in the incoming JointState; check left_joint "
          "and right_joint parameters");
      }
      return;
    }
    publisher_->publish(*result.message);
    ++published_;
    last_ = result;
  }

  void apply_fault_parameters()
  {
    EncoderFaultState fault;
    fault.left_bias_rad_s = get_parameter("fault.left_bias_rad_s").as_double();
    fault.right_bias_rad_s = get_parameter("fault.right_bias_rad_s").as_double();
    fault.noise_scale = get_parameter("fault.noise_scale").as_double();
    fault.outage = get_parameter("fault.outage").as_bool();
    processor_->set_fault(fault);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    EncoderFaultState fault = processor_->fault();
    for (const auto & p : params) {
      if (p.get_name() == "fault.left_bias_rad_s") {
        fault.left_bias_rad_s = p.as_double();
      } else if (p.get_name() == "fault.right_bias_rad_s") {
        fault.right_bias_rad_s = p.as_double();
      } else if (p.get_name() == "fault.noise_scale") {
        if (p.as_double() < 0.0) {
          result.successful = false;
          result.reason = "fault.noise_scale must be >= 0";
          return result;
        }
        fault.noise_scale = p.as_double();
      } else if (p.get_name() == "fault.outage") {
        fault.outage = p.as_bool();
      } else {
        result.successful = false;
        result.reason = p.get_name() + " is not reconfigurable at runtime";
        return result;
      }
    }
    processor_->set_fault(fault);
    RCLCPP_INFO(
      get_logger(),
      "Encoder fault state: left_bias=%.4f right_bias=%.4f noise_scale=%.2f outage=%s",
      fault.left_bias_rad_s, fault.right_bias_rad_s, fault.noise_scale,
      fault.outage ? "true" : "false");
    return result;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(),
      "Encoder: received=%lu published=%lu | dropped: outage=%lu missing=%lu "
      "non-finite=%lu time=%lu | last v=[%.4f %.4f] rad/s sigma=%.4f",
      received_, published_, dropped_outage_, dropped_missing_,
      dropped_nonfinite_, dropped_time_,
      last_.left_velocity_rad_s, last_.right_velocity_rad_s,
      processor_->velocity_sigma(last_.dt_s > 0.0 ? last_.dt_s : 0.02));
  }

  std::unique_ptr<EncoderProcessor> processor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  EncoderResult last_;
  unsigned long received_{0}, published_{0};
  unsigned long dropped_outage_{0}, dropped_missing_{0};
  unsigned long dropped_nonfinite_{0}, dropped_time_{0};
};

}  // namespace arb_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_sensors::EncoderPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
