// Copyright (c) 2026 Emin Ayhan. MIT License.
//
// Normalises the raw scan (one sentinel for every unmeasurable beam),
// despeckles it, and applies the configured noise and dropout model.
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "arb_sensors/lidar_processor.hpp"

namespace arb_sensors
{

class LidarPreprocessorNode : public rclcpp::Node
{
public:
  LidarPreprocessorNode()
  : rclcpp::Node("lidar_preprocessor")
  {
    LidarConfig cfg;
    cfg.rate_hz = declare_parameter("rate_hz", 10.0);
    cfg.range_noise_sigma = declare_parameter("range_noise_sigma", 0.015);
    cfg.input_carries_nominal_noise =
      declare_parameter("input_carries_nominal_noise", true);
    cfg.dropout_probability = declare_parameter("dropout_probability", 0.0);
    cfg.despeckle = declare_parameter("despeckle", true);
    cfg.despeckle_max_obliquity_deg =
      declare_parameter("despeckle_max_obliquity_deg", 85.0);
    cfg.despeckle_floor_m = declare_parameter("despeckle_floor_m", 0.05);
    cfg.seed = static_cast<std::uint32_t>(declare_parameter("seed", 3));

    const std::string input = declare_parameter("input_topic", "/scan_raw");
    const std::string output = declare_parameter("output_topic", "/arb/scan");

    declare_parameter("fault.dropout_ratio", 0.0);
    declare_parameter("fault.noise_scale", 1.0);
    declare_parameter("fault.burst_mode", false);
    declare_parameter("fault.burst_active", false);
    declare_parameter("fault.outage", false);

    processor_ = std::make_unique<LidarProcessor>(cfg);
    apply_fault_parameters();

    const auto qos = rclcpp::SensorDataQoS();
    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output, qos);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input, qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {on_scan(*msg);});

    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_parameters(params);
      });
    report_timer_ = create_wall_timer(
      std::chrono::seconds(10), [this]() {report();});

    RCLCPP_INFO(
      get_logger(),
      "LiDAR preprocessor: %s -> %s | extra noise sigma %.4f m, despeckle=%s "
      "(max obliquity %.1f deg, floor %.2f m)",
      input.c_str(), output.c_str(), cfg.range_noise_sigma,
      cfg.despeckle ? "on" : "off", cfg.despeckle_max_obliquity_deg,
      cfg.despeckle_floor_m);
  }

private:
  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    const auto result = processor_->process(msg, stamp);
    ++received_;
    if (!result.message) {
      switch (result.reason) {
        case LidarReject::kOutage: ++dropped_outage_; break;
        case LidarReject::kEmptyScan: ++dropped_empty_; break;
        case LidarReject::kNonMonotonicTime: ++dropped_time_; break;
        default: break;
      }
      return;
    }
    publisher_->publish(*result.message);
    ++published_;
    last_valid_ = result.valid_beams;
    last_dropped_ = result.dropped_beams;
    last_despeckled_ = result.despeckled_beams;
    total_dropped_ += static_cast<unsigned long>(result.dropped_beams);
    total_despeckled_ += static_cast<unsigned long>(result.despeckled_beams);
  }

  void apply_fault_parameters()
  {
    LidarFaultState fault;
    fault.dropout_ratio = get_parameter("fault.dropout_ratio").as_double();
    fault.noise_scale = get_parameter("fault.noise_scale").as_double();
    fault.burst_mode = get_parameter("fault.burst_mode").as_bool();
    fault.burst_active = get_parameter("fault.burst_active").as_bool();
    fault.outage = get_parameter("fault.outage").as_bool();
    processor_->set_fault(fault);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    LidarFaultState fault = processor_->fault();
    for (const auto & p : params) {
      if (p.get_name() == "fault.dropout_ratio") {
        const double v = p.as_double();
        if (v < 0.0 || v > 1.0) {
          result.successful = false;
          result.reason = "fault.dropout_ratio must be in [0, 1]";
          return result;
        }
        fault.dropout_ratio = v;
      } else if (p.get_name() == "fault.noise_scale") {
        fault.noise_scale = p.as_double();
      } else if (p.get_name() == "fault.burst_mode") {
        fault.burst_mode = p.as_bool();
      } else if (p.get_name() == "fault.burst_active") {
        fault.burst_active = p.as_bool();
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
      "LiDAR fault state: dropout=%.2f noise_scale=%.2f burst=%s/%s outage=%s "
      "(effective dropout %.2f)",
      fault.dropout_ratio, fault.noise_scale,
      fault.burst_mode ? "on" : "off", fault.burst_active ? "active" : "idle",
      fault.outage ? "true" : "false", processor_->effective_dropout());
    return result;
  }

  void report()
  {
    RCLCPP_INFO(
      get_logger(),
      "LiDAR: received=%lu published=%lu | last scan: valid=%d dropped=%d "
      "despeckled=%d | totals: dropped=%lu despeckled=%lu | rejects: outage=%lu "
      "empty=%lu time=%lu",
      received_, published_, last_valid_, last_dropped_, last_despeckled_,
      total_dropped_, total_despeckled_, dropped_outage_, dropped_empty_,
      dropped_time_);
  }

  std::unique_ptr<LidarProcessor> processor_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::TimerBase::SharedPtr report_timer_;

  unsigned long received_{0}, published_{0};
  unsigned long dropped_outage_{0}, dropped_empty_{0}, dropped_time_{0};
  unsigned long total_dropped_{0}, total_despeckled_{0};
  int last_valid_{0}, last_dropped_{0}, last_despeckled_{0};
};

}  // namespace arb_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arb_sensors::LidarPreprocessorNode>());
  rclcpp::shutdown();
  return 0;
}
