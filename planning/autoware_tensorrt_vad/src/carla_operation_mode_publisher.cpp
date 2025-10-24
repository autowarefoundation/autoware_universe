#include <algorithm>
#include <chrono>
#include <string>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::tensorrt_vad
{

class CarlaOperationModePublisher : public rclcpp::Node
{
public:
  explicit CarlaOperationModePublisher(const rclcpp::NodeOptions & options)
  : rclcpp::Node("carla_operation_mode_publisher", options)
  {
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);
    mode_string_ = this->declare_parameter<std::string>("mode", "AUTONOMOUS");

    publisher_ = this->create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>(
      "~/output/state", rclcpp::QoS(1).transient_local());

    update_mode_from_string(mode_string_);

    publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)),
      std::bind(&CarlaOperationModePublisher::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(), "carla_operation_mode_publisher initialized (mode=%s, rate=%.1f Hz)",
      mode_string_.c_str(), publish_rate_hz_);
  }

private:
  void update_mode_from_string(const std::string & mode)
  {
    using autoware_adapi_v1_msgs::msg::OperationModeState;
    if (mode == "AUTONOMOUS") {
      mode_ = OperationModeState::AUTONOMOUS;
    } else if (mode == "STOP") {
      mode_ = OperationModeState::STOP;
    } else if (mode == "LOCAL") {
      mode_ = OperationModeState::LOCAL;
    } else if (mode == "REMOTE") {
      mode_ = OperationModeState::REMOTE;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Unsupported mode '%s', falling back to AUTONOMOUS", mode.c_str());
      mode_ = OperationModeState::AUTONOMOUS;
    }
  }

  void on_timer()
  {
    if (!publisher_) {
      return;
    }

    autoware_adapi_v1_msgs::msg::OperationModeState msg;
    msg.stamp = this->now();
    msg.mode = mode_;
    msg.is_autoware_control_enabled = true;
    msg.is_in_transition = false;
    msg.is_autonomous_mode_available = true;
    msg.is_stop_mode_available = true;
    msg.is_local_mode_available = false;
    msg.is_remote_mode_available = false;

    publisher_->publish(msg);
  }

  double publish_rate_hz_{10.0};
  std::string mode_string_;
  uint8_t mode_{autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS};

  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace autoware::tensorrt_vad

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::CarlaOperationModePublisher)
