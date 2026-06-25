// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mrm_emergency_stop_operator_core.hpp"

#include <autoware_utils/ros/update_param.hpp>

#include <vector>

namespace autoware::mrm_emergency_stop_operator
{

MrmEmergencyStopOperator::MrmEmergencyStopOperator(const rclcpp::NodeOptions & node_options)
: Node("mrm_emergency_stop_operator", node_options)
{
  // Parameter
  params_.update_rate = declare_parameter<int>("update_rate");
  params_.target_acceleration = declare_parameter<double>("target_acceleration");
  params_.target_jerk = declare_parameter<double>("target_jerk");

  // Subscriber
  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control/control_cmd", 1,
    std::bind(&MrmEmergencyStopOperator::onControlCommand, this, std::placeholders::_1));
  sub_driving_mode_request_ = create_subscription<DrivingModeRequest>(
    "~/input/driving_mode_request", 1,
    std::bind(&MrmEmergencyStopOperator::onDrivingModeRequest, this, std::placeholders::_1));
  sub_driving_mode_info_ = create_subscription<DrivingModeInfo>(
    "~/input/driving_mode_info", rclcpp::QoS(1).transient_local(),
    std::bind(&MrmEmergencyStopOperator::onDrivingModeInfo, this, std::placeholders::_1));

  // Server
  service_operation_ = create_service<OperateMrm>(
    "~/input/mrm/emergency_stop/operate", std::bind(
                                            &MrmEmergencyStopOperator::operateEmergencyStop, this,
                                            std::placeholders::_1, std::placeholders::_2));

  // Publisher
  pub_status_ = create_publisher<MrmBehaviorStatus>("~/output/mrm/emergency_stop/status", 1);
  pub_control_cmd_ = create_publisher<Control>("~/output/mrm/emergency_stop/control_cmd", 1);
  pub_mrm_state_ = create_publisher<DrivingModeMrmState>("~/output/mrm_state", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmEmergencyStopOperator::onTimer, this));

  // Initialize
  status_.state = MrmBehaviorStatus::AVAILABLE;
  is_prev_control_cmd_subscribed_ = false;

  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&MrmEmergencyStopOperator::onParameter, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MrmEmergencyStopOperator::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  update_param<double>(parameters, "target_acceleration", params_.target_acceleration);
  update_param<double>(parameters, "target_jerk", params_.target_jerk);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void MrmEmergencyStopOperator::onControlCommand(Control::ConstSharedPtr msg)
{
  if (status_.state != MrmBehaviorStatus::OPERATING) {
    prev_control_cmd_ = *msg;
    is_prev_control_cmd_subscribed_ = true;
  }
}

void MrmEmergencyStopOperator::onDrivingModeRequest(DrivingModeRequest::ConstSharedPtr msg)
{
  if (msg->mode == driving_mode_id_) {
    status_.state = MrmBehaviorStatus::OPERATING;
  } else {
    status_.state = MrmBehaviorStatus::AVAILABLE;
  }
}

void MrmEmergencyStopOperator::onDrivingModeInfo(DrivingModeInfo::ConstSharedPtr msg)
{
  for (const auto & item : msg->items) {
    if (item.name == "emergency_stop") {
      driving_mode_id_ = item.mode;
      break;
    }
  }
}

void MrmEmergencyStopOperator::operateEmergencyStop(
  const OperateMrm::Request::SharedPtr request, const OperateMrm::Response::SharedPtr response)
{
  if (request->operate == true) {
    status_.state = MrmBehaviorStatus::OPERATING;
    response->response.success = true;
  } else {
    status_.state = MrmBehaviorStatus::AVAILABLE;
    response->response.success = true;
  }
}

void MrmEmergencyStopOperator::publishStatus() const
{
  auto status = status_;
  status.stamp = this->now();
  pub_status_->publish(status);
}

void MrmEmergencyStopOperator::publishMrmState() const
{
  // See the following page for the definition of the MRM state.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture-v1/interfaces/ad-api/features/fail-safe/#mrm-state
  using tier4_system_msgs::msg::DrivingModeMrmStateItem;
  const auto convert_mrm_state = [](const uint8_t state) {
    // clang-format off
    switch (state) {
      case MrmBehaviorStatus::AVAILABLE: return DrivingModeMrmStateItem::NORMAL;
      case MrmBehaviorStatus::OPERATING: return DrivingModeMrmStateItem::OPERATING;
      default:                           return DrivingModeMrmStateItem::UNKNOWN;
    }
    // clang-format on
  };

  if (!driving_mode_id_) {
    return;
  }

  DrivingModeMrmStateItem item;
  item.mode = driving_mode_id_.value();
  item.state = convert_mrm_state(status_.state);

  DrivingModeMrmState msg;
  msg.stamp = this->now();
  msg.items = {item};
  pub_mrm_state_->publish(msg);
}

void MrmEmergencyStopOperator::publishControlCommand(const Control & command) const
{
  pub_control_cmd_->publish(command);
}

void MrmEmergencyStopOperator::onTimer()
{
  if (status_.state == MrmBehaviorStatus::OPERATING) {
    auto control_cmd = calcTargetAcceleration(prev_control_cmd_);
    publishControlCommand(control_cmd);
    prev_control_cmd_ = control_cmd;
  } else {
    publishControlCommand(prev_control_cmd_);
  }
  publishStatus();
  publishMrmState();
}

Control MrmEmergencyStopOperator::calcTargetAcceleration(const Control & prev_control_cmd) const
{
  auto control_cmd = Control();

  if (!is_prev_control_cmd_subscribed_) {
    control_cmd.stamp = this->now();
    control_cmd.longitudinal.stamp = this->now();
    control_cmd.longitudinal.velocity = 0.0;
    control_cmd.longitudinal.acceleration = static_cast<float>(params_.target_acceleration);
    control_cmd.longitudinal.jerk = 0.0;
    control_cmd.lateral.stamp = this->now();
    control_cmd.lateral.steering_tire_angle = 0.0;
    control_cmd.lateral.steering_tire_rotation_rate = 0.0;
    return control_cmd;
  }

  control_cmd = prev_control_cmd;
  const auto dt = (this->now() - prev_control_cmd.stamp).seconds();

  control_cmd.stamp = this->now();
  control_cmd.longitudinal.stamp = this->now();
  control_cmd.longitudinal.velocity = static_cast<float>(std::max(
    prev_control_cmd.longitudinal.velocity + prev_control_cmd.longitudinal.acceleration * dt, 0.0));
  control_cmd.longitudinal.acceleration = static_cast<float>(std::max(
    prev_control_cmd.longitudinal.acceleration + params_.target_jerk * dt,
    params_.target_acceleration));
  if (prev_control_cmd.longitudinal.acceleration == params_.target_acceleration) {
    control_cmd.longitudinal.jerk = 0.0;
  } else {
    control_cmd.longitudinal.jerk = static_cast<float>(params_.target_jerk);
  }

  return control_cmd;
}

}  // namespace autoware::mrm_emergency_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mrm_emergency_stop_operator::MrmEmergencyStopOperator)
