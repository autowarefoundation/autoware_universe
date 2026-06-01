//  Copyright 2025 The Autoware Contributors
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
#ifndef SUBSYSTEM_ADAPTER_HPP_
#define SUBSYSTEM_ADAPTER_HPP_

#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_system_msgs/srv/reset_redundancy_switcher.hpp>
#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::redundancy_switcher
{

using CommandModeRequest = tier4_system_msgs::msg::CommandModeRequest;
using CommandModeAvailability = tier4_system_msgs::msg::CommandModeAvailability;
using ActiveControlUnitMsg = tier4_system_msgs::msg::ActiveControlUnit;
using SetBool = std_srvs::srv::SetBool;
using ResetRedundancySwitcher = tier4_system_msgs::srv::ResetRedundancySwitcher;
using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;
using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

/**
 * @brief ROS adapter responsible for I/O with the lower-level system (Autoware / subsystem).
 *
 * Responsibilities:
 *   - Converts incoming ROS messages from Autoware into InputEvents and submits them to the Processor.
 *   - Handles UpdateActiveControlUnitCommand and publishes the ActiveControlUnit message.
 *   - Diagnostics are handled by DiagAdapter (aggregated status) and SwitcherAdapter (switcher-specific diag).
 */
class SubSystemAdapter : public IAdapterPlugin
{
public:
  SubSystemAdapter() = default;
  ~SubSystemAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  void submit_event(const InputEvent & event);

  void on_command_mode_request(const CommandModeRequest::ConstSharedPtr msg);
  void on_command_mode_availability(const CommandModeAvailability::ConstSharedPtr msg);
  void on_set_initializing(
    const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response);
  void on_reset_request(
    const ResetRedundancySwitcher::Request::SharedPtr request,
    ResetRedundancySwitcher::Response::SharedPtr response);
  void on_velocity_report(const VelocityReport::ConstSharedPtr msg);
  void on_control_mode_report(const ControlModeReport::ConstSharedPtr msg);

  void check_sub_ecu_error(const CommandModeAvailability::ConstSharedPtr msg);

  void check_availability_timeout(const CommandModeAvailability::ConstSharedPtr msg);

  void send_active_control_unit(const UpdateActiveControlUnitCommand & command);

  rclcpp::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;

  // Parameters
  bool is_main_ecu_{true};
  double availability_timeout_milli_{1.0};

  // State
  std::optional<CommandModeRequest> last_command_mode_request_;
  std::optional<ActiveControlUnitMsg::_ids_type> last_active_control_unit_ids_;
  std::optional<rclcpp::Time> stamp_another_ecu_availability_;
  bool is_another_ecu_availability_timeout_{false};
  mutable std::mutex state_mutex_;

  // Publishers
  rclcpp::Publisher<ActiveControlUnitMsg>::SharedPtr pub_active_control_unit_;

  // Subscribers
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;
  rclcpp::Subscription<CommandModeRequest>::SharedPtr sub_command_mode_request_;
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_command_mode_availability_;

  // Services
  rclcpp::Service<SetBool>::SharedPtr srv_set_initializing_;
  rclcpp::Service<ResetRedundancySwitcher>::SharedPtr srv_reset_;
};

}  // namespace autoware::redundancy_switcher
#endif  // SUBSYSTEM_ADAPTER_HPP_
