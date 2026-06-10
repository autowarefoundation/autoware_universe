// Copyright 2025 The Autoware Contributors
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

#ifndef AUTOWARE_MRM_RESET_MANAGER__MRM_RESET_MANAGER_HPP_
#define AUTOWARE_MRM_RESET_MANAGER__MRM_RESET_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_external_api_msgs/srv/reset_mrm.hpp>
#include <tier4_system_msgs/srv/reset_diag_graph.hpp>
#include <tier4_system_msgs/srv/reset_redundancy_switcher.hpp>

#include <mutex>
#include <string>

namespace autoware::mrm_reset_manager
{

class MrmResetManager : public rclcpp::Node
{
public:
  explicit MrmResetManager(const rclcpp::NodeOptions & options);

private:
  using LocalizationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  using OperationMode = autoware_adapi_v1_msgs::msg::OperationModeState;
  using SetBool = std_srvs::srv::SetBool;
  using ResetMrm = tier4_external_api_msgs::srv::ResetMrm;
  using ResetDiagGraph = tier4_system_msgs::srv::ResetDiagGraph;
  using ResetRedundancySwitcher = tier4_system_msgs::srv::ResetRedundancySwitcher;

  enum class InitState {
    WAIT_SERVICES_READY,
    SET_AGGREGATOR_INIT,
    RESET_SWITCHER,
    SET_SWITCHER_IFACE_INIT,
    DONE,
  };

  void on_reset_mrm(
    const ResetMrm::Request::SharedPtr request,
    ResetMrm::Response::SharedPtr response);
  void advance_init_state();
  void apply_ready_state();

  bool set_aggregator_initializing(bool initializing);
  bool set_redundancy_switcher_interface_initializing(bool initializing);
  bool call_reset_redundancy_switcher(std::string & message);
  bool call_reset_redundancy_switcher();
  bool call_reset_diag_graph(std::string & message);
  bool call_set_bool(
    const rclcpp::Client<SetBool>::SharedPtr & client,
    const SetBool::Request::SharedPtr & request,
    const char * label,
    std::string & message);

  bool is_autoware_ready() const;
  bool is_initializing() const;

  int service_timeout_ms_;
  bool is_redundant_;
  bool enable_autoware_ready_actions_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cli_set_aggregator_initializing_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cli_set_redundancy_switcher_interface_initializing_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cli_reset_redundancy_switcher_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cli_reset_diag_graph_callback_group_;

  rclcpp::Service<ResetMrm>::SharedPtr srv_reset_mrm_;

  rclcpp::Subscription<LocalizationState>::SharedPtr sub_localization_initialization_state_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<OperationMode>::SharedPtr sub_operation_mode_state_;

  rclcpp::Client<SetBool>::SharedPtr cli_set_aggregator_initializing_;
  rclcpp::Client<SetBool>::SharedPtr cli_set_redundancy_switcher_interface_initializing_;
  rclcpp::Client<ResetRedundancySwitcher>::SharedPtr cli_reset_redundancy_switcher_;
  rclcpp::Client<ResetDiagGraph>::SharedPtr cli_reset_diag_graph_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr reset_redundancy_switcher_timer_;

  mutable std::mutex state_mutex_;
  InitState init_state_{InitState::WAIT_SERVICES_READY};
  bool is_aggregator_initializing_{false};
  bool is_redundancy_switcher_interface_initializing_{false};
  LocalizationState::ConstSharedPtr localization_initialization_state_ptr_;
  RouteState::ConstSharedPtr route_state_ptr_;
  OperationMode::ConstSharedPtr operation_mode_state_ptr_;
};

}  // namespace autoware::mrm_reset_manager

#endif  // AUTOWARE_MRM_RESET_MANAGER__MRM_RESET_MANAGER_HPP_
