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

#include "redundancy_adapi_switcher.hpp"

#include <algorithm>

namespace autoware::redundancy_adapi_switcher
{

RedundancyAdapiSwitcher::RedundancyAdapiSwitcher(const rclcpp::NodeOptions & node_options)
: Node("redundancy_adapi_switcher", node_options)
{
  const auto qos_struct = rclcpp::QoS(1).transient_local();
  const auto qos_status = rclcpp::QoS(5);

  is_main_ecu_ = declare_parameter<bool>("is_main_ecu");
  main_ecu_id_ = declare_parameter<uint8_t>("main_ecu_id");
  sub_ecu_id_  = declare_parameter<uint8_t>("sub_ecu_id");

  pub_mrm_state_   = create_publisher<MrmState>("~/output/mrm_state", qos_status);
  pub_diag_struct_ = create_publisher<DiagGraphStruct>("~/output/diag/struct", qos_struct);
  pub_diag_status_ = create_publisher<DiagGraphStatus>("~/output/diag/status", qos_status);

  sub_mrm_state_ = create_subscription<MrmState>(
    "~/input/mrm_state", qos_status,
    [this](const MrmState::ConstSharedPtr & msg) {
      if (is_active_output_master()) pub_mrm_state_->publish(*msg);
    });

  sub_diag_struct_ = create_subscription<DiagGraphStruct>(
    "~/input/diag/struct", qos_struct,
    [this](const DiagGraphStruct::ConstSharedPtr & msg) {
      pending_diag_struct_ = *msg;
    });

  sub_diag_status_ = create_subscription<DiagGraphStatus>(
    "~/input/diag/status", qos_status,
    [this](const DiagGraphStatus::ConstSharedPtr & msg) {
      if (!is_active_output_master()) return;
      pub_diag_status_->publish(*msg);
      if (pending_diag_struct_) {
        pub_diag_struct_->publish(*pending_diag_struct_);
        pending_diag_struct_.reset();
      }
    });

  sub_active_control_unit_ = create_subscription<ActiveControlUnit>(
    "~/input/active_control_unit", rclcpp::QoS(1).transient_local(),
    std::bind(&RedundancyAdapiSwitcher::on_active_control_unit, this, std::placeholders::_1));
}

void RedundancyAdapiSwitcher::on_active_control_unit(const ActiveControlUnit::ConstSharedPtr & msg)
{
  const bool changed = !last_active_ids_ || *last_active_ids_ != msg->ids;
  last_active_ids_ = msg->ids;

  if (msg->ids.empty()) {
    output_blocked_ = true;
    if (changed) RCLCPP_WARN(get_logger(), "active_control_unit is empty — output blocked.");
    return;
  }
  output_blocked_ = false;

  const auto contains = [&](uint8_t id) {
    return std::find(msg->ids.begin(), msg->ids.end(), id) != msg->ids.end();
  };
  const bool main_active = contains(main_ecu_id_);
  const bool sub_active  = contains(sub_ecu_id_);

  if (main_active && sub_active) {
    if (changed) RCLCPP_WARN(get_logger(), "Both Main and Sub ECU are active — keeping current master.");
    return;
  }

  if (!main_active && !sub_active) {
    // This ECU is not in the active list — assume self-fault and defer to the other ECU.
    use_main_ecu_ = !is_main_ecu_;
    if (changed) {
      RCLCPP_WARN(
        get_logger(), "%s ECU assumed faulty — deferring output to %s ECU.",
        is_main_ecu_ ? "Main" : "Sub",
        is_main_ecu_ ? "Sub" : "Main");
    }
    return;
  }

  // Exactly one ECU is active — switch output master to it if needed.
  const bool next_use_main = main_active;
  if (use_main_ecu_ != next_use_main) {
    use_main_ecu_ = next_use_main;
    RCLCPP_INFO(get_logger(), "Output master switched to %s ECU.", next_use_main ? "Main" : "Sub");
  }
}

}  // namespace autoware::redundancy_adapi_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_adapi_switcher::RedundancyAdapiSwitcher)
