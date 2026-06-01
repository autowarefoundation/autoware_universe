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

#ifndef AUTOWARE_REDUNDANCY_ADAPI_SWITCHER__REDUNDANCY_ADAPI_SWITCHER_HPP_
#define AUTOWARE_REDUNDANCY_ADAPI_SWITCHER__REDUNDANCY_ADAPI_SWITCHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::redundancy_adapi_switcher
{

using autoware_adapi_v1_msgs::msg::MrmState;
using tier4_system_msgs::msg::ActiveControlUnit;
using tier4_system_msgs::msg::DiagGraphStatus;
using tier4_system_msgs::msg::DiagGraphStruct;

class RedundancyAdapiSwitcher : public rclcpp::Node
{
public:
  explicit RedundancyAdapiSwitcher(const rclcpp::NodeOptions & node_options);

private:
  // Returns true when this node should publish outputs: this ECU is the selected output
  // master and active_control_unit is in a valid (non-empty) state.
  bool is_active_output_master() const { return use_main_ecu_ == is_main_ecu_ && !output_blocked_; }

  void on_active_control_unit(const ActiveControlUnit::ConstSharedPtr & msg);

  // Parameters
  bool is_main_ecu_;
  uint8_t main_ecu_id_;
  uint8_t sub_ecu_id_;

  // State
  bool use_main_ecu_{true};    // true: Main ECU is the output master; false: Sub ECU
  bool output_blocked_{false}; // true while active_control_unit reports an empty ID list
  std::optional<std::vector<uint8_t>> last_active_ids_;

  // DiagGraphStruct is transient_local — buffer the latest value so it can be
  // re-published together with the next DiagGraphStatus when the master switches.
  std::optional<DiagGraphStruct> pending_diag_struct_;

  // Subscribers
  rclcpp::Subscription<MrmState>::SharedPtr sub_mrm_state_;
  rclcpp::Subscription<DiagGraphStruct>::SharedPtr sub_diag_struct_;
  rclcpp::Subscription<DiagGraphStatus>::SharedPtr sub_diag_status_;
  rclcpp::Subscription<ActiveControlUnit>::SharedPtr sub_active_control_unit_;

  // Publishers
  rclcpp::Publisher<MrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<DiagGraphStruct>::SharedPtr pub_diag_struct_;
  rclcpp::Publisher<DiagGraphStatus>::SharedPtr pub_diag_status_;
};

}  // namespace autoware::redundancy_adapi_switcher

#endif  // AUTOWARE_REDUNDANCY_ADAPI_SWITCHER__REDUNDANCY_ADAPI_SWITCHER_HPP_
