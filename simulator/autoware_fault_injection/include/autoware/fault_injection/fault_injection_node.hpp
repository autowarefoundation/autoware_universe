// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_
#define AUTOWARE__FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_

#include "autoware/fault_injection/diagnostic_storage.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_simulation_msgs/msg/simulation_events.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simulator::fault_injection
{
using tier4_simulation_msgs::msg::SimulationEvents;
using diagnostic_msgs::msg::DiagnosticArray;

class FaultInjectionNode : public rclcpp::Node
{
public:
  explicit FaultInjectionNode(rclcpp::NodeOptions node_options);

private:
  // Subscribers
  void on_simulation_events(const SimulationEvents::ConstSharedPtr msg);
  void on_diagnostics(const DiagnosticArray::ConstSharedPtr msg);
  rclcpp::Subscription<SimulationEvents>::SharedPtr sub_simulation_events_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_diagnostics_;

  void publish_modified_diagnostics(const DiagnosticArray & msg);
  void apply_fault_injection(DiagnosticArray & msg);

  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diagnostics_;

  std::vector<DiagConfig> read_event_diag_list();

  std::unordered_map<std::string, std::string> event_to_diag_map_;
  std::unordered_map<std::string, int8_t> diag_override_levels_;
  DiagnosticArray last_diagnostics_;
  bool has_last_diagnostics_{false};
  std::mutex data_mutex_;
};

}  // namespace autoware::simulator::fault_injection

#endif  // AUTOWARE__FAULT_INJECTION__FAULT_INJECTION_NODE_HPP_
