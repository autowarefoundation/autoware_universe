// Copyright 2021 Tier IV, Inc.
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

#include "autoware/fault_injection/fault_injection_node.hpp"

#include <diagnostic_aggregator/status_item.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::simulator::fault_injection
{
std::vector<std::string> split(const std::string & str, const char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

#ifdef ROS_DISTRO_GALACTIC
using rosidl_generator_traits::to_yaml;
#endif

FaultInjectionNode::FaultInjectionNode(rclcpp::NodeOptions node_options)
: Node("fault_injection", node_options.automatically_declare_parameters_from_overrides(true))
{
  using std::placeholders::_1;

  // Subscriber
  sub_simulation_events_ = this->create_subscription<SimulationEvents>(
    "~/input/simulation_events", rclcpp::QoS{rclcpp::KeepLast(10)},
    std::bind(&FaultInjectionNode::on_simulation_events, this, _1));

  rclcpp::SubscriptionOptions sub_options;
  sub_options.ignore_local_publications = true;
  sub_diagnostics_ = this->create_subscription<DiagnosticArray>(
    "~/input/diagnostics", rclcpp::SystemDefaultsQoS().keep_last(1000),
    std::bind(&FaultInjectionNode::on_diagnostics, this, _1), sub_options);

  pub_diagnostics_ = this->create_publisher<DiagnosticArray>(
    "~/output/diagnostics", rclcpp::SystemDefaultsQoS().keep_last(1000));

  // Load all config
  for (const auto & diag : read_event_diag_list()) {
    event_to_diag_map_[diag.sim_name] = diag.diag_name;
  }
}

void FaultInjectionNode::on_simulation_events(const SimulationEvents::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received data: %s", to_yaml(*msg).c_str());
  DiagnosticArray updated;
  bool should_publish = false;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto & event : msg->fault_injection_events) {
      const auto mapped = event_to_diag_map_.find(event.name);
      if (mapped == event_to_diag_map_.end()) {
        continue;
      }
      diag_override_levels_[mapped->second] = event.level;
    }
    if (has_last_diagnostics_) {
      updated = last_diagnostics_;
      apply_fault_injection(updated);
      should_publish = true;
    }
  }

  if (should_publish) {
    publish_modified_diagnostics(updated);
  }
}

void FaultInjectionNode::on_diagnostics(const DiagnosticArray::ConstSharedPtr msg)
{
  DiagnosticArray updated;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_diagnostics_ = *msg;
    has_last_diagnostics_ = true;
    updated = *msg;
    apply_fault_injection(updated);
  }

  publish_modified_diagnostics(updated);
}

void FaultInjectionNode::publish_modified_diagnostics(const DiagnosticArray & msg)
{
  DiagnosticArray output = msg;
  output.header.stamp = this->now();
  pub_diagnostics_->publish(output);
}

void FaultInjectionNode::apply_fault_injection(DiagnosticArray & msg)
{
  for (auto & status : msg.status) {
    const auto override_it = diag_override_levels_.find(status.name);
    if (override_it == diag_override_levels_.end()) {
      continue;
    }
    status.level = diagnostic_aggregator::valToLevel(override_it->second);
    status.message = diagnostic_aggregator::valToMsg(override_it->second);
  }
}

std::vector<DiagConfig> FaultInjectionNode::read_event_diag_list()
{
  // Expected parameter name is "event_diag_list.param_name".
  // In this case, depth is 2.
  const auto param_name_list = list_parameters({"event_diag_list"}, 2);

  std::vector<DiagConfig> diag_configs;
  // NOTE: param_name_list.prefixes returns {"event_diag_list"}
  //       and param_name_list.names returns {"event_diag_list.param_name"}
  for (const auto & param_name : param_name_list.names) {
    // Trim parameter prefix
    const auto sim_name = split(param_name, '.').back();
    const auto diag_name = get_parameter(param_name).as_string();
    diag_configs.emplace_back(DiagConfig{sim_name, diag_name});
    RCLCPP_DEBUG(get_logger(), "Parameter: %s, value: %s", sim_name.c_str(), diag_name.c_str());
  }

  return diag_configs;
}
}  // namespace autoware::simulator::fault_injection

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::simulator::fault_injection::FaultInjectionNode)
