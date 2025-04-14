// Copyright 2023 The Autoware Contributors
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

#include "converter.hpp"

#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::hazard_status_converter
{

Converter::Converter(const rclcpp::NodeOptions & options)
: Node("converter", options), mode_interface_(*this)
{
  using std::placeholders::_1;
  pub_hazard_ = create_publisher<HazardStatusStamped>("~/hazard_status", rclcpp::QoS(1));
  sub_graph_.register_create_callback(std::bind(&Converter::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&Converter::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void Converter::on_create(DiagGraph::ConstSharedPtr graph)
{
  const auto find_auto_mode_root = [](const DiagGraph & graph, const std::string & root_name) {
    for (const auto & unit : graph.units()) {
      if (unit->path() == root_name) return unit;
    }
    return static_cast<DiagUnit *>(nullptr);
  };

  const auto make_auto_mode_tree = [](DiagUnit * root) {
    std::unordered_set<DiagUnit *> result;
    std::unordered_set<DiagUnit *> buffer;
    if (root) {
      buffer.insert(root);
    }
    while (!buffer.empty()) {
      const auto unit = *buffer.begin();
      buffer.erase(buffer.begin());
      result.insert(unit);
      for (const auto & child : unit->children()) buffer.insert(child.unit);
    }
    return result;
  };

  std::vector<std::pair<RootModeStatus::Mode, std::string>> root_modes = {
    {RootModeStatus::Mode::AUTO, "/autoware/modes/autonomous"},
    {RootModeStatus::Mode::REMOTE, "/autoware/modes/remote"},
    {RootModeStatus::Mode::LOCAL, "/autoware/modes/local"},
  };
  for (const auto & [mode, root_name] : root_modes) {
    ModeSubgraph subgraph;
    subgraph.root = find_auto_mode_root(*graph, root_name);
    subgraph.nodes = make_auto_mode_tree(subgraph.root);
    mode_subgraphs_[mode] = subgraph;
  }
}

void Converter::on_update(DiagGraph::ConstSharedPtr graph)
{
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticLevel = DiagnosticStatus::_level_type;
  using HazardStatus = autoware_system_msgs::msg::HazardStatus;
  using HazardLevel = HazardStatus::_level_type;

  const auto get_hazard_level = [](DiagnosticLevel unit_level, DiagnosticLevel root_level) {
    // Convert the level according to the table below.
    // The Level other than auto mode is considered OK.
    // |-------|-------------------------------|
    // | Level |           Root level          |
    // |-------|-------------------------------|
    // |       | OK    | WARN  | ERROR | STALE |
    // | OK    | NF    | NF    | NF    | NF    |
    // | WARN  | SF    | LF    | LF    | LF    |
    // | ERROR | SF    | LF    | SPF   | SPF   |
    // | STALE | SF    | LF    | SPF   | SPF   |
    // |-------|-------------------------------|
    if (unit_level == DiagnosticStatus::OK) return HazardStatus::NO_FAULT;
    if (root_level == DiagnosticStatus::OK) return HazardStatus::SAFE_FAULT;
    if (unit_level == DiagnosticStatus::WARN) return HazardStatus::LATENT_FAULT;
    if (root_level == DiagnosticStatus::WARN) return HazardStatus::LATENT_FAULT;
    return HazardStatus::SINGLE_POINT_FAULT;
  };

  const auto get_system_level = [](const HazardStatus & status) {
    if (!status.diag_single_point_fault.empty()) return HazardStatus::SINGLE_POINT_FAULT;
    if (!status.diag_latent_fault.empty()) return HazardStatus::LATENT_FAULT;
    if (!status.diag_safe_fault.empty()) return HazardStatus::SAFE_FAULT;
    return HazardStatus::NO_FAULT;
  };

  const auto get_hazards_vector = [](HazardStatus & status, HazardLevel level) {
    if (level == HazardStatus::SINGLE_POINT_FAULT) return &status.diag_single_point_fault;
    if (level == HazardStatus::LATENT_FAULT) return &status.diag_latent_fault;
    if (level == HazardStatus::SAFE_FAULT) return &status.diag_safe_fault;
    if (level == HazardStatus::NO_FAULT) return &status.diag_no_fault;
    return static_cast<std::vector<DiagnosticStatus> *>(nullptr);
  };

  // Get subgraph related to the current operation mode.
  const auto root = mode_interface_.get_root();
  if (root.mode == RootModeStatus::Mode::UNKNOWN) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "Unknown root mode.");
    return;
  }
  const auto & subgraph = mode_subgraphs_.at(root.mode);
  if (!subgraph.root) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, "No current mode root.");
    return;
  }

  // Calculate hazard level from unit level and root level.
  HazardStatusStamped hazard;
  for (const auto & unit : graph->units()) {
    if (unit->path().empty()) continue;
    const bool is_ignored = root.ignore || !subgraph.nodes.count(unit);
    const auto root_level = is_ignored ? DiagnosticStatus::OK : subgraph.root->level();
    const auto unit_level = unit->level();
    if (auto diags = get_hazards_vector(hazard.status, get_hazard_level(unit_level, root_level))) {
      diags->push_back(unit->create_diagnostic_status());
    }
  }

  // Add external emergency to spf so that it is not ignored.
  if (const auto external_emergency = sub_external_emergency_.take_data()) {
    if (external_emergency->emergency) {
      DiagnosticStatus status;
      status.name = std::string(this->get_name()) + ": external_emergency";
      status.level = DiagnosticStatus::ERROR;
      hazard.status.diag_single_point_fault.push_back(status);
    }
  }

  hazard.stamp = graph->updated_stamp();
  hazard.status.level = get_system_level(hazard.status);
  hazard.status.emergency = hazard.status.level == HazardStatus::SINGLE_POINT_FAULT;

  const auto is_emergency_holding = sub_emergency_holding_.take_data();
  hazard.status.emergency_holding =
    is_emergency_holding == nullptr ? false : is_emergency_holding->is_holding;
  pub_hazard_->publish(hazard);
}

}  // namespace autoware::hazard_status_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::hazard_status_converter::Converter)
