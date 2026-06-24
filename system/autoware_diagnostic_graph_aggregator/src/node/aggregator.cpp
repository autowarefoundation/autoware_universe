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

#include "aggregator.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

AggregatorNode::AggregatorNode(const rclcpp::NodeOptions & options) : Node("aggregator", options)
{
  allow_override_ = declare_parameter<bool>("allow_override_for_debugging");

  const auto stamp = now();

  // Init diagnostics graph.
  {
    const auto graph_file = declare_parameter<std::string>("graph_file");

    // Parse graph variables from YAML map format (e.g., "{key1: value1, key2: value2}").
    auto variables = std::make_shared<std::unordered_map<std::string, std::string>>();
    const auto vars_text = declare_parameter<std::string>("graph_vars", "");
    if (!vars_text.empty()) {
      const auto vars_yaml = YAML::Load(vars_text);
      for (const auto & var : vars_yaml) {
        variables->emplace(var.first.as<std::string>(), var.second.as<std::string>());
      }
    }

    std::ostringstream id;
    id << std::hex << stamp.nanoseconds();
    graph_ = std::make_unique<Graph>(graph_file, id.str(), nullptr, variables);
    graph_->set_initializing(declare_parameter<bool>("initial_latch_suppression"));
  }

  // Init plugins.
  if (declare_parameter<bool>("use_command_mode_mappings")) {
    availability_ = std::make_unique<CommandModeMapping>(*this, *graph_);
  }

  // Init ros interface.
  {
    const auto qos_input = rclcpp::QoS(declare_parameter<int64_t>("input_qos_depth"));
    const auto qos_unknown = rclcpp::QoS(1);
    const auto qos_struct = rclcpp::QoS(1).transient_local();
    const auto qos_status = rclcpp::QoS(declare_parameter<int64_t>("graph_qos_depth"));
    const auto callback = std::bind(&AggregatorNode::on_diag, this, std::placeholders::_1);
    sub_input_ = create_subscription<DiagnosticArray>("~/diagnostics", qos_input, callback);
    pub_struct_ = create_publisher<DiagGraphStruct>("~/struct", qos_struct);
    pub_status_ = create_publisher<DiagGraphStatus>("~/status", qos_status);
    pub_unknown_ = create_publisher<DiagnosticArray>("~/unknowns", qos_unknown);
    srv_reset_ = create_service<ResetDiagGraph>(
      "~/reset",
      std::bind(&AggregatorNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_initializing_ = create_service<SetInitializing>(
      "~/set_initializing",
      std::bind(
        &AggregatorNode::on_set_initializing, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_override_ = create_service<SetOverride>(
      "~/set_override",
      std::bind(
        &AggregatorNode::on_set_override, this, std::placeholders::_1, std::placeholders::_2));

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = autoware::agnocast_wrapper::create_timer(
      this, get_clock(), rate.period(), [this]() { on_timer(); });
  }

  // Send structure topic once.
  {
    auto struct_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_struct_);
    *struct_msg = graph_->create_struct_msg(stamp);
    pub_struct_->publish(std::move(struct_msg));
  }
}

AggregatorNode::~AggregatorNode()
{
  // For unique_ptr members.
}

void AggregatorNode::on_timer()
{
  // Check timeout of diag units.
  const auto stamp = now();
  graph_->update(stamp);

  // Publish status.
  {
    auto status_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_status_);
    *status_msg = graph_->create_status_msg(stamp);
    pub_status_->publish(std::move(status_msg));
  }
  {
    auto unknown_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_unknown_);
    *unknown_msg = graph_->create_unknown_msg(stamp);
    pub_unknown_->publish(std::move(unknown_msg));
  }

  // Update plugins.
  if (availability_) availability_->update(stamp);
}

void AggregatorNode::on_diag(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DiagnosticArray) & msg)
{
  graph_->update(now(), *msg);
}

void AggregatorNode::on_reset(
  AUTOWARE_SERVER_REQUEST_PTR(ResetDiagGraph),
  AUTOWARE_SERVER_RESPONSE_PTR(ResetDiagGraph) response)
{
  graph_->reset();
  response->status.success = true;
}

void AggregatorNode::on_set_initializing(
  AUTOWARE_SERVER_REQUEST_PTR(SetInitializing) request,
  AUTOWARE_SERVER_RESPONSE_PTR(SetInitializing) response)
{
  graph_->set_initializing(request->data);
  response->success = true;
}

void AggregatorNode::on_set_override(
  AUTOWARE_SERVER_REQUEST_PTR(SetOverride) request,
  AUTOWARE_SERVER_RESPONSE_PTR(SetOverride) response)
{
  if (!allow_override_) {
    response->status.success = false;
    response->status.message = "override not allowed";
    return;
  }
  if (request->level == SetOverride::Request::CLEAR) {
    const auto error = graph_->set_override(request->path, std::nullopt);
    response->status.success = error.empty();
    response->status.message = error;
    return;
  }
  if (request->level <= DiagnosticStatus::ERROR) {
    const auto error = graph_->set_override(request->path, request->level);
    response->status.success = error.empty();
    response->status.message = error;
    return;
  }
  response->status.success = false;
  response->status.message = "invalid level: " + std::to_string(request->level);
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::AggregatorNode)
