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

#include <memory>
#include <sstream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

AggregatorNode::AggregatorNode(const rclcpp::NodeOptions & options) : Node("aggregator", options)
{
  const auto stamp = now();

  // Init diagnostics graph.
  {
    const auto graph_file = declare_parameter<std::string>("graph_file");
    std::ostringstream id;
    id << std::hex << stamp.nanoseconds();
    graph_ = std::make_unique<Graph>(graph_file, id.str(), nullptr);
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
    sub_input_ = create_subscription<DiagnosticArray>("/diagnostics", qos_input, callback);
    pub_struct_ = create_publisher<DiagGraphStruct>("~/struct", qos_struct);
    pub_status_ = create_publisher<DiagGraphStatus>("~/status", qos_status);
    pub_unknown_ = create_publisher<DiagnosticArray>("~/unknowns", qos_unknown);
    srv_reset_ = create_service<ResetDiagGraph>(
      "~/reset",
      std::bind(&AggregatorNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));
    srv_set_initializing_ = create_service<SetBool>(
      "~/set_initializing",
      std::bind(
        &AggregatorNode::on_set_initializing, this, std::placeholders::_1, std::placeholders::_2));

    const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
    timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  }

  // Send structure topic once.
  pub_struct_->publish(graph_->create_struct_msg(stamp));
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
  pub_status_->publish(graph_->create_status_msg(stamp));
  pub_unknown_->publish(graph_->create_unknown_msg(stamp));

  // Update plugins.
  if (availability_) availability_->update(stamp);
}

void AggregatorNode::on_diag(const DiagnosticArray & msg)
{
  graph_->update(now(), msg);
}

void AggregatorNode::on_reset(
  const ResetDiagGraph::Request::SharedPtr, const ResetDiagGraph::Response::SharedPtr response)
{
  graph_->reset();
  response->status.success = true;
}

void AggregatorNode::on_set_initializing(
  const SetBool::Request::SharedPtr request, const SetBool::Response::SharedPtr response)
{
  graph_->set_initializing(request->data);
  response->success = true;
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::AggregatorNode)
