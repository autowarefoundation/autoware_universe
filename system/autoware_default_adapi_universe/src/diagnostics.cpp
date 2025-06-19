// Copyright 2024 The Autoware Contributors
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

#include "diagnostics.hpp"

#include <memory>
#include <unordered_map>

namespace autoware::default_adapi
{

DiagnosticsNode::DiagnosticsNode(const rclcpp::NodeOptions & options) : Node("diagnostics", options)
{
  using std::placeholders::_1;
  const auto qos_struct = rclcpp::QoS(1).transient_local();
  const auto qos_status = rclcpp::QoS(1).best_effort();

  pub_struct_ = create_publisher<ExternalGraphStruct>("/api/system/diagnostics/struct", qos_struct);
  pub_status_ = create_publisher<ExternalGraphStatus>("/api/system/diagnostics/status", qos_status);

  sub_struct_ = create_subscription<InternalGraphStruct>(
    "/diagnostics_graph/struct", qos_struct, std::bind(&DiagnosticsNode::on_struct, this, _1));
  sub_status_ = create_subscription<InternalGraphStatus>(
    "/diagnostics_graph/status", qos_status, std::bind(&DiagnosticsNode::on_status, this, _1));

  std::unordered_map<DiagUnit *, size_t> unit_indices_;
  for (size_t i = 0; i < units.size(); ++i) {
    unit_indices_[units[i]] = i;
  }

  autoware_adapi_v1_msgs::msg::DiagGraphStruct msg;
  msg.stamp = graph->created_stamp();
  msg.id = graph->id();
  msg.nodes.reserve(units.size());
  msg.links.reserve(links.size());
  for (const auto & unit : units) {
    msg.nodes.emplace_back();
    msg.nodes.back().path = unit->path_or_name();
  }
  for (const auto & link : links) {
    msg.links.emplace_back();
    msg.links.back().parent = unit_indices_.at(link->parent());
    msg.links.back().child = unit_indices_.at(link->child());
  }
  pub_struct_->publish(msg);
}

void DiagnosticsNode::on_struct(const InternalGraphStruct & internal)
{
  const auto convert_node = [](const InternalNodeStruct & internal) {
    ExternalNodeStruct external;
    external.path = internal.path;
    return external;
  };
  const auto convert_diag = [](const InternalLeafStruct & internal) {
    ExternalLeafStruct external;
    external.name = internal.name;
    return external;
  };
  const auto convert_link = [](const InternalLinkStruct & internal) {
    ExternalLinkStruct external;
    external.parent = internal.parent;
    external.child = internal.child;
    return external;
  };

  ExternalGraphStruct external;
  external.nodes.reserve(internal.nodes.size());
  external.diags.reserve(internal.diags.size());
  external.links.reserve(internal.links.size());
  external.stamp = internal.stamp;
  external.id = internal.id;
  for (const auto & node : internal.nodes) external.nodes.push_back(convert_node(node));
  for (const auto & diag : internal.diags) external.diags.push_back(convert_diag(diag));
  for (const auto & link : internal.links) external.links.push_back(convert_link(link));
  pub_struct_->publish(external);
}

void DiagnosticsNode::on_status(const InternalGraphStatus & internal)
{
  const auto convert_node = [](const InternalNodeStatus & internal) {
    ExternalNodeStatus external;
    external.level = internal.level;
    external.input_level = internal.input_level;
    external.latch_level = internal.latch_level;
    external.is_dependent = internal.is_dependent;
    return external;
  };
  const auto convert_diag = [](const InternalLeafStatus & internal) {
    ExternalLeafStatus external;
    external.level = internal.level;
    external.input_level = internal.input_level;
    external.message = internal.message;
    external.hardware_id = internal.hardware_id;
    for (const auto & value : internal.values) {
      ExternalKeyValue kv;
      kv.key = value.key;
      kv.value = value.value;
      external.values.push_back(kv);
    }
    return external;
  };

  ExternalGraphStatus external;
  external.nodes.reserve(internal.nodes.size());
  external.diags.reserve(internal.diags.size());
  external.stamp = internal.stamp;
  external.id = internal.id;
  for (const auto & node : internal.nodes) external.nodes.push_back(convert_node(node));
  for (const auto & diag : internal.diags) external.diags.push_back(convert_diag(diag));
  pub_status_->publish(external);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::DiagnosticsNode)
