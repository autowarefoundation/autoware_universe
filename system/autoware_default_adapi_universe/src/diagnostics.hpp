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

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/diag_graph_status.hpp>
#include <autoware_adapi_v1_msgs/msg/diag_graph_struct.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/srv/reset_diag_graph.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>
#include <tier4_system_msgs/srv/reset_diag_graph.hpp>

namespace autoware::default_adapi
{

class DiagnosticsNode : public rclcpp::Node
{
public:
  explicit DiagnosticsNode(const rclcpp::NodeOptions & options);

private:
  using InternalGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using InternalGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  using InternalNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
  using InternalNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;
  using InternalLeafStruct = tier4_system_msgs::msg::DiagLeafStruct;
  using InternalLeafStatus = tier4_system_msgs::msg::DiagLeafStatus;
  using InternalLinkStruct = tier4_system_msgs::msg::DiagLinkStruct;
  using InternalReset = tier4_system_msgs::srv::ResetDiagGraph;
  using ExternalGraphStruct = autoware_adapi_v1_msgs::msg::DiagGraphStruct;
  using ExternalGraphStatus = autoware_adapi_v1_msgs::msg::DiagGraphStatus;
  using ExternalNodeStruct = autoware_adapi_v1_msgs::msg::DiagNodeStruct;
  using ExternalNodeStatus = autoware_adapi_v1_msgs::msg::DiagNodeStatus;
  using ExternalLeafStruct = autoware_adapi_v1_msgs::msg::DiagLeafStruct;
  using ExternalLeafStatus = autoware_adapi_v1_msgs::msg::DiagLeafStatus;
  using ExternalLinkStruct = autoware_adapi_v1_msgs::msg::DiagLinkStruct;
  using ExternalKeyValue = autoware_adapi_v1_msgs::msg::KvString;
  using ExternalReset = autoware_adapi_v1_msgs::srv::ResetDiagGraph;
  using ExternalMrmState = autoware_adapi_v1_msgs::msg::MrmState;

  void on_struct(const InternalGraphStruct & internal);
  void on_status(const InternalGraphStatus & internal);
  void on_reset(
    const ExternalReset::Request::SharedPtr req, const ExternalReset::Response::SharedPtr res);

  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::Publisher<ExternalGraphStruct>::SharedPtr pub_struct_;
  rclcpp::Publisher<ExternalGraphStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<InternalGraphStruct>::SharedPtr sub_struct_;
  rclcpp::Subscription<InternalGraphStatus>::SharedPtr sub_status_;
  rclcpp::Service<ExternalReset>::SharedPtr srv_reset_;
  rclcpp::Client<InternalReset>::SharedPtr cli_reset_;
  rclcpp::Subscription<ExternalMrmState>::SharedPtr sub_mrm_state_;

  ExternalMrmState mrm_state_;
};

}  // namespace autoware::default_adapi

#endif  // DIAGNOSTICS_HPP_
