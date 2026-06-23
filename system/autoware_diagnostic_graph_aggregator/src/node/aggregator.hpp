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

#ifndef NODE__AGGREGATOR_HPP_
#define NODE__AGGREGATOR_HPP_

#include "command_mode_mapping.hpp"
#include "graph/graph.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <tier4_system_msgs/srv/reset_diag_graph.hpp>
#include <tier4_system_msgs/srv/set_diag_graph_override.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class AggregatorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit AggregatorNode(const rclcpp::NodeOptions & options);
  ~AggregatorNode();

private:
  std::unique_ptr<Graph> graph_;
  std::unique_ptr<CommandModeMapping> availability_;
  bool allow_override_;

  using ResetDiagGraph = tier4_system_msgs::srv::ResetDiagGraph;
  using SetInitializing = std_srvs::srv::SetBool;
  using SetOverride = tier4_system_msgs::srv::SetDiagGraphOverride;
  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_SUBSCRIPTION_PTR(DiagnosticArray) sub_input_;
  AUTOWARE_PUBLISHER_PTR(DiagGraphStruct) pub_struct_;
  AUTOWARE_PUBLISHER_PTR(DiagGraphStatus) pub_status_;
  AUTOWARE_PUBLISHER_PTR(DiagnosticArray) pub_unknown_;
  AUTOWARE_SERVICE_PTR(ResetDiagGraph) srv_reset_;
  AUTOWARE_SERVICE_PTR(SetInitializing) srv_set_initializing_;
  AUTOWARE_SERVICE_PTR(SetOverride) srv_set_override_;

  void on_timer();
  void on_diag(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DiagnosticArray) & msg);
  void on_reset(
    AUTOWARE_SERVICE_REQUEST_CONST_PTR(ResetDiagGraph) request,
    AUTOWARE_SERVICE_RESPONSE_PTR(ResetDiagGraph) response);
  void on_set_initializing(
    AUTOWARE_SERVICE_REQUEST_CONST_PTR(SetInitializing) request,
    AUTOWARE_SERVICE_RESPONSE_PTR(SetInitializing) response);
  void on_set_override(
    AUTOWARE_SERVICE_REQUEST_CONST_PTR(SetOverride) request,
    AUTOWARE_SERVICE_RESPONSE_PTR(SetOverride) response);
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // NODE__AGGREGATOR_HPP_
