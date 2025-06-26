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

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class AggregatorNode : public rclcpp::Node
{
public:
  explicit AggregatorNode(const rclcpp::NodeOptions & options);
  ~AggregatorNode();

private:
  std::unique_ptr<Graph> graph_;
  std::unique_ptr<CommandModeMapping> availability_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr sub_input_;
  rclcpp::Publisher<DiagGraphStruct>::SharedPtr pub_struct_;
  rclcpp::Publisher<DiagGraphStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_unknown_;
  rclcpp::Service<ResetDiagGraph>::SharedPtr srv_reset_;

  void on_timer();
  void on_diag(const DiagnosticArray & msg);
  void on_reset(
    const ResetDiagGraph::Request::SharedPtr request,
    const ResetDiagGraph::Response::SharedPtr response);
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // NODE__AGGREGATOR_HPP_
