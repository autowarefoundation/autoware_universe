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

#ifndef CONVERTER_HPP_
#define CONVERTER_HPP_

#include "mode_interface.hpp"

#include <autoware/diagnostic_graph_utils/subscription.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_system_msgs/msg/hazard_status_stamped.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_system_msgs/msg/emergency_holding_state.hpp>

#include <unordered_map>
#include <unordered_set>

namespace autoware::hazard_status_converter
{

class Converter : public rclcpp::Node
{
public:
  explicit Converter(const rclcpp::NodeOptions & options);

private:
  using HazardStatusStamped = autoware_system_msgs::msg::HazardStatusStamped;
  using DiagGraph = autoware::diagnostic_graph_utils::DiagGraph;
  using DiagUnit = autoware::diagnostic_graph_utils::DiagUnit;
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  autoware::diagnostic_graph_utils::DiagGraphSubscription sub_graph_;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_;
  autoware_utils::InterProcessPollingSubscriber<tier4_system_msgs::msg::EmergencyHoldingState>
    sub_emergency_holding_{this, "~/input/emergency_holding"};
  autoware_utils::InterProcessPollingSubscriber<tier4_external_api_msgs::msg::Emergency>
    sub_external_emergency_{this, "~/input/external_emergency"};

  struct ModeSubgraph
  {
    DiagUnit * root;
    std::unordered_set<DiagUnit *> nodes;
  };

  ModeInterface mode_interface_;
  std::unordered_map<RootModeStatus::Mode, ModeSubgraph> mode_subgraphs_;
};

}  // namespace autoware::hazard_status_converter

#endif  // CONVERTER_HPP_
