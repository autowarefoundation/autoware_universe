// Copyright 2025 TIER IV, Inc.
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

#ifndef TRAJECTORY_ADAPTER_NODE_HPP_
#define TRAJECTORY_ADAPTER_NODE_HPP_

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <memory>

namespace autoware::trajectory_adapter
{

using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

class TrajectoryAdapterNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit TrajectoryAdapterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(ScoredCandidateTrajectories) & msg);

  AUTOWARE_SUBSCRIPTION_PTR(ScoredCandidateTrajectories) sub_trajectories_;

  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_trajectory_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) pub_turn_indicators_;

  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail)
  debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_adapter

#endif  // TRAJECTORY_ADAPTER_NODE_HPP_
