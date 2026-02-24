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

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <memory>

namespace autoware::trajectory_adapter
{

using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_planning_msgs::msg::Trajectory;

class TrajectoryAdapterNode : public rclcpp::Node
{
public:
  explicit TrajectoryAdapterNode(const rclcpp::NodeOptions & node_options);

private:
  void process(const ScoredCandidateTrajectories::ConstSharedPtr msg);

  void publish_planning_factor(const Trajectory & trajectory);

  rclcpp::Subscription<ScoredCandidateTrajectories>::SharedPtr sub_trajectories_;

  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
};

}  // namespace autoware::trajectory_adapter

#endif  // TRAJECTORY_ADAPTER_NODE_HPP_
