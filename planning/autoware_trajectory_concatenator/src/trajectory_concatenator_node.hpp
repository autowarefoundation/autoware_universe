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

#ifndef TRAJECTORY_CONCATENATOR_NODE_HPP_
#define TRAJECTORY_CONCATENATOR_NODE_HPP_

#include "trajectory_concatenator_structs.hpp"

#include <autoware_trajectory_concatenator/autoware_trajectory_concatenator_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::trajectory_concatenator
{

using namespace std::literals::chrono_literals;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
class TrajectoryConcatenatorNode : public rclcpp::Node
{
public:
  explicit TrajectoryConcatenatorNode(const rclcpp::NodeOptions & node_options);

private:
  void on_trajectories(const CandidateTrajectories::ConstSharedPtr msg);

  void publish();

  auto parameters() const -> std::shared_ptr<ConcatenatorParam>;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr subs_trajectories_;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;

  std::unique_ptr<concatenator::ParamListener> listener_;

  std::map<std::string, CandidateTrajectories::ConstSharedPtr> buffer_;

  mutable std::mutex mutex_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_concatenator

#endif  // TRAJECTORY_CONCATENATOR_NODE_HPP_
