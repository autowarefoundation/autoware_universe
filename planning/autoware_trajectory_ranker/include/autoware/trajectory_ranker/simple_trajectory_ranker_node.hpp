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

#ifndef AUTOWARE__TRAJECTORY_RANKER__SIMPLE_TRAJECTORY_RANKER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__SIMPLE_TRAJECTORY_RANKER_NODE_HPP_

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{

class SimpleTrajectoryRanker : public rclcpp::Node
{
public:
  explicit SimpleTrajectoryRanker(const rclcpp::NodeOptions & options);

private:
  void trajectories_callback(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    sub_trajectories_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>::SharedPtr
    pub_trajectories_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
  std::vector<std::string> ranked_generator_name_prefixes_;
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__SIMPLE_TRAJECTORY_RANKER_NODE_HPP_
