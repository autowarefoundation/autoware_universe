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

#include "autoware/trajectory_ranker/simple_trajectory_ranker_node.hpp"

#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_ranker
{
SimpleTrajectoryRanker::SimpleTrajectoryRanker(const rclcpp::NodeOptions & options)
: Node("simple_trajectory_ranker", options)
{
  sub_trajectories_ =
    create_subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "~/input/candidate_trajectories", 1,
      [this](
        const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg) {
        trajectories_callback(msg);
      });

  // Setup publishers
  pub_trajectories_ =
    create_publisher<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>(
      "~/output/scored_trajectories", 1);

  // Debug publisher
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/trajectory_ranker", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  // Parameters
  ranked_generator_ids_ = autoware_utils_rclcpp::get_or_declare_parameter<std::vector<std::string>>(
    *this, "ranked_generator_ids");
}

void SimpleTrajectoryRanker::trajectories_callback(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  std::unordered_map<
    std::string, std::vector<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory>>
    trajectories_per_generator;
  for (const auto & id : ranked_generator_ids_) {
    trajectories_per_generator[id] = {};
  }
  std::vector<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory>
    unranked_trajectories;
  for (const auto & trajectory : msg->candidate_trajectories) {
    const auto generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory scored_trajectory;
    scored_trajectory.candidate_trajectory = trajectory;
    scored_trajectory.score = 0.0;
    if (trajectories_per_generator.count(generator_id)) {
      trajectories_per_generator[generator_id].push_back(scored_trajectory);
    } else {
      unranked_trajectories.push_back(scored_trajectory);
    }
  }
  autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories scored_msg;
  scored_msg.generator_info = msg->generator_info;
  scored_msg.scored_candidate_trajectories.reserve(msg->candidate_trajectories.size());
  for (const auto & str : ranked_generator_ids_) {
    const auto & trajectories = trajectories_per_generator[str];
    scored_msg.scored_candidate_trajectories.insert(
      scored_msg.scored_candidate_trajectories.end(), trajectories.begin(), trajectories.end());
  }
  scored_msg.scored_candidate_trajectories.insert(
    scored_msg.scored_candidate_trajectories.end(), unranked_trajectories.begin(),
    unranked_trajectories.end());
  pub_trajectories_->publish(scored_msg);
}

}  // namespace autoware::trajectory_ranker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_ranker::SimpleTrajectoryRanker)
