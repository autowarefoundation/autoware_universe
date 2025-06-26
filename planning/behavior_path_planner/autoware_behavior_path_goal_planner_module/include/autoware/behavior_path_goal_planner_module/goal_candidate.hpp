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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{
struct GoalCandidate
{
  geometry_msgs::msg::Pose goal_pose{};
  double distance_from_original_goal{0.0};
  double lateral_offset{0.0};
  size_t id{0};
  bool is_safe{true};
  size_t num_objects_to_avoid{0};
};
using GoalCandidates = std::vector<GoalCandidate>;

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_CANDIDATE_HPP_
