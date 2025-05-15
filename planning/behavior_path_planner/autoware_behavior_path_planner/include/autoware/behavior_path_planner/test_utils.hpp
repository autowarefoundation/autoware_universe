// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER__TEST_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER__TEST_UTILS_HPP_

#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"

#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::BehaviorPathPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager();

std::shared_ptr<BehaviorPathPlannerNode> generateNode(
  const std::vector<std::string> & module_name_vec,
  const std::vector<std::string> & plugin_name_vec);

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node);
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER__TEST_UTILS_HPP_
