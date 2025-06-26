// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_VISITOR_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_VISITOR_HPP_

#include "tier4_planning_msgs/msg/detail/avoidance_debug_msg_array__struct.hpp"
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>

#include <memory>
namespace autoware::behavior_path_planner
{
// Forward Declaration
class StaticObstacleAvoidanceModule;
class AvoidanceByLCModule;
class ExternalRequestLaneChangeModule;
class LaneChangeInterface;
class StartPlannerModule;
class GoalPlannerModule;
class SideShiftModule;

using tier4_planning_msgs::msg::AvoidanceDebugMsg;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using DebugStringMsg = autoware_internal_debug_msgs::msg::StringStamped;

class SceneModuleVisitor
{
public:
  void visitAvoidanceModule(const StaticObstacleAvoidanceModule * module) const;
  void visitStartPlannerModule(const StartPlannerModule * module) const;

  std::shared_ptr<AvoidanceDebugMsgArray> getAvoidanceModuleDebugMsg() const;
  std::shared_ptr<DebugStringMsg> getStartPlannerModuleDebugMsg() const;

protected:
  mutable std::shared_ptr<AvoidanceDebugMsgArray> avoidance_visitor_;
  mutable std::shared_ptr<DebugStringMsg> start_planner_visitor_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_VISITOR_HPP_
