// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__MANAGER_HPP_
#define EXPERIMENTAL__MANAGER_HPP_

#include "scene_occlusion_spot.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
class OcclusionSpotModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit OcclusionSpotModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "occlusion_spot"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    required_subscription_info.occupancy_grid_map = true;
    return required_subscription_info;
  }

private:
  enum class ModuleID { OCCUPANCY, OBJECT };
  using PlannerParam = occlusion_spot_utils::PlannerParam;

  PlannerParam planner_param_;
  lanelet::Id module_id_;

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;
};

class OcclusionSpotModulePlugin : public PluginWrapper<OcclusionSpotModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
