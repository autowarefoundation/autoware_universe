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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene_crosswalk.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner::experimental
{
class CrosswalkModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit CrosswalkModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "crosswalk"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.traffic_signals = true;
    required_subscription_info.predicted_objects = true;
    required_subscription_info.occupancy_grid_map = true;
    return required_subscription_info;
  }

private:
  CrosswalkModule::PlannerParam crosswalk_planner_param_{};

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;
};

class CrosswalkModulePlugin : public experimental::PluginWrapper<CrosswalkModuleManager>
{
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // MANAGER_HPP_
