// Copyright 2020 Tier IV, Inc.
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

#include "scene_walkway.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/crosswalk.hpp>

#include <functional>
#include <memory>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class WalkwayModuleManager : public experimental::SceneModuleManagerInterface<>
{
public:
  explicit WalkwayModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "walkway"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  WalkwayModule::PlannerParam walkway_planner_param_{};

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;
};

class WalkwayModulePlugin : public experimental::PluginWrapper<WalkwayModuleManager>
{
};
}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
