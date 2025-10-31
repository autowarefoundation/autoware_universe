// Copyright 2021 Tier IV, Inc.
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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
class VirtualTrafficLightModuleManager
: public experimental::SceneModuleManagerInterface<VirtualTrafficLightModule>
{
public:
  explicit VirtualTrafficLightModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "virtual_traffic_light"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  VirtualTrafficLightModule::PlannerParam planner_param_;

  void modifyPathVelocity(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<VirtualTrafficLightModule> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;

  autoware_utils::InterProcessPollingSubscriber<
    tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    sub_virtual_traffic_light_states_;

  rclcpp::Publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    pub_infrastructure_commands_;
};

class VirtualTrafficLightModulePlugin
: public experimental::PluginWrapper<VirtualTrafficLightModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
