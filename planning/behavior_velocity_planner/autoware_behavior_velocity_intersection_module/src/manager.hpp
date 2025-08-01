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

#include "scene_intersection.hpp"
#include "scene_merge_from_private_road.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_velocity_planner
{
class IntersectionModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit IntersectionModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "intersection"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.traffic_signals = true;
    required_subscription_info.predicted_objects = true;
    required_subscription_info.occupancy_grid_map = true;
    return required_subscription_info;
  }

private:
  IntersectionModule::PlannerParam intersection_param_;
  // additional for INTERSECTION_OCCLUSION
  RTCInterface occlusion_rtc_interface_;

  void launchNewModules(const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;

  /* called from SceneModuleInterfaceWithRTC::plan */
  void sendRTC(const Time & stamp) override;
  void setActivation() override;
  void modifyPathVelocity(autoware_internal_planning_msgs::msg::PathWithLaneId * path) override;
  /* called from SceneModuleInterface::updateSceneModuleInstances */
  void deleteExpiredModules(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroup>::SharedPtr
    tl_observation_pub_;

  std::shared_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_for_occlusion_;
};

class MergeFromPrivateModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit MergeFromPrivateModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "merge_from_private"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  MergeFromPrivateRoadModule::PlannerParam merge_from_private_area_param_;

  void launchNewModules(const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  bool hasSameParentLaneletAndTurnDirectionWithRegistered(const lanelet::ConstLanelet & lane) const;
};

class IntersectionModulePlugin : public PluginWrapper<IntersectionModuleManager>
{
};

class MergeFromPrivateModulePlugin : public PluginWrapper<MergeFromPrivateModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
