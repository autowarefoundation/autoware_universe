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

#include "scene_roundabout.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <functional>
#include <memory>
#include <set>
#include <unordered_map>

namespace autoware::behavior_velocity_planner::experimental
{
class RoundaboutModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit RoundaboutModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "roundabout"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }

private:
  RoundaboutModule::PlannerParam roundabout_param_;

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp, const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(const Trajectory & path, const PlannerData & planner_data) override;

  /* * @brief Get the lanelets that are associated with the roundabout entry lanelets.
   * This function retrieves the lanelets that are associated with the roundabout entry lanelets.
   * @param lane The lanelet to check.
   * @param roundabout The roundabout to check.
   * @return A set of lanelet IDs that are associated with the roundabout entry lanelets.
   * If the lanelet is not a roundabout entry lanelet, an empty set is returned.
   */
  std::set<lanelet::Id> getAssociativeRoundaboutEntryLanelets(
    const lanelet::ConstLanelet & lane, const lanelet::autoware::Roundabout & roundabout,
    const lanelet::routing::RoutingGraphPtr routing_graph, const PlannerData & planner_data);
  bool isRegisteredModule(const lanelet::ConstLanelet & entry_lanelet) const;

  /* called from SceneModuleInterfaceWithRTC::plan */
  void sendRTC(const Time & stamp) override;
  void setActivation() override;
  /* called from SceneModuleInterface::updateSceneModuleInstances */
  void deleteExpiredModules(const Trajectory & path, const PlannerData & planner_data) override;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
};

class RoundaboutModulePlugin : public experimental::PluginWrapper<RoundaboutModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // MANAGER_HPP_
