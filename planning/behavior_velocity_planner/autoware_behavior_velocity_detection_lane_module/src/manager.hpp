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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <functional>
#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner
{
class DetectionLaneModuleManager : public SceneModuleManagerInterfaceWithRTC
{
public:
  explicit DetectionLaneModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "detection_lane"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.no_ground_pointcloud = true;
    return required_subscription_info;
  }

private:
  std::unique_ptr<detection_lane::ParamListener> param_listener_;

  void launchNewModules(const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
  getModuleExpiredFunction(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  autoware_utils::InterProcessPollingSubscriber<
    autoware_map_msgs::msg::MapProjectorInfo, autoware_utils::polling_policy::Newest>
    sub_map_projector_info_;

  std::string mgrs_grid_{""};
};

class DetectionLaneModulePlugin : public PluginWrapper<DetectionLaneModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
