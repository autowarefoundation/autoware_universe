// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "autoware/behavior_path_side_shift_module/data_structs.hpp"
#include "autoware/behavior_path_side_shift_module/scene.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_planning_msgs/msg/lateral_offset.hpp>
#include <tier4_planning_msgs/srv/set_lateral_offset.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using SetLateralOffset = tier4_planning_msgs::srv::SetLateralOffset;

class SideShiftModuleManager : public SceneModuleManagerInterface
{
public:
  SideShiftModuleManager() : SceneModuleManagerInterface{"side_shift"} {}

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    return std::make_unique<SideShiftModule>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_,
      inserted_lateral_offset_state_, requested_lateral_offset_state_);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  void onSetLateralOffset(
    const tier4_planning_msgs::srv::SetLateralOffset::Request::SharedPtr request,
    tier4_planning_msgs::srv::SetLateralOffset::Response::SharedPtr response);

  void onLateralOffset(const tier4_planning_msgs::msg::LateralOffset::ConstSharedPtr msg);

  std::shared_ptr<SideShiftParameters> parameters_;
  std::shared_ptr<InsertedLateralOffsetState> inserted_lateral_offset_state_;
  std::shared_ptr<RequestedLateralOffsetState> requested_lateral_offset_state_;
  rclcpp::Service<tier4_planning_msgs::srv::SetLateralOffset>::SharedPtr set_lateral_offset_srv_;
  rclcpp::Subscription<tier4_planning_msgs::msg::LateralOffset>::SharedPtr lateral_offset_sub_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__MANAGER_HPP_
