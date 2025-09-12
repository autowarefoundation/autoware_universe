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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;
using lanelet::autoware::Roundabout;

RoundaboutModuleManager::RoundaboutModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc.roundabout"))
{
  const std::string ns(RoundaboutModuleManager::getModuleName());
  auto & rp = roundabout_param_;

  // common
  {
    rp.common.attention_area_margin =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_margin");
    rp.common.attention_area_angle_threshold =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_angle_threshold");
    rp.common.default_stopline_margin =
      get_or_declare_parameter<double>(node, ns + ".common.default_stopline_margin");
    rp.common.path_interpolation_ds =
      get_or_declare_parameter<double>(node, ns + ".common.path_interpolation_ds");
    rp.common.enable_pass_judge_before_default_stopline = get_or_declare_parameter<bool>(
      node, ns + ".common.enable_pass_judge_before_default_stopline");
  }

  // collision_detection
  {
    rp.collision_detection.collision_detection_hold_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_detection_hold_time");
    rp.collision_detection.min_predicted_path_confidence = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.min_predicted_path_confidence");
    rp.collision_detection.collision_start_margin_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_start_margin_time");
    rp.collision_detection.collision_end_margin_time =
      get_or_declare_parameter<double>(node, ns + ".collision_detection.collision_end_margin_time");

    // target_type
    {
      rp.collision_detection.target_type.car =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.car");
      rp.collision_detection.target_type.bus =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bus");
      rp.collision_detection.target_type.truck =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.truck");
      rp.collision_detection.target_type.trailer =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.trailer");
      rp.collision_detection.target_type.motorcycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.motorcycle");
      rp.collision_detection.target_type.bicycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bicycle");
      rp.collision_detection.target_type.unknown =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.unknown");
    }

    // velocity_profile
    {
      rp.collision_detection.velocity_profile.use_upstream = get_or_declare_parameter<bool>(
        node, ns + ".collision_detection.velocity_profile.use_upstream");
      rp.collision_detection.velocity_profile.minimum_upstream_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_upstream_velocity");
      rp.collision_detection.velocity_profile.default_velocity = get_or_declare_parameter<double>(
        node, ns + ".collision_detection.velocity_profile.default_velocity");
      rp.collision_detection.velocity_profile.minimum_default_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_default_velocity");
    }

    rp.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point =
      get_or_declare_parameter<double>(
        node, ns +
                ".collision_detection.avoid_collision_by_acceleration.object_time_margin_to_"
                "collision_point");
  }

  rp.debug.ttc = get_or_declare_parameter<std::vector<int64_t>>(node, ns + ".debug.ttc");

  decision_state_pub_ =
    node.create_publisher<std_msgs::msg::String>("~/debug/roundabout/decision_state", 1);
}

void RoundaboutModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto roundabout_reg_elem_map = planning_utils::getRegElemMapOnPath<Roundabout>(
    path, lanelet_map, planner_data_->current_odometry->pose);

  for (const auto & roundabout : roundabout_reg_elem_map) {
    const auto module_id = roundabout.second.id();
    // Is roundabout entry?
    if (!roundabout.first || !roundabout.first->isEntryLanelet(roundabout.second.id())) {
      continue;
    }

    if (isRegisteredModule(roundabout.second)) {
      continue;
    }

    const auto associative_ids =
      getAssociativeRoundaboutEntryLanelets(roundabout.second, *roundabout.first, routing_graph);

    const auto new_module = std::make_shared<RoundaboutModule>(
      module_id, roundabout.first, roundabout.second.id(), planner_data_, roundabout_param_,
      associative_ids, node_, logger_.get_child("roundabout_module"), clock_, time_keeper_,
      planning_factor_interface_);
    generate_uuid(module_id);
    const UUID uuid = getUUID(new_module->getModuleId());
    rtc_interface_.updateCooperateStatus(
      uuid, true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(), clock_->now());
    registerModule(new_module);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
RoundaboutModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_set](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    const auto & associative_ids = roundabout_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

std::set<lanelet::Id> RoundaboutModuleManager::getAssociativeRoundaboutEntryLanelets(
  const lanelet::ConstLanelet & lane, const lanelet::autoware::Roundabout & roundabout,
  const lanelet::routing::RoutingGraphPtr routing_graph)
{
  std::set<lanelet::Id> associative_lanelets;
  associative_lanelets.insert(lane.id());

  // get same parents lanelets
  const auto parents = routing_graph->previous(lane);
  for (const auto & parent : parents) {
    const auto followings = routing_graph->following(parent);
    for (const auto & following : followings) {
      if (roundabout.isEntryLanelet(following.id())) {
        associative_lanelets.insert(following.id());
      }
    }
  }
  // get adjacent entry lanelets
  const auto right_lanelets =
    planner_data_->route_handler_->getAllRightSharedLinestringLanelets(lane, false);
  const auto left_lanelets =
    planner_data_->route_handler_->getAllLeftSharedLinestringLanelets(lane, false);

  associative_lanelets.insert(lane.id());
  for (const auto & right_lanelet : right_lanelets) {
    if (roundabout.isEntryLanelet(right_lanelet.id())) {
      associative_lanelets.insert(right_lanelet.id());
    }
  }
  for (const auto & left_lanelet : left_lanelets) {
    if (roundabout.isEntryLanelet(left_lanelet.id())) {
      associative_lanelets.insert(left_lanelet.id());
    }
  }
  return associative_lanelets;
}

bool RoundaboutModuleManager::isRegisteredModule(const lanelet::ConstLanelet & entry_lanelet) const
{
  for (const auto & scene_module : scene_modules_) {
    if (scene_module->getModuleId() == entry_lanelet.id()) {
      return true;
    }
  }

  return false;
}

void RoundaboutModuleManager::sendRTC(const Time & stamp)
{
  double min_distance = std::numeric_limits<double>::infinity();
  std_msgs::msg::String decision_type;

  for (const auto & scene_module : scene_modules_) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    const UUID uuid = getUUID(scene_module->getModuleId());
    updateRTCStatus(
      uuid, scene_module->isSafe(), State::RUNNING, scene_module->getDistance(), stamp);

    // ==========================================================================================
    // module debug data
    // ==========================================================================================
    const auto internal_debug_data = roundabout_module->getInternalDebugData();
    if (internal_debug_data.distance < min_distance) {
      min_distance = internal_debug_data.distance;
    }
    decision_type.data += (internal_debug_data.decision_type + "\n");
  }
  rtc_interface_.publishCooperateStatus(stamp);  // publishRTCStatus()

  // ==========================================================================================
  // publish module debug data
  // ==========================================================================================
  decision_state_pub_->publish(decision_type);
}

void RoundaboutModuleManager::modifyPathVelocity(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  SceneModuleManagerInterfaceWithRTC::modifyPathVelocity(path);
}

void RoundaboutModuleManager::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    scene_module->setActivation(rtc_interface_.isActivated(getUUID(scene_module->getModuleId())));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(getUUID(scene_module->getModuleId())));
  }
}

void RoundaboutModuleManager::deleteExpiredModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      // default
      removeRTCStatus(getUUID((*itr)->getModuleId()));
      removeUUID((*itr)->getModuleId());
      registered_module_id_set_.erase((*itr)->getModuleId());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::RoundaboutModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
