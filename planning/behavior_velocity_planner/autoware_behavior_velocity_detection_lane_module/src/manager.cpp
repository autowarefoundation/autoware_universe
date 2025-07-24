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

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;
using lanelet::autoware::DetectionArea;

lanelet::ConstLanelet combineLaneletsShape(const lanelet::ConstLanelets & lanelets)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  lanelet::Points3d centers;
  for (const auto & llt : lanelets) {
    for (const auto & pt : llt.leftBound()) {
      lefts.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.rightBound()) {
      rights.push_back(lanelet::Point3d(pt));
    }
    for (const auto & pt : llt.centerline()) {
      centers.push_back(lanelet::Point3d(pt));
    }
  }
  const auto left_bound = lanelet::LineString3d(lanelet::InvalId, lefts);
  const auto right_bound = lanelet::LineString3d(lanelet::InvalId, rights);
  const auto center_line = lanelet::LineString3d(lanelet::InvalId, centers);
  auto combined_lanelet = lanelet::Lanelet(lanelets.back().id(), left_bound, right_bound);
  combined_lanelet.setCenterline(center_line);
  return combined_lanelet;
}

DetectionLaneModuleManager::DetectionLaneModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc")),
  param_listener_{
    std::make_unique<detection_lane::ParamListener>(node.get_node_parameters_interface())},
  sub_map_projector_info_{&node, "/map/map_projector_info", rclcpp::QoS{1}.transient_local()}
{
}

void DetectionLaneModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto p = param_listener_->get_params();

  const auto map_projector_info = sub_map_projector_info_.take_data();
  if (!map_projector_info) {
    if (mgrs_grid_ == "") {
      RCLCPP_ERROR(logger_, "projector info hasn't been published.");
      return;
    }
  } else {
    mgrs_grid_ = map_projector_info->mgrs_grid;
  }

  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);

  for (const auto & lanelet : lanelets) {
    const auto itr = std::find_if(
      p.config.intersection_names_map.begin(), p.config.intersection_names_map.end(),
      [&lanelet, this](const auto & config) {
        return config.second.mgrs_grid == mgrs_grid_ && config.second.lane_id == lanelet.id();
      });

    if (itr == p.config.intersection_names_map.end()) {
      continue;
    }

    if (!isModuleRegistered(itr->second.lane_id)) {
      const auto passing_lane = lanelet_map->laneletLayer.get(itr->second.lane_id);
      const auto stop_line = lanelet_map->lineStringLayer.get(itr->second.stop_line_id);
      const auto detection_lanes = [&lanelet_map, &itr]() {
        lanelet::ConstLanelets ret{};

        {
          lanelet::ConstLanelets detection_lanes{};
          for (const auto & id : itr->second.lane_1st_ids) {
            detection_lanes.push_back(lanelet_map->laneletLayer.get(id));
          }
          if (!detection_lanes.empty()) {
            ret.push_back(combineLaneletsShape(detection_lanes));
          }
        }

        {
          lanelet::ConstLanelets detection_lanes{};
          for (const auto & id : itr->second.lane_2nd_ids) {
            detection_lanes.push_back(lanelet_map->laneletLayer.get(id));
          }
          if (!detection_lanes.empty()) {
            ret.push_back(combineLaneletsShape(detection_lanes));
          }
        }

        {
          lanelet::ConstLanelets detection_lanes{};
          for (const auto & id : itr->second.lane_3rd_ids) {
            detection_lanes.push_back(lanelet_map->laneletLayer.get(id));
          }
          if (!detection_lanes.empty()) {
            ret.push_back(combineLaneletsShape(detection_lanes));
          }
        }

        {
          lanelet::ConstLanelets detection_lanes{};
          for (const auto & id : itr->second.lane_4th_ids) {
            detection_lanes.push_back(lanelet_map->laneletLayer.get(id));
          }
          if (!detection_lanes.empty()) {
            ret.push_back(combineLaneletsShape(detection_lanes));
          }
        }

        return ret;
      }();

      registerModule(std::make_shared<DetectionLaneModule>(
        itr->second.lane_id, passing_lane, stop_line, detection_lanes, itr->second.ttc_threshold,
        p.detection_lane, logger_.get_child("detection_lane_module"), clock_, time_keeper_,
        planning_factor_interface_));
      generate_uuid(itr->second.lane_id);
      updateRTCStatus(
        getUUID(itr->second.lane_id), true, State::WAITING_FOR_EXECUTION,
        std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
DetectionLaneModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelets = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lanelets](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
    const auto id = scene_module->getModuleId();
    return !std::any_of(
      lanelets.begin(), lanelets.end(), [&id](const auto & lanelet) { return lanelet.id() == id; });
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::DetectionLaneModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
