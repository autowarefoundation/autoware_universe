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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include "structs.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <detection_lane_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::behavior_velocity_planner
{
class DetectionLaneModule : public SceneModuleInterfaceWithRTC
{
public:
  DetectionLaneModule(
    const int64_t module_id, const lanelet::ConstLanelet & passing_lane,
    const lanelet::ConstLineString3d & stop_line, const lanelet::ConstLanelets & detection_lanes,
    const double ttc_threshold, const detection_lane::Params::DetectionLane & planner_param,
    const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(PathWithLaneId * path) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  auto get_clustered_pointcloud(const PointCloud::Ptr in, DebugData & debug) const
    -> PointCloud::Ptr;

  auto get_detection_polygons() const -> lanelet::BasicPolygons3d
  {
    lanelet::BasicPolygons3d ret{};
    for (const auto & info : detection_areas_info_) {
      ret.push_back(info.polygon);
    }
    return ret;
  }

  auto get_detection_lanes() const -> lanelet::ConstLanelets
  {
    lanelet::ConstLanelets ret{};
    for (const auto & info : detection_areas_info_) {
      ret.insert(ret.end(), info.lanelets.begin(), info.lanelets.end());
    }
    return ret;
  }

  auto get_pointcloud_object(
    const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
    const DetectionAreasInfo & detection_areas_info, DebugData & debug) -> PointCloudObjects;

  void fill_ttc(PointCloudObjects & objects) const;

  void fill_velocity(PointCloudObject & pointcloud_object);

  auto filter_pointcloud(DebugData & debug) const -> PointCloud::Ptr;

  bool is_prioritized_lane() const;

  bool is_safe(const PointCloudObjects & pointcloud_objects);

  void post_process()
  {
    auto itr = history_.begin();
    while (itr != history_.end()) {
      if ((clock_->now() - itr->second.last_update_time).seconds() > 1.0) {
        itr = history_.erase(itr);
      } else {
        itr++;
      }
    }
  }

  DetectionAreasInfo detection_areas_info_{};

  std::string turn_direction_{""};

  lanelet::ConstLineString3d stop_line_;

  lanelet::Id traffic_light_regulatory_element_id_;

  double ttc_threshold_;

  detection_lane::Params::DetectionLane planner_param_;

  rclcpp::Time last_safe_time_{clock_->now()};

  rclcpp::Time last_unsafe_time_{clock_->now()};

  std::map<lanelet::Id, PointCloudObject> history_;

  DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
