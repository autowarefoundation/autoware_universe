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

#ifndef ROAD_USER_STOP_MODULE_HPP_
#define ROAD_USER_STOP_MODULE_HPP_

#include "parameters.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

class RoadUserStopModule : public PluginModuleInterface
{
public:
  RoadUserStopModule() = default;
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & raw_trajectory_points,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; }
  void publish_planning_factor() override;
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo required_subscription_info;
    required_subscription_info.predicted_objects = true;
    return required_subscription_info;
  }

private:
  struct TrackedObject
  {
    unique_identifier_msgs::msg::UUID object_id;
    rclcpp::Time first_detection_time;
    rclcpp::Time last_detection_time;
    geometry_msgs::msg::Point last_position;
  };

  RoadUserStopParameters params_;
  std::string module_name_;
  double vehicle_front_offset_;  // Distance from base_link to front bumper

  std::unordered_map<std::string, TrackedObject> tracked_objects_;

  void onPathWithLaneIdSubscription(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);

  // helper functions
  bool isTargetObject(const autoware_perception_msgs::msg::PredictedObject & object) const;
  bool isObjectOnRoad(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const lanelet::LaneletMapPtr & lanelet_map,
    const lanelet::ConstLanelets & relevant_lanelets) const;
  bool isNearCrosswalk(
    const geometry_msgs::msg::Point & position, const lanelet::LaneletMapPtr & lanelet_map) const;
  bool isOnSidewalk(
    const geometry_msgs::msg::Point & position, const lanelet::LaneletMapPtr & lanelet_map) const;
  bool isWrongWayUser(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const lanelet::ConstLanelet & lanelet) const;

  lanelet::ConstLanelets getRelevantLanelets(
    const std::shared_ptr<const PlannerData> planner_data) const;

  std::optional<size_t> calculateStopPointWithMargin(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points,
    const autoware_perception_msgs::msg::PredictedObject & object) const;

  double calculateRequiredDeceleration(
    const double current_velocity, const double stop_distance) const;

  void updateTrackedObjects(
    const autoware_perception_msgs::msg::PredictedObjects & objects,
    const rclcpp::Time & current_time);

  bool hasMinimumDetectionDuration(
    const std::string & object_id, const rclcpp::Time & current_time) const;

  void publishProcessingTime(const double processing_time_ms) const;

  // debug
  struct DebugData
  {
    lanelet::ConstLanelets relevant_lanelets;
    lanelet::ConstLanelets ego_lanelets;       // lanelets on ego lane
    lanelet::ConstLanelets adjacent_lanelets;  // adjacent lanelets (left/right)
    std::vector<autoware_perception_msgs::msg::PredictedObject> filtered_objects;
    std::optional<size_t> stop_index;
    std::optional<geometry_msgs::msg::Point> stop_point;  // for planning factor
    std::optional<autoware_perception_msgs::msg::PredictedObject>
      stop_target_object;  // object causing stop
  };

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() const;

  visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
    const lanelet::ConstLanelets & lanelets, const std::string & ns,
    const std::array<double, 3> & color) const;

  mutable DebugData debug_data_;

  rclcpp::Logger logger_{rclcpp::get_logger("road_user_stop_module")};
  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    sub_path_with_lane_id_;
  autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr path_with_lane_id_;
};

}  // namespace autoware::motion_velocity_planner

#endif  // ROAD_USER_STOP_MODULE_HPP_
