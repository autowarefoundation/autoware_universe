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

#include "path_length_buffer.hpp"
#include "type_alias.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware_motion_velocity_road_user_stop_module/road_user_stop_module_parameters.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
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
  // parameter listener
  std::shared_ptr<road_user_stop::ParamListener> param_listener_;
  CommonParam common_param_{};

  std::string module_name_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    processing_time_detail_pub_{};
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{};

  mutable std::unordered_map<std::string, TrackedObject> tracked_objects_;

  void onPathWithLaneIdSubscription(
    const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg);

  // helper functions
  bool isTargetObject(const uint8_t label) const;
  bool isObjectOnRoad(
    const PredictedObject & object,
    const std::vector<autoware_utils_geometry::Polygon2d> & relevant_polygons) const;
  bool isNearCrosswalk(
    const geometry_msgs::msg::Point & position, const lanelet::LaneletMapPtr & lanelet_map) const;
  bool isOppositeTrafficUser(
    const PredictedObject & object, const lanelet::ConstLanelet & lanelet) const;

  std::pair<
    std::vector<autoware_utils_geometry::Polygon2d>,
    std::vector<autoware_utils_geometry::Polygon2d>>
  getRelevantLanelets(const std::shared_ptr<const PlannerData> planner_data) const;

  std::vector<autoware_utils_geometry::Polygon2d> getTrajectoryPolygons(
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::optional<geometry_msgs::msg::Point> planStop(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & trajectory_points,
    const std::vector<StopObstacle> & stop_obstacles, const double dist_to_bumper);

  std::vector<StopObstacle> filterStopObstacles(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polygons,
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons_for_vru,
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons_for_opposing_traffic,
    const rclcpp::Time & current_time, const double dist_to_bumper);

  void updateTrackedObjects(
    const std::vector<std::shared_ptr<PlannerData::Object>> & objects,
    const rclcpp::Time & current_time);

  bool hasMinimumDetectionDuration(
    const std::string & object_id, const rclcpp::Time & current_time) const;

  lanelet::ConstLanelets getIntersectionLanelets(
    const lanelet::ConstLanelet & intersection_lanelet,
    const lanelet::LaneletMapPtr & lanelet_map) const;

  MarkerArray createDebugMarkerArray() const;

  MarkerArray createLaneletPolygonsMarkerArray(
    const lanelet::ConstLanelets & lanelets, const std::string & ns,
    const std::array<double, 3> & color) const;

  mutable DebugData debug_data_;

  rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)};

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    sub_path_with_lane_id_;
  autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr path_with_lane_id_;

  mutable std::unordered_map<double, std::vector<Polygon2d>> trajectory_polygon_for_inside_map_{};
  std::vector<StopObstacle> prev_stop_obstacles_{};

  std::optional<StopObstacle> pickStopObstacleFromPredictedObject(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::shared_ptr<PlannerData::Object> object,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const std::vector<autoware_utils_geometry::Polygon2d> & decimated_traj_polygons,
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons_for_vru,
    const std::vector<autoware_utils_geometry::Polygon2d> & polygons_for_opposing_traffic,
    const lanelet::ConstLanelets & lanelets_for_opposing_traffic, const rclcpp::Time & current_time,
    const double dist_to_bumper);

  // Stop planning
  void holdPreviousStopIfNecessary(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    std::optional<double> & determined_zero_vel_dist);

  std::optional<geometry_msgs::msg::Point> calcStopPoint(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points,
    const std::optional<StopObstacle> & determined_stop_obstacle,
    const std::optional<double> & determined_zero_vel_dist);

  std::vector<StopObstacle> getClosestStopObstacles(
    const std::vector<StopObstacle> & stop_obstacles) const;

  // Previous stop distance info for holding stop position
  std::optional<std::pair<std::vector<TrajectoryPoint>, double>> prev_stop_distance_info_{
    std::nullopt};

  // Path length buffer for negative velocity obstacles
  PathLengthBuffer path_length_buffer_;

  // Stop margin calculation
  double calcDesiredStopMargin(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const size_t ego_segment_idx,
    const double dist_to_collide_on_ref_traj) const;

  std::optional<double> calcCandidateZeroVelDist(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const double dist_to_collide_on_ref_traj,
    const double desired_stop_margin, const double dist_to_bumper) const;

  double calcMarginFromObstacleOnCurve(
    const std::shared_ptr<const PlannerData> planner_data,
    const std::vector<TrajectoryPoint> & traj_points, const StopObstacle & stop_obstacle,
    const double dist_to_bumper, const double default_stop_margin) const;
};

}  // namespace autoware::motion_velocity_planner

#endif  // ROAD_USER_STOP_MODULE_HPP_
