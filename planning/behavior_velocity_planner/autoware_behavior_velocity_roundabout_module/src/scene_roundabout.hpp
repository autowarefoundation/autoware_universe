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

#ifndef SCENE_ROUNDABOUT_HPP_
#define SCENE_ROUNDABOUT_HPP_

#include "decision_result.hpp"
#include "object_manager.hpp"
#include "roundabout_lanelets.hpp"
#include "roundabout_stoplines.hpp"

#include <autoware/behavior_velocity_intersection_module/interpolated_path_info.hpp>
#include <autoware/behavior_velocity_intersection_module/result.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <autoware/behavior_velocity_rtc_interface/scene_module_interface_with_rtc.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/roundabout.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::behavior_velocity_planner
{

class RoundaboutModule : public SceneModuleInterfaceWithRTC
{
public:
  struct PlannerParam
  {
    struct Common
    {
      double attention_area_margin;
      double attention_area_angle_threshold;
      double default_stopline_margin;
      double path_interpolation_ds;
      double max_accel;
      double max_jerk;
      double delay_response_time;
      bool enable_pass_judge_before_default_stopline;
    } common;

    struct TargetType
    {
      bool car;
      bool bus;
      bool truck;
      bool trailer;
      bool motorcycle;
      bool bicycle;
      bool unknown;
    };

    struct CollisionDetection
    {
      double collision_detection_hold_time;
      double min_predicted_path_confidence;
      double collision_start_margin_time;
      double collision_end_margin_time;
      TargetType target_type;
      struct VelocityProfile
      {
        bool use_upstream;
        double minimum_upstream_velocity;
        double default_velocity;
        double minimum_default_velocity;
      } velocity_profile;
      struct AvoidCollisionByAcceleration
      {
        double object_time_margin_to_collision_point;
      } avoid_collision_by_acceleration;
    } collision_detection;

    struct Debug
    {
      std::vector<int64_t> ttc;
    } debug;
  };

  struct DebugData
  {
    std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};
    std::optional<geometry_msgs::msg::Pose> first_pass_judge_wall_pose{std::nullopt};
    bool passed_first_pass_judge{false};
    std::optional<geometry_msgs::msg::Pose> too_late_stop_wall_pose{std::nullopt};

    std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area{std::nullopt};
    std::optional<std::vector<lanelet::CompoundPolygon3d>> adjacent_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> first_attention_area{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> ego_lane{std::nullopt};

    std::optional<geometry_msgs::msg::Polygon> candidate_collision_ego_lane_polygon{std::nullopt};
    std::optional<geometry_msgs::msg::Polygon> candidate_collision_object_polygon{std::nullopt};
    autoware_perception_msgs::msg::PredictedObjects unsafe_targets;
    autoware_perception_msgs::msg::PredictedObjects misjudge_targets;
    autoware_perception_msgs::msg::PredictedObjects too_late_detect_targets;
  };

  struct InternalDebugData
  {
    double distance{0.0};
    std::string decision_type{};
  };

  using TimeDistanceArray = std::vector<std::pair<double /* time*/, double /* distance*/>>;

  /**
   * @brief
   */
  struct PassJudgeStatus
  {
    //! true if ego is over the 1st pass judge line
    const bool is_over_1st_pass_judge;

    //! true only when ego passed 1st pass judge line safely for the first time
    const bool safely_passed_1st_judge_line;
  };

  /**
   * @brief
   */
  struct CollisionStatus
  {
    enum BlameType {
      BLAME_AT_FIRST_PASS_JUDGE,
      BLAME_AT_SECOND_PASS_JUDGE,
    };
    const bool collision_detected;
    const CollisionInterval::LanePosition collision_position;
    const std::vector<std::pair<BlameType, std::shared_ptr<ObjectInfo>>> too_late_detect_objects;
    const std::vector<std::pair<BlameType, std::shared_ptr<ObjectInfo>>> misjudge_objects;
  };

  RoundaboutModule(
    const int64_t module_id, std::shared_ptr<const lanelet::autoware::Roundabout> roundabout,
    const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
    const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
    rclcpp::Node & node, const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup primary-function [fn] primary functions
   * the entrypoint of this module is modifyPathVelocity() function that calculates safety decision
   * of latest context and send it to RTC and then react to RTC approval. The reaction to RTC
   * approval may not be based on the latest decision of this module depending on the auto-mode
   * configuration. For module side it is not visible if the module is operating in auto-mode or
   * manual-module. At first, initializeRTCStatus() is called to reset the safety value of
   * ROUNDABOUT. Then modifyPathVelocityDetail() is called to analyze
   * the context. Then prepareRTCStatus() is called to set the safety value of ROUNDABOUT.
   * @{
   */
  bool modifyPathVelocity(PathWithLaneId * path) override;
  /** @}*/

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

  const std::set<lanelet::Id> & getAssociativeIds() const { return associative_ids_; }
  InternalDebugData & getInternalDebugData() const { return internal_debug_data_; }

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup planning-factor-variables [var] planning factor variables
   * following variables are used to publish planning factors
   * @{
   */

  autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factor_array_;

  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup const-variables [var] const variables
   * following variables are unique to this roundabout lanelet or to this module
   * @{
   */

  //! roundabout lanelet element
  const std::shared_ptr<const lanelet::autoware::Roundabout> roundabout_reg_elem_;

  const PlannerParam planner_param_;

  //! lanelet of this roundabout
  const lanelet::Id lane_id_;

  //! associative(sibling) lanelets ids
  const std::set<lanelet::Id> associative_ids_;

  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup semi-const-variables [var] semi-const variables
   * following variables are immutable once initialized
   * @{
   */

  //! cache RoundaboutLanelets struct
  std::optional<RoundaboutLanelets> roundabout_lanelets_{std::nullopt};
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup pass-judge-variable [var] pass judge variables
   * following variables are state variables that depends on how the vehicle passed the roundabout
   * @{
   */
  //! if true, this module never commands to STOP anymore
  bool is_permanent_go_{false};

  //! for checking if ego is over the pass judge lines because previously the situation was SAFE
  DecisionResult prev_decision_result_{InternalError{""}};

  //! save the time and ego position when ego passed the 1st_pass_judge_line with safe
  //! decision. If collision is expected after these variables are non-null, then it is the fault of
  //! past perception failure at these time.
  std::optional<std::pair<rclcpp::Time, geometry_msgs::msg::Pose>>
    safely_passed_1st_judge_line_time_{std::nullopt};
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup collision-variables [var] collision detection
   * @{
   */
  //! debouncing for stable SAFE decision
  StateMachine collision_state_machine_;

  //! container for storing safety status of objects on the attention area
  ObjectInfoManager object_info_manager_;
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @ingroup primary-functions
   * @{
   */
  /**
   * @brief set all RTC variable to true(safe) and -INF
   */
  void initializeRTCStatus();

  /**
   * @brief analyze collision objects context and return DecisionResult
   */
  DecisionResult modifyPathVelocityDetail(PathWithLaneId * path);

  /**
   * @brief set RTC value according to calculated DecisionResult
   */
  void prepareRTCStatus(
    const DecisionResult &, const autoware_internal_planning_msgs::msg::PathWithLaneId & path);

  /**
   * @brief act based on current RTC approval
   */
  void reactRTCApproval(
    const DecisionResult & decision_result,
    autoware_internal_planning_msgs::msg::PathWithLaneId * path);
  /** @}*/

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup prepare-data [fn] basic data construction
   * @{
   */
  /**
   * @struct
   */
  struct BasicData
  {
    InterpolatedPathInfo interpolated_path_info;
    RoundaboutStopLines roundabout_stoplines;
    PathLanelets path_lanelets;
  };

  /**
   * @brief prepare basic data structure
   * @return return RoundaboutStopLines if all data is valid, otherwise InternalError
   * @note if successful, it is ensure that roundabout_lanelets_,
   * roundabout_lanelets.first_conflicting_lane are not null
   *
   * To simplify modifyPathVelocityDetail(), this function is used at first
   */
  Result<BasicData, InternalError> prepareRoundaboutData(PathWithLaneId * path);

  /**
   * @brief generate RoundaboutStopLines
   */
  std::optional<RoundaboutStopLines> generateRoundaboutStopLines(
    const lanelet::ConstLanelet & first_attention_lane,
    const InterpolatedPathInfo & interpolated_path_info,
    autoware_internal_planning_msgs::msg::PathWithLaneId * original_path) const;

  /**
   * @brief generate RoundaboutLanelets
   */
  RoundaboutLanelets generateObjectiveLanelets(
    lanelet::LaneletMapConstPtr lanelet_map_ptr,
    lanelet::routing::RoutingGraphPtr routing_graph_ptr,
    const lanelet::ConstLanelet & assigned_lanelet) const;

  /**
   * @brief generate PathLanelets
   */
  std::optional<PathLanelets> generatePathLanelets(
    const lanelet::ConstLanelets & lanelets_on_path,
    const InterpolatedPathInfo & interpolated_path_info, const size_t closest_idx) const;

  /**
   * @brief generate discretized detection lane linestring.
   */
  std::vector<lanelet::ConstLineString3d> generateDetectionLaneDivisions(
    const lanelet::ConstLanelets & conflicting_detection_lanelets,
    const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution) const;

  /**
   * @brief get preceding lanelets
   */

  lanelet::ConstLanelets getPrecedingLanelets(
    const lanelet::routing::RoutingGraphPtr & graph, const lanelet::ConstLanelet & lanelet) const;
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup pass-judge-decision [fn] pass judge decision
   * @{
   */
  /**
   * @brief check if ego is already over the pass judge line
   * @return if ego is over 1st pass judge lines, return InternalError, else return
   * is_over_1st_pass_judge
   * @attention this function has access to value() of roundabout_stoplines.default_stopline
   */
  PassJudgeStatus isOverPassJudgeLinesStatus(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
    const RoundaboutStopLines & roundabout_stoplines);
  /** @} */

private:
  /**
   ***********************************************************
   ***********************************************************
   ***********************************************************
   * @defgroup collision-detection [fn] check collision
   * @{
   */
  bool isTargetCollisionVehicleType(
    const autoware_perception_msgs::msg::PredictedObject & object) const;

  /**
   * @brief find the objects on attention_area/roundabout_area and update positional information
   * @attention this function has access to value() of roundabout_lanelets_
   */
  void updateObjectInfoManagerArea();

  /**
   * @brief find the collision Interval/CollisionKnowledge of registered objects
   * @attention this function has access to value() of roundabout_lanelets_
   */
  void updateObjectInfoManagerCollision(
    const PathLanelets & path_lanelets, const TimeDistanceArray & time_distance_array,
    const bool passed_1st_judge_line_first_time,
    autoware_internal_debug_msgs::msg::Float64MultiArrayStamped * object_ttc_time_array);

  void cutPredictPathWithinDuration(
    const builtin_interfaces::msg::Time & object_stamp, const double time_thr,
    autoware_perception_msgs::msg::PredictedPath * path) const;

  /**
   * @brief generate the message explaining why too_late_detect_objects/misjudge_objects exist and
   * blame past perception fault
   */
  std::string generateDetectionBlameDiagnosis(
    const std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<ObjectInfo>>> &
      too_late_detect_objects,
    const std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<ObjectInfo>>> &
      misjudge_objects) const;

  /**
   * @brief generate the message explaining how much ego should accelerate to avoid future dangerous
   * situation
   */
  std::string generateEgoRiskEvasiveDiagnosis(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
    const TimeDistanceArray & ego_time_distance_array,
    const std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<ObjectInfo>>> &
      too_late_detect_objects,
    const std::vector<std::pair<CollisionStatus::BlameType, std::shared_ptr<ObjectInfo>>> &
      misjudge_objects) const;

  /**
   * @brief return if collision is detected and the collision position
   */
  CollisionStatus detectCollision(const bool is_over_pass_judge_line) const;

  std::optional<size_t> checkAngleForTargetLanelets(
    const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets) const;

  /**
   * @brief calculate ego vehicle profile along the path inside the roundabout as the sequence of
   * (time of arrival, traveled distance) from current ego position
   * @attention this function has access to value() of
   * roundabout_stoplines.first_attention_stopline
   */
  TimeDistanceArray calcRoundaboutPassingTime(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
    const RoundaboutStopLines & roundabout_stoplines,
    autoware_internal_debug_msgs::msg::Float64MultiArrayStamped * ego_ttc_array) const;
  /** @} */

  mutable DebugData debug_data_;
  mutable InternalDebugData internal_debug_data_{};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr
    ego_ttc_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr
    object_ttc_pub_;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_ROUNDABOUT_HPP_
