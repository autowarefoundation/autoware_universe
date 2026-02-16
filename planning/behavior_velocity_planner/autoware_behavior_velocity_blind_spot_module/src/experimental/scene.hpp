// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__SCENE_HPP_
#define EXPERIMENTAL__SCENE_HPP_

#include "autoware/behavior_velocity_blind_spot_module/parameter.hpp"
#include "time_to_collision.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/state_machine.hpp>
#include <autoware/behavior_velocity_rtc_interface/experimental/scene_module_interface_with_rtc.hpp>
#include <autoware/lanelet2_utils/intersection.hpp>

#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

/**
 * @brief represent action
 */
struct InternalError
{
  const std::string error;
};

struct OverPassJudge
{
  const std::string report;
};

struct Unsafe
{
  const double stop_s;
};

struct Safe
{
  const double stop_s;
};

struct UnsafeObject
{
  UnsafeObject(
    const autoware_perception_msgs::msg::PredictedObject & object_, const double critical_time_,
    const autoware_perception_msgs::msg::PredictedPath & predicted_path_,
    const TimeInterval & object_passage_interval_)
  : object(object_),
    critical_time(critical_time_),
    predicted_path(predicted_path_),
    object_passage_interval(object_passage_interval_)
  {
  }

  autoware_perception_msgs::msg::PredictedObject object;
  double critical_time;
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  TimeInterval object_passage_interval;

  [[nodiscard]] autoware_internal_planning_msgs::msg::SafetyFactor to_safety_factor() const
  {
    autoware_internal_planning_msgs::msg::SafetyFactor factor;
    factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
    factor.object_id = object.object_id;
    factor.predicted_path = predicted_path;
    factor.ttc_begin = object_passage_interval.start;
    factor.ttc_end = object_passage_interval.end;
    factor.points.push_back(object.kinematics.initial_pose_with_covariance.pose.position);
    factor.is_safe = false;
    return factor;
  }
};

using BlindSpotDecision = std::variant<InternalError, OverPassJudge, Unsafe, Safe>;
using TurnDirection = autoware::experimental::lanelet2_utils::TurnDirection;

class BlindSpotModule : public SceneModuleInterfaceWithRTC
{
public:
  struct DebugData
  {
    std::optional<geometry_msgs::msg::Pose> virtual_wall_pose{std::nullopt};
    std::optional<lanelet::CompoundPolygon3d> attention_area;
    std::optional<lanelet::CompoundPolygon3d> path_polygon;
    std::optional<lanelet::ConstLineString3d> virtual_blind_lane_boundary_after_turning;
    std::optional<lanelet::ConstLineString3d> virtual_ego_straight_path_after_turning;
    std::optional<TimeInterval> ego_passage_interval;
    std::optional<double> critical_time;
    std::optional<std::vector<UnsafeObject>> unsafe_objects;
  };

public:
  BlindSpotModule(
    const lanelet::Id module_id, const lanelet::Id lane_id, const TurnDirection turn_direction,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface,
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  std::vector<autoware::motion_utils::VirtualWall> createVirtualWalls() override;

private:
  // const variables
  const lanelet::Id lane_id_;
  const PlannerParam planner_param_;
  const TurnDirection turn_direction_;

  // (semi) const variables
  std::optional<lanelet::ConstLanelet> road_lanelets_before_turning_merged_{std::nullopt};
  std::optional<lanelet::ConstLanelets> blind_side_lanelets_before_turning_{std::nullopt};
  std::optional<lanelet::ConstLineString3d> virtual_blind_lane_boundary_after_turning_{
    std::nullopt};

  // state variables
  bool is_over_pass_judge_line_{false};

  // Parameter

  void initializeRTCStatus();
  BlindSpotDecision modifyPathVelocityDetail(Trajectory & path, const PlannerData & planner_data);
  // setSafe(), setDistance()
  void setRTCStatus(
    const BlindSpotDecision & decision, const Trajectory & path, const PlannerData & planner_data);
  template <typename Decision>
  void setRTCStatusByDecision(
    const Decision & decision, const Trajectory & path, const PlannerData & planner_data);
  // stop/GO
  void reactRTCApproval(
    const BlindSpotDecision & decision, Trajectory & path, const PlannerData & planner_data);
  template <typename Decision>
  void reactRTCApprovalByDecision(
    const Decision & decision, Trajectory & path, const PlannerData & planner_data);

  /**
   * @brief obtain object with ttc information which is considered dangerous
   * @return return unsafe objects, in order of collision time (front element is nearest)
   */
  std::vector<UnsafeObject> collect_unsafe_objects(
    const std::vector<autoware_perception_msgs::msg::PredictedObject> & attention_objects,
    const lanelet::ConstLanelet & ego_path_polygon,
    const TimeInterval & ego_passage_time_interval) const;

  /**
   * @brief filter objects whose position is inside the attention_area and whose type is target type
   */
  std::vector<autoware_perception_msgs::msg::PredictedObject> filter_attention_objects(
    const lanelet::BasicPolygon2d & attention_area, const double lateral_gap,
    const PlannerData & planner_data) const;

  /**
   * @brief Check if object is belong to targeted classes
   * @param object Dynamic object
   * @return True when object belong to targeted classes
   */
  bool isTargetObjectType(const autoware_perception_msgs::msg::PredictedObject & object) const;

  static bool is_vru_object_type(const autoware_perception_msgs::msg::PredictedObject & object);

  /**
   * @brief compute the deceleration and jerk for collision stop from `ttc`
   * if ttc < critical_threshold_ub, use critical profile
   * if ttc > semi_critical_lb, use semi_critical profile
   * otherwise, interpolated between the two
   */
  std::pair<double, double> compute_decel_and_jerk_from_ttc(const double ttc) const;

  StateMachine state_machine_;  //! for state

  // Debug
  mutable DebugData debug_data_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_state_pub_;
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_HPP_
