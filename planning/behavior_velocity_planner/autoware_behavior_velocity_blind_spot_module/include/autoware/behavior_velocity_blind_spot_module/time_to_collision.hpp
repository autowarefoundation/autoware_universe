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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__TIME_TO_COLLISION_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__TIME_TO_COLLISION_HPP_

#include "autoware_utils_geometry/boost_geometry.hpp"

#include <autoware/behavior_velocity_planner_common/planner_data.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{

struct FuturePosition
{
  const autoware_internal_planning_msgs::msg::PathPointWithLaneId position;
  const double duration;
};

/**
 * @brief calculate ego vehicle's future position & duration from current position
 */
std::vector<FuturePosition> calculate_future_profile(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const double minimum_default_velocity, const double time_to_restart,
  const std::shared_ptr<const PlannerData> & planner_data, const lanelet::Id lane_id);

/**
 * @brief compute the time interval in which the `future_positions` enter the `line1` and exit the
 * `line2` considering footprint
 */
std::optional<std::pair<double, double>> compute_time_interval_for_passing_line(
  const std::vector<FuturePosition> & future_positions,
  const autoware_utils_geometry::LinearRing2d & footprint, const lanelet::ConstLineString3d & line1,
  const lanelet::ConstLineString3d & line2);

/**
 * @brief compute the time interval in which the `object` along the predicted path enters the
 * `line1`(or `entry_line` instead) and exits the `line2` considering footprint
 */
std::vector<std::tuple<double, double, autoware_perception_msgs::msg::PredictedPath>>
compute_time_interval_for_passing_line(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLineString3d & line1, const lanelet::ConstLineString3d & entry_line,
  const lanelet::ConstLineString3d & line2);

struct UnsafeObject
{
  UnsafeObject(
    const autoware_perception_msgs::msg::PredictedObject & object_, const double critical_time_,
    const autoware_perception_msgs::msg::PredictedPath & predicted_path_,
    const std::pair<double, double> & object_passage_interval_)
  : object(object_),
    critical_time(critical_time_),
    predicted_path(predicted_path_),
    object_passage_interval(object_passage_interval_)
  {
  }
  autoware_perception_msgs::msg::PredictedObject object;
  double critical_time;
  autoware_perception_msgs::msg::PredictedPath predicted_path;
  std::pair<double, double> object_passage_interval;

  [[nodiscard]] autoware_internal_planning_msgs::msg::SafetyFactor to_safety_factor() const;
};

/**
 * @brief return the most critical time for collision if collision is detected
 */
std::optional<double> get_unsafe_time_if_critical(
  const std::pair<double, double> & ego_passage_interval,
  const std::pair<double, double> & object_passage_interval, const double ttc_start_margin,
  const double ttc_end_margin);

}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_BLIND_SPOT_MODULE__TIME_TO_COLLISION_HPP_
