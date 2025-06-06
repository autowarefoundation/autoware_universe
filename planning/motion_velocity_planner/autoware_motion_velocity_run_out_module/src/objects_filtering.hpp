// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef OBJECTS_FILTERING_HPP_
#define OBJECTS_FILTERING_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/overlaps/interface.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

/// @brief get the most probable classification label of a predicted object
uint8_t get_most_probable_classification_label(
  const autoware_perception_msgs::msg::PredictedObject & object);

/// @brief identify the most probable class of an object to determine if it is a target and if it is
/// stopped
void classify(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const std::vector<uint8_t> & target_labels, const Parameters & params);

/// @brief calculate the current footprint of an object
void calculate_current_footprint(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object);

/// @brief return true if an object should be ignored
bool skip_object_condition(
  Object & object, const std::optional<DecisionHistory> & prev_decisions,
  const universe_utils::Segment2d & ego_rear_segment, const FilteringData & filtering_data,
  const Parameters & parameters);

/// @brief get the predicted paths with confidence above threshold
std::vector<autoware_perception_msgs::msg::PredictedPath> filter_by_confidence(
  const std::vector<autoware_perception_msgs::msg::PredictedPath> & predicted_paths,
  const uint8_t label, const Parameters & params);

/// @brief cut a predicted path after the given index and repeat the last point for the given
/// duration
void cut_predicted_path_footprint(
  ObjectPredictedPathFootprint & footprint, const size_t cut_index,
  const double standstill_duration_after_cut);

/// @brief calculate the predicted path footprints of an object
void calculate_predicted_path_footprints(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const Parameters & params);

/// @brief calculate the first index of a predicted path that crosses a cut line in the map data
std::optional<size_t> get_cut_predicted_path_index(
  const autoware::motion_velocity_planner::run_out::ObjectPredictedPathFootprint & path,
  const FilteringData & map_data);

/// @brief filter predicted paths of an object using the map filtering data
void filter_predicted_paths(
  Object & object, const FilteringData & map_data, const ObjectParameters & params);

/// @brief prepare data for the dynamic objects and their path footprints to use for collision
/// detection
std::vector<Object> prepare_dynamic_objects(
  const std::vector<std::shared_ptr<motion_velocity_planner::PlannerData::Object>> & objects,
  const TrajectoryCornerFootprint & ego_trajectory,
  const ObjectDecisionsTracker & previous_decisions, const FilteringDataPerLabel & filtering_data,
  const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // OBJECTS_FILTERING_HPP_
