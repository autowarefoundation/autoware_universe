// Copyright 2024 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__UTILS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__UTILS_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{

namespace utils
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjectKinematics;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjectKinematics;

/**
 * @brief calc absolute normalized yaw difference between lanelet and object
 *
 * @param object
 * @param lanelet
 * @return double
 */
double calcAbsYawDiffBetweenLaneletAndObject(
  const TrackedObject & object, const lanelet::ConstLanelet & lanelet);

bool withinRoadLanelet(
  const TrackedObject & object,
  const std::vector<std::pair<double, lanelet::Lanelet>> & surrounding_lanelets_with_dist,
  const bool use_yaw_information = false);

bool withinRoadLanelet(
  const TrackedObject & object, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const bool use_yaw_information = false);

/**
 * @brief change label for prediction
 *
 * @param label
 * @return ObjectClassification::_label_type
 */
ObjectClassification::_label_type changeVRULabelForPrediction(
  const ObjectClassification::_label_type & label, const TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_);

template <typename T>
std::unordered_set<std::string> removeOldObjectsHistory(
  const double current_time, const double buffer_time,
  std::unordered_map<std::string, std::deque<T>> & target_objects);

extern template std::unordered_set<std::string> removeOldObjectsHistory<RoadUser>(
  const double current_time, const double buffer_time,
  std::unordered_map<std::string, std::deque<RoadUser>> & target_objects);
extern template std::unordered_set<std::string> removeOldObjectsHistory<CrosswalkUser>(
  const double current_time, const double buffer_time,
  std::unordered_map<std::string, std::deque<CrosswalkUser>> & target_objects);

PredictedObjectKinematics convertToPredictedKinematics(
  const TrackedObjectKinematics & tracked_object);

PredictedObject convertToPredictedObject(const TrackedObject & tracked_object);

double calculateLocalLikelihood(
  const lanelet::ConstLanelet & current_lanelet, const TrackedObject & object,
  const double sigma_lateral_offset, const double sigma_yaw_angle_deg);

bool isDuplicated(
  const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
  const LaneletsData & lanelets_data);

bool isDuplicated(
  const PredictedPath & predicted_path, const std::vector<PredictedPath> & predicted_paths);

bool checkCloseLaneletCondition(
  const std::pair<double, lanelet::ConstLanelet> & lanelet, const TrackedObject & object,
  const std::unordered_map<std::string, std::deque<RoadUser>> & road_users_history,
  const double dist_threshold_for_searching_lanelet,
  const double delta_yaw_threshold_for_searching_lanelet);

// NOTE: These two functions are copied from the route_handler package.
lanelet::Lanelets getRightOppositeLanelets(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::ConstLanelet & lanelet);

lanelet::Lanelets getLeftOppositeLanelets(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::ConstLanelet & lanelet);

LaneletsData getCurrentLanelets(
  const TrackedObject & object, lanelet::LaneletMapPtr lanelet_map_ptr,
  const std::unordered_map<std::string, std::deque<RoadUser>> & road_users_history,
  const double dist_threshold_for_searching_lanelet,
  const double delta_yaw_threshold_for_searching_lanelet, const double sigma_lateral_offset,
  const double sigma_yaw_angle_deg);
double lateral_distance_to_lanelet_bounds(
  const lanelet::ConstLanelet & ll, const geometry_msgs::msg::Point & point);

/// Deceleration [m/ss] assumed for each object class when judging whether an object can stop
/// before a line it is predicted to cross. Values are negative accelerations, and every class of
/// autoware_perception_msgs::msg::ObjectClassification has an entry.
struct ObjectDecelerationParams
{
  std::unordered_map<uint8_t, double> per_label{};

  /// Deceleration configured for @p label. Returns 0.0 for a label that has no entry at all, so
  /// that such an object is treated as unable to stop.
  [[nodiscard]] double get(uint8_t label) const;
};

/// Distance [m] travelled before coming to a stop. Infinite when @p deceleration is not negative,
/// i.e. when the object cannot stop at all.
double distance_to_stop_with_deceleration(double speed, double deceleration);

/// 2D linestring over the path positions up to and including @p last_idx.
lanelet::BasicLineString2d to_linestring_2d(
  const std::vector<geometry_msgs::msg::Pose> & path, size_t last_idx);

}  // namespace utils

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__UTILS_HPP_
