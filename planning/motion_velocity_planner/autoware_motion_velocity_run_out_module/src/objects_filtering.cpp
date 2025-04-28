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

#include "objects_filtering.hpp"

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_object__struct.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/overlaps/interface.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

uint8_t get_most_probable_classification_label(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  double highest_probability = 0.0;
  uint8_t most_probable_label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  for (const auto & classification : object.classification) {
    if (classification.probability > highest_probability) {
      highest_probability = classification.probability;
      most_probable_label = classification.label;
    }
  }
  return most_probable_label;
}

void classify(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const std::vector<uint8_t> & target_labels, const Parameters & params)
{
  object.label = get_most_probable_classification_label(predicted_object);
  object.has_target_label =
    std::find(target_labels.begin(), target_labels.end(), object.label) != target_labels.end();
  if (
    predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x <=
    params.object_parameters_per_label[object.label].stopped_velocity_threshold) {
    object.is_stopped = true;
  }
}

void calculate_current_footprint(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object)
{
  const auto half_length = predicted_object.shape.dimensions.x * 0.5;
  object.current_footprint = autoware_utils::to_footprint(
    predicted_object.kinematics.initial_pose_with_covariance.pose, half_length, half_length,
    predicted_object.shape.dimensions.y);
}

bool skip_object_condition(
  const Object & object, const std::optional<DecisionHistory> & prev_decisions,
  const universe_utils::Segment2d & ego_rear_segment, const FilteringData & filtering_data,
  const Parameters & params)
{
  constexpr auto skip_object = true;
  const auto rear_vector = ego_rear_segment.second - ego_rear_segment.first;
  // normal vector in the direction coming from the rear
  const auto rear_normal = universe_utils::Point2d(-rear_vector.y(), rear_vector.x());
  const auto object_vector = object.position - ego_rear_segment.first;
  const auto is_behind_ego = rear_normal.dot(object_vector) < 0.0;
  if (params.object_parameters_per_label[object.label].ignore_if_behind_ego && is_behind_ego) {
    return skip_object;
  }
  const auto & is_previous_target =
    prev_decisions && (prev_decisions->decisions.back().type == stop ||
                       (prev_decisions->decisions.back().collision.has_value() &&
                        prev_decisions->decisions.back().collision->type == collision));
  if (is_previous_target) {
    return !skip_object;
  }
  if (params.object_parameters_per_label[object.label].ignore_if_stopped && object.is_stopped) {
    return skip_object;
  }
  if (!object.has_target_label) {
    return skip_object;
  }
  if (!filtering_data.ignore_objects_rtree.is_geometry_disjoint_from_rtree_polygons(
        object.current_footprint, filtering_data.ignore_objects_polygons)) {
    return skip_object;
  }
  return !skip_object;
}

std::vector<autoware_perception_msgs::msg::PredictedPath> filter_by_confidence(
  const std::vector<autoware_perception_msgs::msg::PredictedPath> & predicted_paths,
  const uint8_t label, const Parameters & params)
{
  std::vector<autoware_perception_msgs::msg::PredictedPath> filtered;
  auto max_confidence = 0.0f;
  for (const auto & path : predicted_paths) {
    max_confidence = std::max(max_confidence, path.confidence);
    if (
      path.confidence >= params.object_parameters_per_label[label].confidence_filtering_threshold) {
      filtered.push_back(path);
    }
  }
  if (params.object_parameters_per_label[label].confidence_filtering_only_use_highest) {
    const auto new_end = std::remove_if(filtered.begin(), filtered.end(), [&](const auto & p) {
      return p.confidence != max_confidence;
    });
    filtered.erase(new_end, filtered.end());
  }
  return filtered;
}

void calculate_predicted_path_footprints(
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  [[maybe_unused]] const Parameters & params)
{
  auto width = 0.0;
  auto half_length = 0.0;
  if (
    predicted_object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX ||
    predicted_object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    half_length = predicted_object.shape.dimensions.x * 0.5;
    width = predicted_object.shape.dimensions.y;
  } else if (predicted_object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    for (const auto p : predicted_object.shape.footprint.points) {
      const auto zero_point = geometry_msgs::msg::Point().set__x(0.0).set__y(0.0);
      width = std::max(width, autoware_utils_geometry::calc_distance2d(p, zero_point));
    }
    half_length = width / 2.0;
  }
  // calculate footprint
  for (const auto & path :
       filter_by_confidence(predicted_object.kinematics.predicted_paths, object.label, params)) {
    ObjectPredictedPathFootprint footprint;
    footprint.time_step = rclcpp::Duration(path.time_step).seconds();
    for (const auto & p : path.path) {
      const auto object_polygon = autoware_utils::to_footprint(p, half_length, half_length, width);
      footprint.predicted_path_footprint.corner_linestrings[front_left].push_back(
        object_polygon.outer()[0]);
      footprint.predicted_path_footprint.corner_linestrings[front_right].push_back(
        object_polygon.outer()[1]);
      footprint.predicted_path_footprint.corner_linestrings[rear_right].push_back(
        object_polygon.outer()[2]);
      footprint.predicted_path_footprint.corner_linestrings[rear_left].push_back(
        object_polygon.outer()[3]);
    }
    object.predicted_path_footprints.push_back(footprint);
  }
}

void cut_footprint_after_index(ObjectPredictedPathFootprint & footprint, const size_t index)
{
  footprint.predicted_path_footprint.corner_linestrings[front_left].resize(index);
  footprint.predicted_path_footprint.corner_linestrings[front_right].resize(index);
  footprint.predicted_path_footprint.corner_linestrings[rear_left].resize(index);
  footprint.predicted_path_footprint.corner_linestrings[rear_right].resize(index);
}

std::optional<size_t> get_cut_predicted_path_index(
  const ObjectPredictedPathFootprint & predicted_path_footprint, const FilteringData & map_data)
{
  for (auto i = 0UL; i + 1 < predicted_path_footprint.predicted_path_footprint.size(); ++i) {
    for (const auto & corner : {front_left, front_right, rear_left, rear_right}) {
      const auto & ls =
        predicted_path_footprint.predicted_path_footprint.corner_linestrings[corner];
      const auto & segment = universe_utils::Segment2d(ls[i], ls[i + 1]);
      std::vector<SegmentNode> query_results;
      map_data.cut_predicted_paths_rtree.query(
        boost::geometry::index::intersects(segment), std::back_inserter(query_results));
      for (const auto & candidate : query_results) {
        if (universe_utils::intersect(
              segment.first, segment.second, candidate.first.first, candidate.first.second)) {
          return i;
        }
      }
    }
  }
  return std::nullopt;
}

void filter_predicted_paths(Object & object, const FilteringData & map_data)
{
  for (auto & predicted_path_footprint : object.predicted_path_footprints) {
    const auto cut_index = get_cut_predicted_path_index(predicted_path_footprint, map_data);
    if (cut_index) {
      cut_footprint_after_index(predicted_path_footprint, *cut_index);
    }
  }
}

std::vector<Object> prepare_dynamic_objects(
  const std::vector<std::shared_ptr<motion_velocity_planner::PlannerData::Object>> & objects,
  const TrajectoryCornerFootprint & ego_trajectory,
  const ObjectDecisionsTracker & previous_decisions, const FilteringDataPerLabel & filtering_data,
  const Parameters & params)
{
  std::vector<Object> filtered_objects;
  const auto target_labels = params.target_labels();
  const auto ego_rear_segment = ego_trajectory.get_rear_segment(0);
  for (const auto & object : objects) {
    Object filtered_object;
    filtered_object.object = object;
    filtered_object.uuid = universe_utils::toHexString(object->predicted_object.object_id);
    filtered_object.position =
      universe_utils::fromMsg(
        object->predicted_object.kinematics.initial_pose_with_covariance.pose.position)
        .to_2d();
    classify(filtered_object, object->predicted_object, target_labels, params);
    calculate_current_footprint(filtered_object, object->predicted_object);
    const auto & previous_object_decisions = previous_decisions.get(filtered_object.uuid);
    if (skip_object_condition(
          filtered_object, previous_object_decisions, ego_rear_segment,
          filtering_data[filtered_object.label], params)) {
      continue;
    }
    calculate_predicted_path_footprints(filtered_object, object->predicted_object, params);
    filter_predicted_paths(filtered_object, filtering_data[filtered_object.label]);
    if (!filtered_object.predicted_path_footprints.empty()) {
      filtered_objects.push_back(filtered_object);
    }
  }
  return filtered_objects;
}
}  // namespace autoware::motion_velocity_planner::run_out
