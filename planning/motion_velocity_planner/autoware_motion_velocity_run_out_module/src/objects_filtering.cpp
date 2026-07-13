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

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/overlaps/interface.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
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
  Object & object, const std::optional<DecisionHistory> & prev_decisions, const bool is_uncertain,
  const universe_utils::Segment2d & ego_rear_segment, const FilteringData & filtering_data,
  const Parameters & parameters)
{
  constexpr auto skip_object = true;
  const auto rear_vector = ego_rear_segment.second - ego_rear_segment.first;
  // normal vector in the direction coming from the rear
  const auto rear_normal = universe_utils::Point2d(-rear_vector.y(), rear_vector.x());
  const auto object_vector = object.position - ego_rear_segment.first;
  const auto is_behind_ego = rear_normal.dot(object_vector) < 0.0;
  const auto & params = parameters.object_parameters_per_label[object.label];
  if (params.ignore_if_behind_ego && is_behind_ego) {
    return skip_object;
  }
  const auto & is_previous_target =
    prev_decisions && (prev_decisions->decisions.back().type == stop ||
                       (prev_decisions->decisions.back().collision.has_value() &&
                        prev_decisions->decisions.back().collision->type == collision));
  if (!object.has_target_label) {
    return skip_object;
  }
  if (is_previous_target) {
    return !skip_object;
  }
  if (params.ignore_if_stopped && object.is_stopped && !is_uncertain) {
    return skip_object;
  }
  if (!filtering_data.ignore_objects_rtree.is_geometry_disjoint_from_rtree_polygons(
        object.current_footprint, filtering_data.ignore_objects_polygons)) {
    if (params.preserved_duration == 0.0 && params.preserved_distance == 0.0) {
      return skip_object;
    }
    object.ignore_but_preserve_predicted_paths = true;
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
  Object & object, const autoware_perception_msgs::msg::PredictedObject & predicted_object)
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
  for (const auto & path : predicted_object.kinematics.predicted_paths) {
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

void recalculate_predicted_paths(
  autoware_perception_msgs::msg::PredictedObject & object, const double min_vel)
{
  for (auto & path : object.kinematics.predicted_paths) {
    auto base_path = path.path;
    path.path.clear();
    std::vector<double> arc_lengths = {0.0};
    std::vector<double> recalculated_arc_lengths = {0.0};
    arc_lengths.reserve(base_path.size());
    const auto min_ds = min_vel * rclcpp::Duration(path.time_step).seconds();
    for (size_t i = 0; i + 1 < base_path.size(); ++i) {
      const auto ds = autoware_utils_geometry::calc_distance2d(base_path[i], base_path[i + 1]);
      arc_lengths.push_back(arc_lengths.back() + ds);
      recalculated_arc_lengths.push_back(recalculated_arc_lengths.back() + std::max(min_ds, ds));
    }
    if (recalculated_arc_lengths.back() > arc_lengths.back()) {
      const auto last_ds = arc_lengths.back() - arc_lengths[arc_lengths.size() - 2];
      const auto ratio = (last_ds + recalculated_arc_lengths.back() - arc_lengths.back()) / last_ds;
      const auto extended_x = autoware::interpolation::lerp(
        base_path[base_path.size() - 2].position.x, base_path[base_path.size() - 1].position.y,
        ratio);
      const auto extended_y = autoware::interpolation::lerp(
        base_path[base_path.size() - 2].position.y, base_path[base_path.size() - 1].position.y,
        ratio);
      auto extended_pose = base_path.back();
      extended_pose.position.x = extended_x;
      extended_pose.position.y = extended_y;
      base_path.push_back(extended_pose);
      arc_lengths.push_back(recalculated_arc_lengths.back());
    }
    for (const auto s : recalculated_arc_lengths) {
      path.path.push_back(autoware::motion_utils::calcInterpolatedPose(base_path, s));
    }
  }
}

void cut_predicted_path_footprint(
  ObjectPredictedPathFootprint & footprint, const size_t cut_index,
  const double standstill_duration_after_cut)
{
  const auto standstill_indexes =
    static_cast<size_t>(standstill_duration_after_cut / footprint.time_step);
  for (const auto pos : {front_left, front_right, rear_left, rear_right}) {
    footprint.predicted_path_footprint.corner_linestrings[pos].resize(cut_index);
    if (cut_index > 0UL) {
      footprint.predicted_path_footprint.corner_linestrings[pos].resize(
        cut_index + standstill_indexes,
        footprint.predicted_path_footprint.corner_linestrings[pos].back());
    }
  }
}

std::optional<size_t> get_cut_predicted_path_index(
  const autoware::motion_velocity_planner::run_out::ObjectPredictedPathFootprint & path,
  const SegmentRtree & cut_segments_rtree)
{
  const auto crosses_cut_line_in_map = [&](const auto & segment) {
    std::vector<SegmentNode> query_results;
    cut_segments_rtree.query(
      boost::geometry::index::intersects(segment), std::back_inserter(query_results));
    for (const auto & candidate : query_results) {
      if (universe_utils::intersect(
            segment.first, segment.second, candidate.first.first, candidate.first.second)) {
        return true;
      }
    }
    return false;
  };
  std::optional<size_t> cut_index;
  for (const auto & corner : {front_left, front_right, rear_left, rear_right}) {
    const auto & ls = path.predicted_path_footprint.corner_linestrings[corner];
    for (auto i = 0UL; i + 1 < ls.size(); ++i) {
      const auto & segment = universe_utils::Segment2d(ls[i], ls[i + 1]);
      if (crosses_cut_line_in_map(segment)) {
        cut_index = std::min(cut_index.value_or(i), i);
        break;
      }
    }
  }
  return cut_index;
}

size_t get_preserved_cut_index(
  const autoware::motion_velocity_planner::run_out::ObjectPredictedPathFootprint & path,
  const double time, const double distance)
{
  if (time == 0.0 && distance == 0.0) {
    return 0UL;
  }
  double t = 0.0;
  double d = 0.0;
  const auto & ls = path.predicted_path_footprint.corner_linestrings[front_left];
  for (auto i = 0UL; i + 1 < ls.size(); ++i) {
    const auto & segment = universe_utils::Segment2d(ls[i], ls[i + 1]);
    t += path.time_step;
    d += static_cast<double>(boost::geometry::length(segment));
    if ((time == 0.0 || t >= time) && (distance == 0.0 || d >= distance)) {
      return i + 1;
    }
  }
  return path.predicted_path_footprint.size() == 0 ? 0UL : path.predicted_path_footprint.size() - 1;
}

void filter_predicted_paths(
  Object & object, const FilteringData & map_data, const ObjectParameters & params)
{
  for (auto & predicted_path_footprint : object.predicted_path_footprints) {
    const auto strict_cut_index = get_cut_predicted_path_index(
      predicted_path_footprint, map_data.strict_cut_predicted_paths_rtree);
    // the preserved index is a lower bound that can only be reduced by the strict cut index
    const auto preserved_index = get_preserved_cut_index(
      predicted_path_footprint, params.preserved_duration, params.preserved_distance);
    const auto normal_cut_index =
      get_cut_predicted_path_index(predicted_path_footprint, map_data.cut_predicted_paths_rtree);
    std::optional<size_t> cut_index;
    if (object.ignore_but_preserve_predicted_paths) {
      // the normal cut is ignored but the strict cut will still be applied
      cut_index = preserved_index;
    } else if (normal_cut_index) {
      // apply the preserved index
      cut_index = std::max(*normal_cut_index, preserved_index);
    }
    if (strict_cut_index) {  // apply the strict cut index
      cut_index = std::min(*strict_cut_index, cut_index.value_or(*strict_cut_index));
    }
    if (cut_index) {
      cut_predicted_path_footprint(
        predicted_path_footprint, *cut_index, params.standstill_duration_after_cut);
    }
  }
}

std::vector<Object> prepare_dynamic_objects(
  const std::vector<std::shared_ptr<motion_velocity_planner::PlannerData::Object>> & objects,
  const TrajectoryCornerFootprint & ego_trajectory,
  const ObjectDecisionsTracker & previous_decisions,
  const ObjectDetectionTracker & detection_tracker, const FilteringDataPerLabel & filtering_data,
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
    const auto & detection_duration =
      detection_tracker.get_detection_duration(filtered_object.uuid);
    const auto is_uncertain =
      detection_duration &&
      *detection_duration < params.objects_uncertain_mode_detection_duration_threshold;
    const auto & previous_object_decisions = previous_decisions.get(filtered_object.uuid);
    if (skip_object_condition(
          filtered_object, previous_object_decisions, is_uncertain, ego_rear_segment,
          filtering_data[filtered_object.label], params)) {
      continue;
    }
    auto predicted_object = object->predicted_object;
    predicted_object.kinematics.predicted_paths = filter_by_confidence(
      predicted_object.kinematics.predicted_paths, filtered_object.label, params);
    if (is_uncertain) {
      recalculate_predicted_paths(predicted_object, params.objects_uncertain_mode_min_velocity);
    }
    calculate_predicted_path_footprints(filtered_object, predicted_object);
    filter_predicted_paths(
      filtered_object, filtering_data[filtered_object.label],
      params.object_parameters_per_label[filtered_object.label]);
    if (!filtered_object.predicted_path_footprints.empty()) {
      filtered_objects.push_back(filtered_object);
    }
  }
  return filtered_objects;
}
}  // namespace autoware::motion_velocity_planner::run_out
