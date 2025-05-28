
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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief parameters for the filtering of the dynamic objects
struct ObjectParameters
{
  bool ignore_if_stopped;
  double stopped_velocity_threshold;
  bool ignore_if_on_ego_trajectory;
  bool ignore_if_behind_ego;
  std::vector<std::string> ignore_objects_polygon_types;
  std::vector<std::string> ignore_objects_lanelet_subtypes;
  std::vector<std::string> ignore_collisions_polygon_types;
  std::vector<std::string> ignore_collisions_lanelet_subtypes;
  std::vector<std::string> cut_linestring_types;
  std::vector<std::string> cut_polygon_types;
  std::vector<std::string> cut_lanelet_subtypes;
  bool cut_if_crossing_ego_from_behind;
  double confidence_filtering_threshold;
  bool confidence_filtering_only_use_highest;
};

/// @brief conditions to ignore collisions
struct IgnoreCollisionConditionsParameters
{
  /// @brief collision where ego arrives first
  struct
  {
    bool enable;
    struct
    {
      std::vector<double>
        ego_enter_times;  // [s] predicted times when ego starts overlapping the path of the object
      std::vector<double>
        time_margins;  // [s] margin values used such that ego needs to enter the overlap before the
                       // object with this much margin to decide to ignore the collision
    } margin;
    double max_overlap_duration;  // [s] the collision is not ignored if ego is predicted to stay on
                                  // the object's path for longer than this duration
  } if_ego_arrives_first;
  /// @brief collision where ego arrives first and cannot stop before the collision
  struct
  {
    bool enable;
    double deceleration_limit;  // [m/s²] deceleration used to determine if ego can stop before a
                                // collision
    double calculated_stop_time_limit;  // [s] calculated from the deceleration limit and the
                                        // current ego velocity
  } if_ego_arrives_first_and_cannot_stop;
};

/// @brief Parameters of the run_out module
struct Parameters
{
  double
    max_history_duration;  // [s]  calculated as the maximum duration among all buffer parameters
  double stop_on_time_buffer;   // [s] successive collision detection time required to start the
                                // stopping decision
  double stop_off_time_buffer;  // [s] successive non-collision detection time required to remove a
                                // stopping decision
  double stop_distance_buffer;  // [m] longitudinal safety distance to keep between ego and the
                                // collision position
  double stop_deceleration_limit;  // [m/s²] if a stop requires a higher deceleration, then an ERROR
                                   // diagnostic is published
  double keep_stop_condition_time;  // [s] time along the ego trajectory where any collision will
                                    // make us keep a stop decision
  double keep_stop_condition_distance;  // [m] distance along the ego trajectory where any collision
                                        // will make us keep a stop decision

  double slowdown_on_time_buffer;      // [s] successive collision detection time required to
                                       // start the slowdown decision
  double slowdown_off_time_buffer;     // [s] successive non-collision detection time
                                       // required to remove the slowdown decision
  double slowdown_distance_buffer;     // [m] longitudinal distance between the collision
                                       // and the slowdown positions
  double slowdown_deceleration_limit;  // [m/s²] maximum deceleration that can be applied
                                       // by the preventive slowdown

  double ego_lateral_margin;                // [m] ego footprint lateral margin
  double ego_longitudinal_margin;           // [m] ego footprint longitudinal margin
  double collision_time_margin;             // [s] extra time margin to determine collisions
  double collision_time_overlap_tolerance;  // [s] when calculating overlap time intervals,
                                            // intervals are grouped if they are separated by less
                                            // than this tolerance value
  double
    collision_same_direction_angle_threshold;  // [rad] threshold to determine if a collision is
                                               // going in the same direction as the ego trajectory
  double collision_opposite_direction_angle_threshold;  // [rad] threshold to determine if a
                                                        // collision is going in a direction
                                                        // opposite to the ego trajectory
  IgnoreCollisionConditionsParameters ignore_collision_conditions;
  // object parameters
  std::vector<std::string> objects_target_labels;
  std::vector<ObjectParameters> object_parameters_per_label;

  struct
  {
    std::string object_label;
  } debug;

  /// @brief Get the parameter defined for a specific object label, or the default value if it was
  /// not specified
  template <class T>
  auto get_object_parameter(
    rclcpp::Node & node, const std::string & ns, const uint8_t object_label,
    const std::string & param)
  {
    using universe_utils::getOrDeclareParameter;
    const auto label_str = ".objects." + label_to_string(object_label);
    try {
      return getOrDeclareParameter<T>(node, ns + label_str + param);
    } catch (const std::exception &) {
      return getOrDeclareParameter<T>(node, ns + ".objects.DEFAULT" + param);
    }
  }
  /// @brief Initialize the parameters
  void initialize(rclcpp::Node & node, const std::string & ns)
  {
    using universe_utils::getOrDeclareParameter;
    const auto ignore_collisions_ns = ns + ".collision.ignore_conditions";
    ignore_collision_conditions.if_ego_arrives_first.enable =
      getOrDeclareParameter<bool>(node, ignore_collisions_ns + ".if_ego_arrives_first.enable");
    ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times =
      getOrDeclareParameter<std::vector<double>>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.margin.ego_enter_times");
    ignore_collision_conditions.if_ego_arrives_first.margin.time_margins =
      getOrDeclareParameter<std::vector<double>>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.margin.time_margins");
    ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration =
      getOrDeclareParameter<double>(
        node, ignore_collisions_ns + ".if_ego_arrives_first.max_overlap_duration");
    ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable =
      getOrDeclareParameter<bool>(
        node, ignore_collisions_ns + ".if_ego_arrives_first_and_cannot_stop.enable");
    ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit =
      getOrDeclareParameter<double>(
        node, ignore_collisions_ns + ".if_ego_arrives_first_and_cannot_stop.deceleration_limit");
    stop_off_time_buffer = getOrDeclareParameter<double>(node, ns + ".stop.off_time_buffer");
    stop_on_time_buffer = getOrDeclareParameter<double>(node, ns + ".stop.on_time_buffer");
    stop_distance_buffer = getOrDeclareParameter<double>(node, ns + ".stop.distance_buffer");
    stop_deceleration_limit = getOrDeclareParameter<double>(node, ns + ".stop.deceleration_limit");
    keep_stop_condition_time =
      getOrDeclareParameter<double>(node, ns + ".stop.keep_condition.time");
    keep_stop_condition_distance =
      getOrDeclareParameter<double>(node, ns + ".stop.keep_condition.distance");
    slowdown_off_time_buffer =
      getOrDeclareParameter<double>(node, ns + ".slowdown.off_time_buffer");
    slowdown_on_time_buffer = getOrDeclareParameter<double>(node, ns + ".slowdown.on_time_buffer");
    slowdown_distance_buffer =
      getOrDeclareParameter<double>(node, ns + ".slowdown.distance_buffer");
    slowdown_deceleration_limit =
      getOrDeclareParameter<double>(node, ns + ".slowdown.deceleration_limit");
    ego_lateral_margin = getOrDeclareParameter<double>(node, ns + ".ego.lateral_margin");
    ego_longitudinal_margin = getOrDeclareParameter<double>(node, ns + ".ego.longitudinal_margin");
    collision_time_margin = getOrDeclareParameter<double>(node, ns + ".collision.time_margin");
    collision_time_overlap_tolerance =
      getOrDeclareParameter<double>(node, ns + ".collision.time_overlap_tolerance");
    collision_same_direction_angle_threshold =
      getOrDeclareParameter<double>(node, ns + ".collision.same_direction_angle_threshold");
    collision_opposite_direction_angle_threshold =
      getOrDeclareParameter<double>(node, ns + ".collision.opposite_direction_angle_threshold");

    const auto all_object_labels = all_labels();
    object_parameters_per_label.resize(
      *std::max_element(all_object_labels.begin(), all_object_labels.end()) + 1);
    objects_target_labels =
      getOrDeclareParameter<std::vector<std::string>>(node, ns + ".objects.target_labels");
    for (const auto label : all_object_labels) {
      object_parameters_per_label[label].ignore_if_on_ego_trajectory =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_on_ego_trajectory");
      object_parameters_per_label[label].ignore_if_behind_ego =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_behind_ego");
      object_parameters_per_label[label].ignore_objects_polygon_types =
        get_object_parameter<std::vector<std::string>>(node, ns, label, ".ignore.polygon_types");
      object_parameters_per_label[label].ignore_objects_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(node, ns, label, ".ignore.lanelet_subtypes");
      object_parameters_per_label[label].ignore_collisions_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".ignore_collisions.polygon_types");
      object_parameters_per_label[label].ignore_collisions_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".ignore_collisions.lanelet_subtypes");
      object_parameters_per_label[label].ignore_if_stopped =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_stopped");
      object_parameters_per_label[label].stopped_velocity_threshold =
        get_object_parameter<double>(node, ns, label, ".ignore.stopped_velocity_threshold");
      object_parameters_per_label[label].confidence_filtering_threshold =
        get_object_parameter<double>(node, ns, label, ".confidence_filtering.threshold");
      object_parameters_per_label[label].confidence_filtering_only_use_highest =
        get_object_parameter<bool>(node, ns, label, ".confidence_filtering.only_use_highest");
      object_parameters_per_label[label].cut_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.polygon_types");
      object_parameters_per_label[label].cut_lanelet_subtypes =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.lanelet_subtypes");
      object_parameters_per_label[label].cut_linestring_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.linestring_types");
      object_parameters_per_label[label].cut_if_crossing_ego_from_behind =
        get_object_parameter<bool>(
          node, ns, label, ".cut_predicted_paths.if_crossing_ego_from_behind");
    }
    debug.object_label = getOrDeclareParameter<std::string>(node, ns + ".debug.object_label");

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief Update the parameters
  void update(const std::vector<rclcpp::Parameter> & params, const std::string & ns)
  {
    using universe_utils::updateParam;
    updateParam(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.enable",
      ignore_collision_conditions.if_ego_arrives_first.enable);
    updateParam(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.margin.ego_enter_times",
      ignore_collision_conditions.if_ego_arrives_first.margin.ego_enter_times);
    updateParam(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.margin.time_margins",
      ignore_collision_conditions.if_ego_arrives_first.margin.time_margins);
    updateParam(
      params, ns + ".collision.ignore_conditions.if_ego_arrives_first.max_overlap_duration",
      ignore_collision_conditions.if_ego_arrives_first.max_overlap_duration);
    updateParam(
      params, ns + ".collision.if_ego_arrives_first_and_cannot_stop.enable",
      ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.enable);
    updateParam(
      params, ns + ".collision.if_ego_arrives_first_and_cannot_stop.deceleration_limit",
      ignore_collision_conditions.if_ego_arrives_first_and_cannot_stop.deceleration_limit);
    updateParam(params, ns + ".slowdown.on_time_buffer", slowdown_on_time_buffer);
    updateParam(params, ns + ".slowdown.off_time_buffer", slowdown_off_time_buffer);
    updateParam(params, ns + ".slowdown.distance_buffer", slowdown_distance_buffer);
    updateParam(params, ns + ".slowdown.deceleration_limit", slowdown_deceleration_limit);
    updateParam(params, ns + ".stop.on_time_buffer", stop_on_time_buffer);
    updateParam(params, ns + ".stop.off_time_buffer", stop_off_time_buffer);
    updateParam(params, ns + ".stop.distance_buffer", stop_distance_buffer);
    updateParam(params, ns + ".stop.keep_condition.time", keep_stop_condition_time);
    updateParam(params, ns + ".stop.keep_condition.distance", keep_stop_condition_distance);
    updateParam(params, ns + ".ego.lateral_margin", ego_lateral_margin);
    updateParam(params, ns + ".ego.longitudinal_margin", ego_longitudinal_margin);
    updateParam(params, ns + ".collision.time_margin", collision_time_margin);
    updateParam(params, ns + ".collision.time_overlap_tolerance", collision_time_overlap_tolerance);
    updateParam(
      params, ns + ".collision.same_direction_angle_threshold",
      collision_same_direction_angle_threshold);
    updateParam(
      params, ns + ".collision.opposite_direction_angle_threshold",
      collision_opposite_direction_angle_threshold);

    updateParam(params, ns + ".objects.target_labels", objects_target_labels);

    for (const auto label : all_labels()) {
      const auto str = ".objects." + label_to_string(label);
      updateParam(
        params, ns + str + ".ignore.if_stopped",
        object_parameters_per_label[label].ignore_if_stopped);
      updateParam(
        params, ns + str + ".ignore.stopped_velocity_threshold",
        object_parameters_per_label[label].stopped_velocity_threshold);
      updateParam(
        params, ns + str + ".ignore.if_on_ego_trajectory",
        object_parameters_per_label[label].ignore_if_on_ego_trajectory);
      updateParam(
        params, ns + ".ignore.if_behind_ego",
        object_parameters_per_label[label].ignore_if_behind_ego);
      updateParam(
        params, ns + str + ".ignore.lanelet_subtypes",
        object_parameters_per_label[label].ignore_objects_lanelet_subtypes);
      updateParam(
        params, ns + str + ".ignore_collisions.lanelet_subtypes",
        object_parameters_per_label[label].ignore_collisions_lanelet_subtypes);
      updateParam(
        params, ns + str + ".ignore.polygon_types",
        object_parameters_per_label[label].ignore_objects_polygon_types);
      updateParam(
        params, ns + str + ".ignore_collisions.polygon_types",
        object_parameters_per_label[label].ignore_collisions_polygon_types);
      updateParam(
        params, ns + str + ".confidence_filtering.threshold",
        object_parameters_per_label[label].confidence_filtering_threshold);
      updateParam(
        params, ns + str + ".confidence_filtering.only_use_highest",
        object_parameters_per_label[label].confidence_filtering_only_use_highest);
      updateParam(
        params, ns + str + ".cut_predicted_paths.if_crossing_ego_from_behind",
        object_parameters_per_label[label].cut_if_crossing_ego_from_behind);
      updateParam(
        params, ns + str + ".cut_predicted_paths.lanelet_subtypes",
        object_parameters_per_label[label].cut_lanelet_subtypes);
      updateParam(
        params, ns + str + ".cut_predicted_paths.polygon_types",
        object_parameters_per_label[label].cut_polygon_types);
      updateParam(
        params, ns + str + ".cut_predicted_paths.linestring_types",
        object_parameters_per_label[label].cut_linestring_types);
    }
    updateParam(params, ns + ".debug.object_label", debug.object_label);

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief get a string representation of the given classification label
  static std::string label_to_string(
    const autoware_perception_msgs::msg::ObjectClassification::_label_type & label)
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    switch (label) {
      case ObjectClassification::CAR:
        return "CAR";
      case ObjectClassification::TRUCK:
        return "TRUCK";
      case ObjectClassification::BICYCLE:
        return "BICYCLE";
      case ObjectClassification::BUS:
        return "BUS";
      case ObjectClassification::MOTORCYCLE:
        return "MOTORCYCLE";
      case ObjectClassification::PEDESTRIAN:
        return "PEDESTRIAN";
      case ObjectClassification::TRAILER:
        return "TRAILER";
      case ObjectClassification::UNKNOWN:
        return "UNKNOWN";
      default:
        return "DEFAULT";
    }
  }
  /// @brief get the label corresponding to the given string
  static autoware_perception_msgs::msg::ObjectClassification::_label_type string_to_label(
    const std::string & label)
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    if (label == "CAR") {
      return ObjectClassification::CAR;
    }
    if (label == "TRUCK") {
      return ObjectClassification::TRUCK;
    }
    if (label == "BICYCLE") {
      return ObjectClassification::BICYCLE;
    }
    if (label == "BUS") {
      return ObjectClassification::BUS;
    }
    if (label == "MOTORCYCLE") {
      return ObjectClassification::MOTORCYCLE;
    }
    if (label == "PEDESTRIAN") {
      return ObjectClassification::PEDESTRIAN;
    }
    if (label == "TRAILER") {
      return ObjectClassification::TRAILER;
    }
    return ObjectClassification::UNKNOWN;
  }
  [[nodiscard]] std::vector<uint8_t> target_labels() const
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    std::vector<uint8_t> labels;
    for (uint8_t label = ObjectClassification::UNKNOWN; label <= ObjectClassification::PEDESTRIAN;
         ++label) {
      const auto is_target_label =
        std::find(
          objects_target_labels.begin(), objects_target_labels.end(),
          Parameters::label_to_string(label)) != objects_target_labels.end();
      if (is_target_label) {
        labels.push_back(label);
      }
    }
    return labels;
  }

  /// @brief get all possible classification labels
  static std::vector<uint8_t> all_labels()
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    std::vector<uint8_t> labels;
    for (uint8_t label = ObjectClassification::UNKNOWN; label <= ObjectClassification::PEDESTRIAN;
         ++label) {
      labels.push_back(label);
    }
    return labels;
  }
};
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // PARAMETERS_HPP_
