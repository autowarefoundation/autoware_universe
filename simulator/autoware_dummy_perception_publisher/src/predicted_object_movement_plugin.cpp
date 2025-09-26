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

#include "autoware/dummy_perception_publisher/predicted_object_movement_plugin.hpp"

#include "autoware/dummy_perception_publisher/movement_utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
namespace autoware::dummy_perception_publisher::pluginlib
{
using autoware_utils_geometry::calc_distance2d;

void PredictedObjectMovementPlugin::initialize()
{
  using autoware_utils_rclcpp::get_or_declare_parameter;
  set_associated_action_type(tier4_simulation_msgs::msg::DummyObject::PREDICT);
  // Declare prediction parameters
  auto node_ptr = get_node();
  predicted_object_params_.min_predicted_path_keep_duration =
    get_or_declare_parameter<double>(*node_ptr, "min_predicted_path_keep_duration");
  predicted_object_params_.switch_time_threshold =
    get_or_declare_parameter<double>(*node_ptr, "switch_time_threshold");

  // Initialize vehicle parameters
  predicted_object_params_.vehicle_params = {
    get_or_declare_parameter<double>(*node_ptr, "vehicle.max_remapping_distance"),
    get_or_declare_parameter<double>(*node_ptr, "vehicle.max_speed_difference_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "vehicle.min_speed_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "vehicle.max_speed_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "vehicle.speed_check_threshold"),
    get_or_declare_parameter<std::string>(*node_ptr, "vehicle.path_selection_strategy")};
  // Initialize pedestrian parameters
  predicted_object_params_.pedestrian_params = {
    get_or_declare_parameter<double>(*node_ptr, "pedestrian.max_remapping_distance"),
    get_or_declare_parameter<double>(*node_ptr, "pedestrian.max_speed_difference_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "pedestrian.min_speed_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "pedestrian.max_speed_ratio"),
    get_or_declare_parameter<double>(*node_ptr, "pedestrian.speed_check_threshold"),
    get_or_declare_parameter<std::string>(*node_ptr, "pedestrian.path_selection_strategy")};

  // Initialize random generator
  std::random_device seed_gen;
  random_generator_.seed(seed_gen());

  // Initialize predicted objects subscriber
  predicted_objects_sub_ = node_ptr->create_subscription<PredictedObjects>(
    "input/predicted_objects", 100,
    std::bind(
      &PredictedObjectMovementPlugin::predicted_objects_callback, this, std::placeholders::_1));
}

void PredictedObjectMovementPlugin::predicted_objects_callback(
  const PredictedObjects::ConstSharedPtr msg)
{
  // Add to buffer, removing oldest if necessary
  auto & predicted_objects_buffer = predicted_dummy_objects_tracking_info_.predicted_objects_buffer;
  if (predicted_objects_buffer.size() >= predicted_dummy_objects_tracking_info_.max_buffer_size) {
    predicted_objects_buffer.pop_front();
  }
  predicted_objects_buffer.push_back(*msg);

  // Update the dummy-to-predicted mapping based on euclidean distance
  update_dummy_to_predicted_mapping(objects_, *msg);
}
std::map<std::string, geometry_msgs::msg::Point>
PredictedObjectMovementPlugin::collect_dummy_object_positions(
  const std::vector<DummyObject> & dummy_objects, const rclcpp::Time & current_time,
  std::vector<std::string> & unmapped_dummy_uuids)
{
  std::map<std::string, Point> dummy_positions;
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;

  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);

    // Use last known position if available (which includes straight-line calculated position)
    // Otherwise calculate current position using straight-line model
    auto info_it = dummy_predicted_info_map.find(dummy_uuid_str);

    dummy_positions[dummy_uuid_str] =
      (info_it != dummy_predicted_info_map.end() && info_it->second.last_known_position.has_value())
        ? info_it->second.last_known_position.value()
        : utils::MovementUtils::calculate_straight_line_position(dummy_obj, current_time).position;

    if (info_it == dummy_predicted_info_map.end() || info_it->second.predicted_uuid.empty()) {
      unmapped_dummy_uuids.push_back(dummy_uuid_str);
    }
  }

  return dummy_positions;
}

void PredictedObjectMovementPlugin::update_dummy_to_predicted_mapping(
  const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
  const PredictedObjects & predicted_objects)
{
  const auto node_ptr = get_node();
  const rclcpp::Time current_time = node_ptr->now();

  // Create sets of available UUIDs
  std::map<std::string, Point> predicted_positions;
  auto available_predicted_uuids =
    collect_available_predicted_uuids(predicted_objects, predicted_positions);

  // Check for disappeared predicted objects and mark dummy objects for remapping
  auto dummy_objects_to_remap = find_disappeared_predicted_object_uuids(available_predicted_uuids);

  // Update dummy object positions and find unmapped dummy objects
  std::vector<std::string> unmapped_dummy_uuids;
  auto dummy_positions =
    collect_dummy_object_positions(dummy_objects, current_time, unmapped_dummy_uuids);

  // Handle remapping for dummy objects whose predicted objects disappeared
  create_remapping_for_disappeared_objects(
    dummy_objects_to_remap, available_predicted_uuids, predicted_positions, dummy_positions,
    predicted_objects);

  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;

  // Map unmapped dummy objects to closest available predicted objects
  for (const auto & dummy_uuid : unmapped_dummy_uuids) {
    if (available_predicted_uuids.empty()) {
      break;
    }

    const auto & dummy_pos = dummy_positions[dummy_uuid];
    auto best_match = find_best_predicted_object_match(
      dummy_uuid, dummy_pos, available_predicted_uuids, predicted_positions, predicted_objects);
    if (best_match) {
      dummy_predicted_info_map[dummy_uuid].predicted_uuid = *best_match;
      dummy_predicted_info_map[dummy_uuid].mapping_timestamp = current_time;
      available_predicted_uuids.erase(*best_match);
    }
  }

  std::set<std::string> current_dummy_uuids;
  for (const auto & dummy_obj : dummy_objects) {
    current_dummy_uuids.insert(autoware_utils_uuid::to_hex_string(dummy_obj.id));
  }

  // Clean up mappings for dummy objects that no longer exist
  for (auto it = dummy_predicted_info_map.begin(); it != dummy_predicted_info_map.end();) {
    if (current_dummy_uuids.find(it->first) != current_dummy_uuids.end()) {
      ++it;
      continue;
    }
    it = dummy_predicted_info_map.erase(it);
  }

  // Update last known positions for all dummy objects
  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);
    dummy_predicted_info_map[dummy_uuid_str].last_known_position =
      dummy_obj.initial_state.pose_covariance.pose.position;
  }
}

std::set<std::string> PredictedObjectMovementPlugin::collect_available_predicted_uuids(
  const PredictedObjects & predicted_objects,
  std::map<std::string, geometry_msgs::msg::Point> & predicted_positions)
{
  std::set<std::string> available_predicted_uuids;

  for (const auto & pred_obj : predicted_objects.objects) {
    const auto pred_uuid_str = autoware_utils_uuid::to_hex_string(pred_obj.object_id);
    available_predicted_uuids.insert(pred_uuid_str);
    predicted_positions[pred_uuid_str] =
      pred_obj.kinematics.initial_pose_with_covariance.pose.position;
  }

  return available_predicted_uuids;
}

std::vector<std::string> PredictedObjectMovementPlugin::find_disappeared_predicted_object_uuids(
  std::set<std::string> & available_predicted_uuids)
{
  std::vector<std::string> dummy_objects_to_remap;
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;
  for (const auto & mapping : dummy_predicted_info_map) {
    const std::string & dummy_uuid = mapping.first;
    const std::string & predicted_uuid = mapping.second.predicted_uuid;

    // Skip entries without a predicted UUID mapping
    if (predicted_uuid.empty()) {
      continue;
    }

    // If the predicted object ID no longer exists, mark dummy for remapping
    if (available_predicted_uuids.find(predicted_uuid) == available_predicted_uuids.end()) {
      dummy_objects_to_remap.push_back(dummy_uuid);
    } else {
      // Remove already assigned predicted objects from available set
      available_predicted_uuids.erase(predicted_uuid);
    }
  }

  return dummy_objects_to_remap;
}

void PredictedObjectMovementPlugin::create_remapping_for_disappeared_objects(
  const std::vector<std::string> & dummy_objects_to_remap,
  std::set<std::string> & available_predicted_uuids,
  const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
  const std::map<std::string, geometry_msgs::msg::Point> & dummy_positions,
  const PredictedObjects & predicted_objects)
{
  const auto node_ptr = get_node();
  const rclcpp::Time current_time = node_ptr->now();
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;
  // First, remove old mappings
  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    auto it = dummy_predicted_info_map.find(dummy_uuid);
    if (it != dummy_predicted_info_map.end()) {
      it->second.predicted_uuid.clear();
    }
  }

  // Find best matches for all objects that need remapping
  std::vector<std::pair<std::string, double>> mapping_candidates;

  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    // Use last known position if available, otherwise use current position
    Point remapping_position;
    auto current_pos_it = dummy_positions.find(dummy_uuid);
    if (current_pos_it == dummy_positions.end()) {
      continue;
    }
    auto info_it = dummy_predicted_info_map.find(dummy_uuid);

    remapping_position =
      (info_it != dummy_predicted_info_map.end() && info_it->second.last_known_position.has_value())
        ? info_it->second.last_known_position.value()
        : current_pos_it->second;
    // Find closest available predicted object for remapping
    auto best_match = find_best_predicted_object_match(
      dummy_uuid, remapping_position, available_predicted_uuids, predicted_positions,
      predicted_objects);

    if (best_match) {
      const double distance =
        calc_distance2d(remapping_position, predicted_positions.at(*best_match));
      mapping_candidates.emplace_back(dummy_uuid + ":" + *best_match, distance);
    }
  }

  // Sort candidates by distance to ensure closest matches get priority
  // This is because multiple dummy objects may have the same best match
  std::sort(
    mapping_candidates.begin(), mapping_candidates.end(),
    [](const auto & a, const auto & b) { return a.second < b.second; });

  // Create mappings in order of proximity, ensuring one-to-one mapping
  for (const auto & candidate : mapping_candidates) {
    const std::string & combined = candidate.first;
    const size_t colon_pos = combined.find(':');
    const std::string dummy_uuid = combined.substr(0, colon_pos);
    const std::string predicted_uuid = combined.substr(colon_pos + 1);

    // Only create mapping if predicted object is still available
    if (available_predicted_uuids.find(predicted_uuid) != available_predicted_uuids.end()) {
      dummy_predicted_info_map[dummy_uuid].predicted_uuid = predicted_uuid;
      dummy_predicted_info_map[dummy_uuid].mapping_timestamp = current_time;
      available_predicted_uuids.erase(predicted_uuid);
    }
  }
}

std::optional<std::string> PredictedObjectMovementPlugin::find_best_predicted_object_match(
  const std::string & dummy_uuid, const geometry_msgs::msg::Point & dummy_position,
  const std::set<std::string> & available_predicted_uuids,
  const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
  const PredictedObjects & predicted_objects)
{
  // Get the best matching predicted object based on several metrics
  std::string closest_pred_uuid;
  double min_distance = std::numeric_limits<double>::max();

  // Iterate over all available predicted objects to find the best match
  for (const auto & pred_uuid : available_predicted_uuids) {
    const auto & pred_pos = predicted_positions.at(pred_uuid);
    auto pred_obj = std::find_if(
      predicted_objects.objects.begin(), predicted_objects.objects.end(),
      [&pred_uuid](const auto & obj) {
        return autoware_utils_uuid::to_hex_string(obj.object_id) == pred_uuid;
      });

    // Handle case where predicted object is not found (should not happen)
    if (pred_obj == predicted_objects.objects.end()) {
      continue;
    }

    // Find the actual predicted object for validation
    const PredictedObject & candidate_pred_obj = *pred_obj;

    // In case of multiple valid candidates, choose the closest one
    double distance = calc_distance2d(dummy_position, pred_pos);

    // Check if there is a valid remapping candidate based on position and speed
    if (
      distance >= min_distance ||
      !is_valid_remapping_candidate(candidate_pred_obj, dummy_uuid, dummy_position)) {
      continue;
    }

    min_distance = distance;
    closest_pred_uuid = pred_uuid;
  }

  if (closest_pred_uuid.empty()) {
    return std::nullopt;
  }

  return closest_pred_uuid;
}

bool PredictedObjectMovementPlugin::is_valid_remapping_candidate(
  const PredictedObject & candidate_prediction, const std::string & dummy_uuid_str,
  const geometry_msgs::msg::Point & expected_position)
{
  // Perform various checks to validate if the candidate predicted object is a good match
  // for the dummy object
  auto dummy_it =
    std::find_if(objects_.begin(), objects_.end(), [&dummy_uuid_str](const auto & obj) {
      const auto uuid = autoware_utils_uuid::to_hex_string(obj.id);
      return uuid == dummy_uuid_str;
    });

  if (dummy_it == objects_.end()) {
    return false;
  }

  const auto & dummy_object = *dummy_it;
  // Check if this is a pedestrian object

  bool is_pedestrian =
    (dummy_object.classification.label ==
     autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);

  // Use class-specific thresholds
  // Pedestrians: more lenient due to unpredictable movement patterns
  // Vehicles: stricter for more predictable movement
  const auto & pedestrian_params = predicted_object_params_.pedestrian_params;
  const auto & vehicle_params = predicted_object_params_.vehicle_params;
  const double max_remapping_distance = is_pedestrian ? pedestrian_params.max_remapping_distance
                                                      : vehicle_params.max_remapping_distance;
  const double max_speed_difference_ratio = is_pedestrian
                                              ? pedestrian_params.max_speed_difference_ratio
                                              : vehicle_params.max_speed_difference_ratio;

  // Check if candidate has predicted paths
  if (candidate_prediction.kinematics.predicted_paths.empty()) {
    return false;
  }

  // Get dummy object speed for comparison (reuse dummy_it from above)
  const double dummy_speed = dummy_object.initial_state.twist_covariance.twist.linear.x;

  // Get candidate predicted object speed
  const auto & candidate_twist =
    candidate_prediction.kinematics.initial_twist_with_covariance.twist;
  const double candidate_speed = std::sqrt(
    candidate_twist.linear.x * candidate_twist.linear.x +
    candidate_twist.linear.y * candidate_twist.linear.y);

  // Speed bounds check - more lenient for pedestrians
  const double min_speed_ratio =
    is_pedestrian ? pedestrian_params.min_speed_ratio : vehicle_params.min_speed_ratio;
  const double max_speed_ratio =
    is_pedestrian ? pedestrian_params.max_speed_ratio : vehicle_params.max_speed_ratio;
  const double speed_check_threshold =
    is_pedestrian ? pedestrian_params.speed_check_threshold : vehicle_params.speed_check_threshold;

  auto is_within_speed_bounds = [&](double speed) {
    return speed >= min_speed_ratio * dummy_speed && speed <= max_speed_ratio * dummy_speed;
  };

  // Reject if candidate speed is too different when dummy speed is significant
  if (dummy_speed > speed_check_threshold && !is_within_speed_bounds(candidate_speed)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Rejecting remapping candidate for object %s (%s) due to speed difference: dummy=%fm/s, "
      "candidate=%fm/s",
      dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", dummy_speed,
      candidate_speed);
    return false;
  }

  // Compare speeds if both are significant
  if (dummy_speed > 0.1 && candidate_speed > 0.1) {
    const double speed_ratio =
      std::max(dummy_speed / candidate_speed, candidate_speed / dummy_speed);
    if (speed_ratio > max_speed_difference_ratio) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("dummy_perception_publisher"),
        "Rejecting remapping candidate for object %s (%s) due to speed difference: %fx (dummy: "
        "%fm/s, "
        "candidate: %fm/s, max_ratio: %f)",
        dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", speed_ratio, dummy_speed,
        candidate_speed, max_speed_difference_ratio);
      return false;
    }
  }

  // Calculate expected position based on last known trajectory if available
  Point comparison_position = expected_position;
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;
  auto info_it = dummy_predicted_info_map.find(dummy_uuid_str);
  if (
    info_it == dummy_predicted_info_map.end() ||
    !info_it->second.last_used_prediction.has_value()) {
    return true;  // No last prediction, allow the match
  }

  // Calculate where the dummy object should be based on its last known trajectory
  const auto & last_trajectory =
    info_it->second.last_used_prediction.value().kinematics.predicted_paths.at(0);
  const auto expected_pos = calculate_expected_position(last_trajectory, dummy_uuid_str);
  if (expected_pos.has_value()) {
    comparison_position = expected_pos.value();
  }

  // Check position similarity
  const auto & candidate_pos =
    candidate_prediction.kinematics.initial_pose_with_covariance.pose.position;
  const double distance = calc_distance2d(comparison_position, candidate_pos);

  if (distance > max_remapping_distance) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Rejecting remapping candidate for object %s (%s) due to large distance: %fm > %fm "
      "(expected: %f, %f, "
      "candidate: %f, %f)",
      dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", distance,
      max_remapping_distance, comparison_position.x, comparison_position.y, candidate_pos.x,
      candidate_pos.y);
    return false;
  }

  return true;
}

std::optional<geometry_msgs::msg::Point> PredictedObjectMovementPlugin::calculate_expected_position(
  const autoware_perception_msgs::msg::PredictedPath & last_prediction,
  const std::string & dummy_uuid_str)
{
  // Check if we have predicted paths
  if (last_prediction.path.empty()) {
    return std::nullopt;
  }

  // Find the dummy object by UUID
  auto dummy_it =
    std::find_if(objects_.begin(), objects_.end(), [&dummy_uuid_str](const auto & obj) {
      const auto uuid = autoware_utils_uuid::to_hex_string(obj.id);
      return uuid == dummy_uuid_str;
    });

  if (dummy_it == objects_.end()) {
    return std::nullopt;
  }

  const auto & dummy_object = *dummy_it;

  const auto & selected_path = last_prediction;

  if (selected_path.path.empty()) {
    return std::nullopt;
  }

  // If only one point in path, return that point
  if (selected_path.path.size() < 2) {
    return selected_path.path.back().position;
  }

  // Calculate elapsed time from last prediction to current time
  const auto node_ptr = get_node();
  const auto current_time = node_ptr->now();
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;
  auto info_it = dummy_predicted_info_map.find(dummy_uuid_str);
  if (
    info_it == dummy_predicted_info_map.end() ||
    !info_it->second.last_used_prediction_time.has_value()) {
    return std::nullopt;
  }

  const double elapsed_time =
    (current_time - info_it->second.last_used_prediction_time.value()).seconds();

  // Calculate distance traveled based on elapsed time and dummy object speed
  const double speed = dummy_object.initial_state.twist_covariance.twist.linear.x;
  const double distance_traveled = speed * elapsed_time;

  // Calculate cumulative distances along the path
  std::vector<double> cumulative_distances;
  cumulative_distances.reserve(selected_path.path.size());
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < selected_path.path.size(); ++i) {
    const auto & prev_pose = selected_path.path.at(i - 1);
    const auto & curr_pose = selected_path.path.at(i);

    const double dx = curr_pose.position.x - prev_pose.position.x;
    const double dy = curr_pose.position.y - prev_pose.position.y;
    const double dz = curr_pose.position.z - prev_pose.position.z;
    const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    cumulative_distances.push_back(cumulative_distances.back() + segment_length);
  }

  Point expected_position;

  if (distance_traveled <= 0.0) {
    // No movement, use initial position
    return std::nullopt;
  }
  if (distance_traveled >= cumulative_distances.back()) {
    // Extrapolate beyond the path end
    // Use the last two points to determine direction and extrapolate
    const auto & second_last_pose = selected_path.path.at(selected_path.path.size() - 2);
    const auto & last_pose = selected_path.path.back();

    // Calculate direction vector from second-last to last pose
    const double dx = last_pose.position.x - second_last_pose.position.x;
    const double dy = last_pose.position.y - second_last_pose.position.y;
    const double dz = last_pose.position.z - second_last_pose.position.z;
    const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (segment_length < std::numeric_limits<double>::epsilon()) {
      return last_pose.position;
    }
    // Normalize direction vector
    const double dir_x = dx / segment_length;
    const double dir_y = dy / segment_length;
    const double dir_z = dz / segment_length;

    // Extrapolate position
    const double overshoot_distance = distance_traveled - cumulative_distances.back();
    expected_position.x = last_pose.position.x + dir_x * overshoot_distance;
    expected_position.y = last_pose.position.y + dir_y * overshoot_distance;
    expected_position.z = last_pose.position.z + dir_z * overshoot_distance;
    return expected_position;
  }

  // Interpolate along the path
  for (size_t i = 1; i < cumulative_distances.size(); ++i) {
    if (distance_traveled > cumulative_distances.at(i)) {
      continue;
    }
    // Interpolate between path points i-1 and i
    const double segment_start_distance = cumulative_distances.at(i - 1);
    const double segment_end_distance = cumulative_distances.at(i);
    const double segment_length = segment_end_distance - segment_start_distance;

    if (segment_length < std::numeric_limits<double>::epsilon()) {
      return selected_path.path.at(i - 1).position;
    }
    const double interpolation_factor =
      (distance_traveled - segment_start_distance) / segment_length;

    const auto & start_pose = selected_path.path.at(i - 1);
    const auto & end_pose = selected_path.path.at(i);

    expected_position.x =
      start_pose.position.x + interpolation_factor * (end_pose.position.x - start_pose.position.x);
    expected_position.y =
      start_pose.position.y + interpolation_factor * (end_pose.position.y - start_pose.position.y);
    expected_position.z =
      start_pose.position.z + interpolation_factor * (end_pose.position.z - start_pose.position.z);
    return expected_position;
  }
  return selected_path.path.back().position;
}

std::pair<PredictedObject, rclcpp::Time>
PredictedObjectMovementPlugin::find_matching_predicted_object(
  const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time)
{
  PredictedObject empty_object;
  rclcpp::Time empty_time(0, 0, RCL_ROS_TIME);

  const auto & obj_uuid_str = autoware_utils_uuid::to_hex_string(object_id);

  // Check if this dummy object is mapped to a predicted object
  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;
  auto mapping_it = dummy_predicted_info_map.find(obj_uuid_str);
  if (mapping_it == dummy_predicted_info_map.end() || mapping_it->second.predicted_uuid.empty()) {
    return std::make_pair(empty_object, empty_time);
  }

  const std::string & mapped_predicted_uuid = mapping_it->second.predicted_uuid;

  // Check if we should keep using the current prediction for at least
  // min_predicted_path_keep_duration Check if we have a last used prediction and if we should keep
  // using it
  auto info_it = dummy_predicted_info_map.find(obj_uuid_str);
  if (info_it == dummy_predicted_info_map.end()) {
    // Create entry if it doesn't exist (needed for later updates)
    dummy_predicted_info_map[obj_uuid_str];
    info_it = dummy_predicted_info_map.find(obj_uuid_str);
  }

  if (
    info_it->second.last_used_prediction.has_value() &&
    info_it->second.prediction_update_timestamp.has_value()) {
    const double time_since_last_update =
      (current_time - info_it->second.prediction_update_timestamp.value()).seconds();

    // If less than min_predicted_path_keep_duration has passed since last update, keep using the
    // same prediction
    if (time_since_last_update < predicted_object_params_.min_predicted_path_keep_duration) {
      if (info_it->second.last_used_prediction_time.has_value()) {
        return std::make_pair(
          info_it->second.last_used_prediction.value(),
          info_it->second.last_used_prediction_time.value());
      }
    }
  }

  // Time to update: find the closest prediction in the past
  const auto & predicted_objects_buffer =
    predicted_dummy_objects_tracking_info_.predicted_objects_buffer;

  auto predicted_objects_buffer_itr = std::find_if(
    predicted_objects_buffer.rbegin(), predicted_objects_buffer.rend(),
    [&current_time](const auto & predicted_objects_msg) {
      const rclcpp::Time msg_time(predicted_objects_msg.header.stamp);
      return msg_time <= current_time;
    });

  // If no suitable past message found, return empty
  if (predicted_objects_buffer_itr == predicted_objects_buffer.rend()) {
    return std::make_pair(empty_object, empty_time);
  }

  const auto & predicted_objects_msg = *predicted_objects_buffer_itr;
  const rclcpp::Time msg_time(predicted_objects_msg.header.stamp);

  // Look for the mapped predicted object UUID
  auto predicted_object_itr = std::find_if(
    predicted_objects_msg.objects.begin(), predicted_objects_msg.objects.end(),
    [&mapped_predicted_uuid](const auto & obj) {
      return autoware_utils_uuid::to_hex_string(obj.object_id) == mapped_predicted_uuid;
    });

  if (predicted_object_itr == predicted_objects_msg.objects.end()) {
    return std::make_pair(empty_object, empty_time);
  }

  const auto & predicted_object = *predicted_object_itr;

  // Apply path selection strategy based on object type configuration
  PredictedObject modified_predicted_object = predicted_object;

  // Check if this is a pedestrian object
  const bool is_pedestrian = std::any_of(
    predicted_object.classification.begin(), predicted_object.classification.end(),
    [](const auto & classification) {
      return classification.label ==
             autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
    });

  // Determine path selection strategy based on object type
  const auto & pedestrian_params = predicted_object_params_.pedestrian_params;
  const auto & vehicle_params = predicted_object_params_.vehicle_params;
  const std::string path_selection_strategy = is_pedestrian
                                                ? pedestrian_params.path_selection_strategy
                                                : vehicle_params.path_selection_strategy;

  if (predicted_object.kinematics.predicted_paths.empty()) {
    return std::make_pair(empty_object, empty_time);
  }

  auto & paths = modified_predicted_object.kinematics.predicted_paths;
  if (path_selection_strategy == "random") {
    // Randomly select a path index
    const size_t num_paths = predicted_object.kinematics.predicted_paths.size();
    std::uniform_int_distribution<size_t> path_index_dist(0, num_paths - 1);
    const size_t random_path_index = path_index_dist(random_generator_);
    // Reorder paths to put the randomly selected path first
    std::swap(paths[0], paths[random_path_index]);

    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Randomly selected path %zu out of %zu for %s object %s", random_path_index, num_paths,
      is_pedestrian ? "pedestrian" : "vehicle", obj_uuid_str.c_str());
  } else if (path_selection_strategy == "highest_confidence") {
    // Find path with highest confidence and move it to first position
    auto max_confidence_it = std::max_element(
      paths.begin(), paths.end(),
      [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
    std::swap(paths[0], *max_confidence_it);

    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Selected most likely path (confidence: %.3f) for %s object %s", paths[0].confidence,
      is_pedestrian ? "pedestrian" : "vehicle", obj_uuid_str.c_str());
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Unknown path selection strategy '%s' for %s object %s. Using first path as is.",
      path_selection_strategy.c_str(), is_pedestrian ? "pedestrian" : "vehicle",
      obj_uuid_str.c_str());

    return std::make_pair(empty_object, empty_time);
  }

  // Store this as the new prediction to use for some seconds
  dummy_predicted_info_map[obj_uuid_str].last_used_prediction = modified_predicted_object;
  dummy_predicted_info_map[obj_uuid_str].last_used_prediction_time = msg_time;
  dummy_predicted_info_map[obj_uuid_str].prediction_update_timestamp = current_time;

  return std::make_pair(modified_predicted_object, msg_time);
}

std::vector<ObjectInfo> PredictedObjectMovementPlugin::move_objects()
{
  std::vector<ObjectInfo> obj_infos;
  const auto node_ptr = get_node();
  const auto current_time = node_ptr->now();

  for (const auto & object : objects_) {
    auto predicted_object_pair = find_matching_predicted_object(object.id, current_time);
    const auto & predicted_object = predicted_object_pair.first;
    const auto & predicted_time = predicted_object_pair.second;

    const bool matched_predicted =
      (!predicted_object.object_id.uuid.empty() &&
       !predicted_object.kinematics.predicted_paths.empty() && predicted_time.nanoseconds() > 0);
    auto & dummy_predicted_info_map =
      predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;

    ObjectInfo obj_info = [&]() {
      if (matched_predicted) {
        return create_object_info_with_predicted_path(
          object, predicted_object, predicted_time, current_time);
      }

      // Check if we have a last used prediction for this object
      const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
      auto info_it = dummy_predicted_info_map.find(dummy_uuid_str);

      if (
        info_it != dummy_predicted_info_map.end() &&
        info_it->second.last_used_prediction.has_value() &&
        info_it->second.last_used_prediction_time.has_value()) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("dummy_perception_publisher"),
          "Using last known prediction for lost object with ID: %s", dummy_uuid_str.c_str());
        return create_object_info_with_predicted_path(
          object, info_it->second.last_used_prediction.value(),
          info_it->second.last_used_prediction_time.value(), current_time);
      }

      RCLCPP_DEBUG(
        rclcpp::get_logger("dummy_perception_publisher"),
        "No matching predicted object found for dummy object with ID: %s", dummy_uuid_str.c_str());
      // Use straight-line motion for all other cases
      return create_object_info_with_straight_line(object, current_time);
    }();
    obj_infos.push_back(obj_info);
    // Update last known position based on calculated ObjectInfo position
    const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
    dummy_predicted_info_map[dummy_uuid_str].last_known_position =
      obj_info.pose_covariance_.pose.position;

    // Track object creation time if not already tracked
    auto & info = dummy_predicted_info_map[dummy_uuid_str];
    if (!info.creation_timestamp.has_value()) {
      info.creation_timestamp = rclcpp::Time(object.header.stamp);
    }
  }
  return obj_infos;
}

ObjectInfo PredictedObjectMovementPlugin::create_object_info_with_straight_line(
  const DummyObject & object, const rclcpp::Time & current_time) const
{
  // Create basic ObjectInfo with dimensions and covariances
  auto obj_info = utils::MovementUtils::create_basic_object_info(object);

  // Calculate position using straight-line movement
  const auto current_pose =
    utils::MovementUtils::calculate_straight_line_position(object, current_time);

  // Update ObjectInfo with the calculated movement
  utils::MovementUtils::update_object_info_with_movement(
    obj_info, object, current_pose, current_time);

  return obj_info;
}

ObjectInfo PredictedObjectMovementPlugin::create_object_info_with_predicted_path(
  const DummyObject & object, const PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time) const
{
  // Create basic ObjectInfo with dimensions and covariances
  auto obj_info = utils::MovementUtils::create_basic_object_info(object);

  // Check if threshold time has passed since object creation
  const double time_since_creation = (current_time - rclcpp::Time(object.header.stamp)).seconds();

  // Use straight-line movement for first switch_time_threshold seconds, then switch to predicted
  // path
  const auto current_pose =
    (time_since_creation < predicted_object_params_.switch_time_threshold ||
     predicted_object.kinematics.predicted_paths.empty())
      ? utils::MovementUtils::calculate_straight_line_position(object, current_time)
      : utils::MovementUtils::calculate_trajectory_based_position(
          object, predicted_object, predicted_time, current_time);

  // Update ObjectInfo with the calculated movement
  utils::MovementUtils::update_object_info_with_movement(
    obj_info, object, current_pose, current_time);

  // Use dummy object's velocity consistently
  obj_info.twist_covariance_.twist.linear.x = object.initial_state.twist_covariance.twist.linear.x;

  return obj_info;
}

}  // namespace autoware::dummy_perception_publisher::pluginlib
