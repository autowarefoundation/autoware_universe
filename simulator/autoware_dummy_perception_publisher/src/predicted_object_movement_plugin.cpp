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

#include <map>
#include <set>
#include <string>
#include <vector>
namespace autoware::dummy_perception_publisher::pluginlib
{

void PredictedObjectMovementPlugin::initialize()
{
  // Declare prediction parameters
  auto node_ptr = get_node();
  predicted_object_params_.min_predicted_path_keep_duration =
    node_ptr->declare_parameter("min_predicted_path_keep_duration", 3.0);
  predicted_object_params_.switch_time_threshold =
    node_ptr->declare_parameter("switch_time_threshold", 2.0);

  // Initialize vehicle parameters
  predicted_object_params_.vehicle_params = {
    node_ptr->declare_parameter("vehicle.max_remapping_distance", 2.0),
    node_ptr->declare_parameter("vehicle.max_speed_difference_ratio", 1.05),
    node_ptr->declare_parameter("vehicle.min_speed_ratio", 0.5),
    node_ptr->declare_parameter("vehicle.max_speed_ratio", 1.5),
    node_ptr->declare_parameter("vehicle.speed_check_threshold", 1.0),
    node_ptr->declare_parameter(
      "vehicle.path_selection_strategy", std::string("highest_confidence"))};
  // Initialize pedestrian parameters
  predicted_object_params_.pedestrian_params = {
    node_ptr->declare_parameter("pedestrian.max_remapping_distance", 3.0),
    node_ptr->declare_parameter("pedestrian.max_speed_difference_ratio", 1.3),
    node_ptr->declare_parameter("pedestrian.min_speed_ratio", 0.3),
    node_ptr->declare_parameter("pedestrian.max_speed_ratio", 2.0),
    node_ptr->declare_parameter("pedestrian.speed_check_threshold", 0.5),
    node_ptr->declare_parameter("pedestrian.path_selection_strategy", std::string("random"))};

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

void PredictedObjectMovementPlugin::update_dummy_to_predicted_mapping(
  const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
  const PredictedObjects & predicted_objects)
{
  const auto node_ptr = get_node();
  const rclcpp::Time current_time = node_ptr->now();

  // Create sets of available UUIDs
  std::map<std::string, Point> predicted_positions;
  std::set<std::string> available_predicted_uuids =
    collectAvailablePredictedUUIDs(predicted_objects, predicted_positions);

  // Check for disappeared predicted objects and mark dummy objects for remapping
  std::vector<std::string> dummy_objects_to_remap =
    findDisappearedPredictedObjectUUIDs(available_predicted_uuids);

  // Update dummy object positions and find unmapped dummy objects
  std::vector<std::string> unmapped_dummy_uuids;
  std::map<std::string, Point> dummy_positions =
    collectDummyObjectPositions(dummy_objects, current_time, unmapped_dummy_uuids);

  // Handle remapping for dummy objects whose predicted objects disappeared
  createRemappingsForDisappearedObjects(
    dummy_objects_to_remap, available_predicted_uuids, predicted_positions, dummy_positions,
    predicted_objects);

  auto & dummy_predicted_info_map = predicted_dummy_objects_tracking_info_.dummy_predicted_info_map;

  // Map unmapped dummy objects to closest available predicted objects
  for (const auto & dummy_uuid : unmapped_dummy_uuids) {
    if (available_predicted_uuids.empty()) {
      break;
    }

    const auto & dummy_pos = dummy_positions[dummy_uuid];
    auto best_match = findBestPredictedObjectMatch(
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

}  // namespace autoware::dummy_perception_publisher::pluginlib
