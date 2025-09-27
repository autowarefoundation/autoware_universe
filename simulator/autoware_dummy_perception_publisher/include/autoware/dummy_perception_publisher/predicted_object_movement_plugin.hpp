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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__PREDICTED_OBJECT_MOVEMENT_PLUGIN_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__PREDICTED_OBJECT_MOVEMENT_PLUGIN_HPP_

#include "autoware/dummy_perception_publisher/dummy_object_movement_base_plugin.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <deque>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::dummy_perception_publisher::pluginlib
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_simulation_msgs::msg::DummyObject;

struct CommonParameters
{
  double max_remapping_distance;
  double max_speed_difference_ratio;
  double min_speed_ratio;
  double max_speed_ratio;
  double speed_check_threshold;
  std::string path_selection_strategy;  // "highest_confidence" or "random"
};

struct PredictedObjectParameters
{
  // Configuration parameters
  double predicted_path_delay{0.0};
  double min_predicted_path_keep_duration{0.0};
  double switch_time_threshold{0.0};

  // Vehicle parameters
  CommonParameters pedestrian_params;
  CommonParameters vehicle_params;
};

// Tracking info of a single dummy object and its mapping to a predicted object
struct PredictedDummyObjectInfo
{
  std::string predicted_uuid;
  std::optional<geometry_msgs::msg::Point> last_known_position;
  std::optional<rclcpp::Time> creation_timestamp;
  std::optional<PredictedObject> last_used_prediction;
  std::optional<rclcpp::Time> last_used_prediction_time;
  std::optional<rclcpp::Time> prediction_update_timestamp;
  std::optional<rclcpp::Time> mapping_timestamp;
};

// Struct that holds all tracking info, to track multiple NPCs moving using predicted objects'
// predicted paths
struct PredictedDummyObjectsTrackingInfo
{
  std::deque<PredictedObjects> predicted_objects_buffer;
  size_t max_buffer_size{50};  // Store last 5 seconds at 10Hz
  // mapping between dummy object UUID (string) and its tracking info
  std::map<std::string, PredictedDummyObjectInfo> dummy_predicted_info_map;
};
class PredictedObjectMovementPlugin : public DummyObjectMovementBasePlugin
{
public:
  explicit PredictedObjectMovementPlugin(rclcpp::Node * node) : DummyObjectMovementBasePlugin(node)
  {
    initialize();
  }
  void initialize() override;
  std::vector<ObjectInfo> move_objects() override;

private:
  rclcpp::Subscription<PredictedObjects>::SharedPtr predicted_objects_sub_;
  PredictedDummyObjectsTrackingInfo predicted_dummy_objects_tracking_info_;
  PredictedObjectParameters predicted_object_params_;
  std::mt19937 random_generator_;

  void predicted_objects_callback(const PredictedObjects::ConstSharedPtr msg);
  std::pair<PredictedObject, rclcpp::Time> find_matching_predicted_object(
    const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time);
  void update_dummy_to_predicted_mapping(
    const std::vector<DummyObject> & dummy_objects, const PredictedObjects & predicted_objects);

  [[nodiscard]] bool is_valid_remapping_candidate(
    const PredictedObject & candidate_prediction, const std::string & dummy_uuid_str,
    const geometry_msgs::msg::Point & expected_position);
  std::optional<geometry_msgs::msg::Point> calculate_expected_position(
    const autoware_perception_msgs::msg::PredictedPath & last_prediction,
    const std::string & dummy_uuid_str);

  static std::set<std::string> collect_available_predicted_uuids(
    const PredictedObjects & predicted_objects,
    std::map<std::string, geometry_msgs::msg::Point> & predicted_positions);
  std::vector<std::string> find_disappeared_predicted_object_uuids(
    std::set<std::string> & available_predicted_uuids);
  std::map<std::string, geometry_msgs::msg::Point> collect_dummy_object_positions(
    const std::vector<DummyObject> & dummy_objects, const rclcpp::Time & current_time,
    std::vector<std::string> & unmapped_dummy_uuids);
  std::optional<std::string> find_best_predicted_object_match(
    const std::string & dummy_uuid, const geometry_msgs::msg::Point & dummy_position,
    const std::set<std::string> & available_predicted_uuids,
    const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
    const PredictedObjects & predicted_objects);
  void create_remapping_for_disappeared_objects(
    const std::vector<std::string> & dummy_objects_to_remap,
    std::set<std::string> & available_predicted_uuids,
    const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
    const std::map<std::string, geometry_msgs::msg::Point> & dummy_positions,
    const PredictedObjects & predicted_objects);

  // Helper methods for creating ObjectInfo
  [[nodiscard]] ObjectInfo create_object_info_with_straight_line(
    const DummyObject & object, const rclcpp::Time & current_time) const;
  [[nodiscard]] ObjectInfo create_object_info_with_predicted_path(
    const DummyObject & object, const PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time) const;
};

}  // namespace autoware::dummy_perception_publisher::pluginlib

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__PREDICTED_OBJECT_MOVEMENT_PLUGIN_HPP_
