// Copyright 2026 TIER IV, Inc.
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

#ifndef MULTI_OBJECT_TRACKER_CORE_HPP_
#define MULTI_OBJECT_TRACKER_CORE_HPP_

#include "autoware/multi_object_tracker/association/association.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/odometry.hpp"
#include "debugger/debugger.hpp"
#include "processor/input_manager.hpp"
#include "processor/processor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2_ros/buffer.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{
struct MultiObjectTrackerParameters
{
  // Given parameters
  double publish_rate;
  std::string world_frame_id;
  std::string ego_frame_id;
  bool enable_delay_compensation;
  bool enable_odometry_uncertainty;
  bool publish_processing_time_detail;
  bool publish_merged_objects;

  std::vector<types::InputChannel> input_channels_config;

  AssociatorConfig::LabelToTrackerAssociationParametersMap association_params_map;
  std::map<std::string, std::string> tracker_type_map;
  TrackedLabelThresholds pruning_giou_thresholds;
  TrackedLabelThresholds pruning_distance_thresholds;

  // Induced parameters
  TrackerProcessorConfig processor_config;
  AssociatorConfig associator_config;
};

struct MultiObjectTrackerInternalState
{
  std::unique_ptr<TrackerProcessor> processor;
  std::unique_ptr<InputManager> input_manager;
  std::shared_ptr<Odometry> odometry;

  rclcpp::Time last_publish_time;
  rclcpp::Time last_updated_time;
  rclcpp::Time last_tracker_time;

  MultiObjectTrackerInternalState();

  void init(
    const MultiObjectTrackerParameters & params, rclcpp::Node & node,
    const std::function<void(size_t)> & trigger_function);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
};

namespace core
{

// Result structs for core functions
struct MeasurementProcessingResult
{
  bool has_objects;     // true if objects were accepted from InputManager
  bool should_process;  // true if this is the target channel
};

struct ObjectProcessingResult
{
  bool should_publish;  // true if should publish immediately (no delay compensation)
};

struct PublishingData
{
  rclcpp::Time object_time;
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  size_t tracked_objects_size;
};

struct OptionalPublishingData
{
  double min_extrapolation_time;
  std::optional<autoware_perception_msgs::msg::DetectedObjects> merged_objects;
  std::optional<autoware_perception_msgs::msg::TrackedObjects> tentative_objects;
};

// Parameter processing
void process_parameters(MultiObjectTrackerParameters & params);

// Utility functions
bool should_publish(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state);

autoware_perception_msgs::msg::TrackedObjects get_tracked_objects_(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state);

std::optional<autoware_perception_msgs::msg::DetectedObjects> get_merged_objects_(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state, const rclcpp::Logger & logger);

// Low-level processing functions
MeasurementProcessingResult process_measurement(
  const size_t channel_index,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg,
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger);

void process_objects_(
  const types::ObjectsWithAssociation & objects_with_associations,
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state,
  const rclcpp::Logger & logger);

// High-level orchestration functions
ObjectProcessingResult process_objects_batch(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state, TrackerDebugger & debugger,
  const rclcpp::Logger & logger);

PublishingData prepare_publishing_data(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state, const rclcpp::Logger & logger);

OptionalPublishingData prepare_optional_publishing_data(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state, const TrackerDebugger & debugger,
  const rclcpp::Logger & logger);

}  // namespace core

}  // namespace autoware::multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_CORE_HPP_
