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
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.h>

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::multi_object_tracker
{
using ObjectsList = std::vector<types::DynamicObjectList>;

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

  std::vector<int64_t> can_assign_matrix;
  std::vector<double> max_dist_matrix;
  std::vector<double> max_area_matrix;
  std::vector<double> min_area_matrix;
  std::vector<double> min_iou_matrix;
  std::map<std::string, std::string> tracker_type_map;
  std::vector<double> pruning_giou_thresholds;
  std::vector<double> pruning_distance_thresholds;

  // Induced parameters
  TrackerProcessorConfig processor_config;
  AssociatorConfig associator_config;
};

struct MultiObjectTrackerInternalState
{
  std::unique_ptr<TrackerProcessor> processor;
  std::unique_ptr<InputManager> input_manager;
  std::shared_ptr<Odometry> odometry;

  rclcpp::Time last_published_time;
  rclcpp::Time last_updated_time;

  MultiObjectTrackerInternalState();

  void init(
    const MultiObjectTrackerParameters & params, rclcpp::Node & node,
    const std::function<void()> & trigger_function);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
};

namespace core
{

void process_objects(
  const types::DynamicObjectList & objects, const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params, MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger, const rclcpp::Logger & logger);

std::optional<ObjectsList> get_objects(
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state);

void process_parameters(MultiObjectTrackerParameters & params);

bool should_publish(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state);

autoware_perception_msgs::msg::TrackedObjects get_tracked_objects(
  const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params, MultiObjectTrackerInternalState & state);

std::optional<autoware_perception_msgs::msg::DetectedObjects> get_merged_objects(
  const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
  const MultiObjectTrackerParameters & params, MultiObjectTrackerInternalState & state,
  const rclcpp::Logger & logger);

void prune_objects(const rclcpp::Time & time, MultiObjectTrackerInternalState & state);

}  // namespace core

}  // namespace autoware::multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_CORE_HPP_
