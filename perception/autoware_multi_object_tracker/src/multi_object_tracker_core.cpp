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

#include "multi_object_tracker_core.hpp"

#include <tf2_ros/create_timer_interface.hpp>

#include <tf2_ros/create_timer_ros.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

MultiObjectTrackerInternalState::MultiObjectTrackerInternalState()
: last_publish_time(0, 0, RCL_ROS_TIME),
  last_updated_time(0, 0, RCL_ROS_TIME),
  last_tracker_time(0, 0, RCL_ROS_TIME)
{
}

void MultiObjectTrackerInternalState::init(
  const MultiObjectTrackerParameters & params, rclcpp::Node & node,
  const std::function<void(size_t)> & trigger_function)
{
  tf_buffer = std::make_shared<tf2_ros::Buffer>(node.get_clock());
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    node.get_node_base_interface(), node.get_node_timers_interface());
  tf_buffer->setCreateTimerInterface(cti);

  odometry = std::make_shared<Odometry>(
    node.get_logger(), node.get_clock(), tf_buffer, params.world_frame_id, params.ego_frame_id,
    params.enable_odometry_uncertainty);

  // Initialize input manager
  input_manager = std::make_unique<InputManager>(odometry, node.get_logger(), node.get_clock());
  input_manager->init(params.input_channels_config);
  input_manager->setTriggerFunction(trigger_function);

  // Initialize processor
  processor = std::make_unique<TrackerProcessor>(
    params.processor_config, params.associator_config, params.input_channels_config);

  last_publish_time = node.now();
  last_updated_time = node.now();
  last_tracker_time = node.now();
}

namespace core
{

// Parameter processing
void process_parameters(MultiObjectTrackerParameters & params)
{
  using Label = classes::Label;

  auto getTrackerType = [&params](const std::string & classification) -> TrackerType {
    const auto tracker_name_it = params.tracker_type_map.find(classification);
    const auto parameter_name = "initial_tracker." + classification;
    if (tracker_name_it == params.tracker_type_map.end()) {
      throw std::runtime_error("Missing tracker parameter: " + parameter_name);
    }

    const auto tracker_type = toTrackerType(tracker_name_it->second);
    if (!tracker_type.has_value()) {
      throw std::runtime_error(
        "Invalid tracker type: '" + tracker_name_it->second + "' for parameter '" + parameter_name +
        "'. Strict string match is required.");
    }
    return *tracker_type;
  };

  // Set the tracker map for processor config
  params.processor_config.tracker_map = {
    {Label::CAR, getTrackerType("car")},
    {Label::TRUCK, getTrackerType("truck")},
    {Label::BUS, getTrackerType("bus")},
    {Label::TRAILER, getTrackerType("trailer")},
    {Label::PEDESTRIAN, getTrackerType("pedestrian")},
    {Label::BICYCLE, getTrackerType("bicycle")},
    {Label::MOTORCYCLE, getTrackerType("motorcycle")},
    {Label::UNKNOWN, TrackerType::POLYGON}};
  // Set the pruning thresholds for processor config
  params.processor_config.pruning_giou_thresholds = params.pruning_giou_thresholds.to_label_map();
  params.processor_config.pruning_distance_thresholds =
    params.pruning_distance_thresholds.to_label_map();
  params.processor_config.pruning_distance_thresholds_sq.clear();
  params.processor_config.pruning_distance_thresholds_sq.reserve(
    params.processor_config.pruning_distance_thresholds.size());
  for (const auto & [label, threshold] : params.processor_config.pruning_distance_thresholds) {
    params.processor_config.pruning_distance_thresholds_sq.emplace(label, threshold * threshold);
  }

  for (const auto measurement_label : classes::trackedLabels()) {
    const auto label_params_opt =
      get_map_value_if_exists(params.association_params_map, measurement_label);
    if (!label_params_opt || label_params_opt->get().empty()) {
      throw std::runtime_error(
        "Missing association configuration for measurement label: " +
        classes::toString(measurement_label));
    }

    const auto & label_params = label_params_opt->get();

    const auto default_tracker_type_opt =
      get_map_value_if_exists(params.processor_config.tracker_map, measurement_label);
    if (!default_tracker_type_opt) {
      throw std::runtime_error(
        "Missing default tracker mapping for measurement label: " +
        classes::toString(measurement_label));
    }

    const auto default_tracker_type = default_tracker_type_opt->get();
    if (!get_map_value_if_exists(label_params, default_tracker_type)) {
      throw std::runtime_error(
        "Inconsistent configuration: default tracker '" + toString(default_tracker_type) +
        "' for measurement label '" + classes::toString(measurement_label) +
        "' is not included in association.can_assign." + classes::toString(measurement_label));
    }
  }

  params.associator_config.association_params_map = params.association_params_map;
}

// Utility functions
bool should_publish(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state)
{
  if (state.last_updated_time.nanoseconds() == 0) {
    state.last_updated_time = current_time;
  }

  // ensure minimum interval: room for the next process(prediction)
  static constexpr double minimum_publish_interval_ratio = 0.85;
  static constexpr double maximum_publish_interval_ratio = 1.05;

  const double publisher_period = 1.0 / params.publish_rate;
  const double minimum_publish_interval = publisher_period * minimum_publish_interval_ratio;
  const auto elapsed_time = (current_time - state.last_publish_time).seconds();

  if (elapsed_time < minimum_publish_interval) {
    return false;
  }

  // if there was update after publishing, publish new messages
  bool should_publish = state.last_publish_time < state.last_updated_time;

  // if there was no update, publish if the elapsed time is longer than the maximum publish latency
  // in this case, it will perform extrapolate/remove old objects
  const double maximum_publish_interval = publisher_period * maximum_publish_interval_ratio;
  should_publish = should_publish || elapsed_time > maximum_publish_interval;

  return should_publish;
}

autoware_perception_msgs::msg::TrackedObjects get_tracked_objects_(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state)
{
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  tracked_objects.header.frame_id = params.world_frame_id;
  state.processor->getTrackedObjects(object_time, tracked_objects);

  return tracked_objects;
}

std::optional<autoware_perception_msgs::msg::DetectedObjects> get_merged_objects_(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state, const rclcpp::Logger & logger)
{
  if (!params.publish_merged_objects) {
    return std::nullopt;
  }

  const auto & last_tracker_time = state.last_tracker_time;

  const auto tf_base_to_world = state.odometry->getTransform(last_tracker_time);
  if (tf_base_to_world) {
    autoware_perception_msgs::msg::DetectedObjects merged_output_msg;
    state.processor->getMergedObjects(object_time, *tf_base_to_world, merged_output_msg);
    merged_output_msg.header.frame_id = params.ego_frame_id;
    return merged_output_msg;
  }

  RCLCPP_WARN(
    logger, "No odometry information available at the publishing time: %.9f",
    last_tracker_time.seconds());
  return std::nullopt;
}

// Low-level processing functions
MeasurementProcessingResult process_measurement(
  const size_t channel_index,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg,
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state,
  TrackerDebugger & debugger)
{
  MeasurementProcessingResult result;
  result.has_objects = false;
  result.should_process = false;

  const auto objects = state.input_manager->processMessage(channel_index, msg);
  if (!objects) {
    return result;
  }

  const auto association_result = state.processor->associate(*objects);
  state.input_manager->push(channel_index, *objects, association_result);

  result.has_objects = true;
  result.should_process = (channel_index == state.input_manager->getTargetChannelIdx());

  const auto measurement_time = rclcpp::Time(objects->header.stamp, current_time.get_clock_type());

  // Collect debug information - tracker list, existence probabilities, association results
  const types::AssociatedObjects associated_objects{*objects, association_result};
  debugger.collectObjectInfo(
    measurement_time, state.processor->getListTracker(), associated_objects);

  return result;
}

void process_objects_(
  const types::ObjectsWithAssociation & objects_with_associations,
  const rclcpp::Time & current_time, MultiObjectTrackerInternalState & state,
  const rclcpp::Logger & logger)
{
  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    objects_with_associations.getTimestamp(current_time.get_clock_type());

  std::optional<geometry_msgs::msg::Pose> ego_pose;
  if (const auto odometry_info = state.odometry->getOdometryFromTf(measurement_time)) {
    ego_pose = odometry_info->pose.pose;
  }

  if (!ego_pose) {
    RCLCPP_WARN(
      logger, "No odometry information available at the measurement time: %.9f",
      measurement_time.seconds());
  }

  /* predict trackers to the measurement time */
  state.processor->predict(measurement_time, ego_pose);

  /* object association */
  const types::AssociatedObjects associated_objects{
    objects_with_associations.objects, objects_with_associations.association};

  /* tracker update */
  state.processor->update(associated_objects);

  /* tracker pruning */
  state.processor->prune(measurement_time);

  /* spawn new tracker */
  state.processor->spawn(associated_objects);
}

// High-level orchestration functions
ObjectProcessingResult process_objects_batch(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state, TrackerDebugger & debugger,
  const rclcpp::Logger & logger)
{
  ObjectProcessingResult result;
  result.should_publish = false;

  // get objects from the input manager and run process
  types::ObjectsWithAssociationList objects_with_associations;
  const bool is_objects_ready =
    state.input_manager->getObjects(current_time, objects_with_associations);
  if (!is_objects_ready) {
    return result;
  }

  // process start - start measurement time before processing
  debugger.startMeasurementTime(current_time, objects_with_associations.back().getTimestamp());

  // run process for each DynamicObject
  for (const auto & objects_data : objects_with_associations) {
    process_objects_(objects_data, current_time, state, logger);
  }

  // Update last_updated_time and last_tracker_time
  state.last_updated_time = current_time;
  state.last_tracker_time = objects_with_associations.back().getTimestamp();

  // process end - end measurement time after processing
  debugger.endMeasurementTime(current_time);

  // Publish immediately if delay compensation is disabled
  result.should_publish = !params.enable_delay_compensation;

  return result;
}

PublishingData prepare_publishing_data(
  const rclcpp::Time & current_time, const MultiObjectTrackerParameters & params,
  MultiObjectTrackerInternalState & state, [[maybe_unused]] const rclcpp::Logger & logger)
{
  PublishingData result;

  const auto & last_tracker_time = state.last_tracker_time;

  // Calculate object_time based on delay compensation setting
  result.object_time = params.enable_delay_compensation ? current_time : last_tracker_time;

  /* tracker pruning*/
  state.processor->prune(last_tracker_time);

  // Get tracked objects
  result.tracked_objects = get_tracked_objects_(result.object_time, params, state);
  result.tracked_objects_size = result.tracked_objects.objects.size();

  // Update last_publish_time
  state.last_publish_time = current_time;

  return result;
}

OptionalPublishingData prepare_optional_publishing_data(
  const rclcpp::Time & object_time, const MultiObjectTrackerParameters & params,
  const MultiObjectTrackerInternalState & state, const TrackerDebugger & debugger,
  const rclcpp::Logger & logger)
{
  OptionalPublishingData result;

  // Get merged objects
  if (params.publish_merged_objects) {
    result.merged_objects = get_merged_objects_(object_time, params, state, logger);
  }

  // Calculate min_extrapolation_time
  const double dt = (object_time - state.last_tracker_time).seconds();
  result.min_extrapolation_time = dt > 0.0 ? dt : 0.0;

  // Prepare tentative objects
  if (debugger.shouldPublishTentativeObjects()) {
    autoware_perception_msgs::msg::TrackedObjects tentative_objects;
    tentative_objects.header.frame_id = params.world_frame_id;
    state.processor->getTentativeObjects(object_time, tentative_objects);
    result.tentative_objects = std::move(tentative_objects);
  }

  return result;
}

}  // namespace core
}  // namespace autoware::multi_object_tracker
