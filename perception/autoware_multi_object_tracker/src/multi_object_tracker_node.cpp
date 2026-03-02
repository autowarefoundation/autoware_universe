// Copyright 2020 TIER IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include "multi_object_tracker_node.hpp"

#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/uncertainty/uncertainty_processor.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/optional.hpp>

#include <array>
#include <iomanip>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using autoware_utils_debug::ScopedTimeTrack;

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options)
{
  ////// Get parameters
  params_.publish_rate = declare_parameter<double>("publish_rate");  // [hz]
  params_.world_frame_id = declare_parameter<std::string>("world_frame_id");
  params_.ego_frame_id = declare_parameter<std::string>("ego_frame_id");
  params_.enable_delay_compensation = declare_parameter<bool>("enable_delay_compensation");
  params_.enable_odometry_uncertainty = declare_parameter<bool>("consider_odometry_uncertainty");
  params_.publish_processing_time_detail =
    declare_parameter<bool>("publish_processing_time_detail");
  params_.publish_merged_objects = declare_parameter<bool>("publish_merged_objects");

  // define input channel parameters. the channel size is defined by this array.
  constexpr size_t MAX_INPUT_CHANNELS = 12;
  std::array<std::string, MAX_INPUT_CHANNELS> input_channels;
  for (size_t i = 0; i < input_channels.size(); i++) {
    std::ostringstream oss;
    oss << "input/detection" << std::setfill('0') << std::setw(2) << (i + 1) << "/channel";
    // channel parameters: input/detection01/channel, input/detection02/channel, ...,
    // input/detection12/channel
    input_channels[i] = declare_parameter<std::string>(oss.str());
  }

  {
    // parse input channels
    for (size_t i = 0; i < input_channels.size(); i++) {
      const std::string & input_channel = input_channels.at(i);

      types::InputChannel input_channel_config;
      input_channel_config.index = static_cast<uint>(i);

      if (input_channel.empty() || input_channel == "none") {
        input_channel_config.is_enabled = false;
        input_channel_config.is_spawn_enabled = false;
        input_channel_config.trust_existence_probability = false;
        input_channel_config.trust_extension = false;
        input_channel_config.trust_classification = false;
        input_channel_config.trust_orientation = false;
        input_channel_config.long_name = "none";
        input_channel_config.short_name = "none";
        params_.input_channels_config.push_back(input_channel_config);
        continue;
      }

      input_channel_config.is_enabled = true;

      const std::string input_channel_config_name = "input_channels." + input_channel;
      // required parameter, but can set a default value
      input_channel_config.is_spawn_enabled =
        declare_parameter<bool>(input_channel_config_name + ".flags.can_spawn_new_tracker", true);

      // trust object existence probability
      input_channel_config.trust_existence_probability = declare_parameter<bool>(
        input_channel_config_name + ".flags.can_trust_existence_probability", false);

      // trust object extension, size beyond the visible area
      input_channel_config.trust_extension =
        declare_parameter<bool>(input_channel_config_name + ".flags.can_trust_extension", true);

      // trust object classification
      input_channel_config.trust_classification = declare_parameter<bool>(
        input_channel_config_name + ".flags.can_trust_classification", true);

      // trust object orientation(yaw)
      input_channel_config.trust_orientation =
        declare_parameter<bool>(input_channel_config_name + ".flags.can_trust_orientation", true);

      // optional parameters
      const std::string default_name = input_channel;
      const std::string name_long =
        declare_parameter<std::string>(input_channel_config_name + ".optional.name", default_name);
      input_channel_config.long_name = name_long;

      const std::string default_name_short = input_channel.substr(0, 3);
      const std::string name_short = declare_parameter<std::string>(
        input_channel_config_name + ".optional.short_name", default_name_short);
      input_channel_config.short_name = name_short;

      params_.input_channels_config.push_back(input_channel_config);
    }
  }

  // tracker type map
  params_.tracker_type_map["car_tracker"] = declare_parameter<std::string>("car_tracker");
  params_.tracker_type_map["truck_tracker"] = declare_parameter<std::string>("truck_tracker");
  params_.tracker_type_map["bus_tracker"] = declare_parameter<std::string>("bus_tracker");
  params_.tracker_type_map["trailer_tracker"] = declare_parameter<std::string>("trailer_tracker");
  params_.tracker_type_map["pedestrian_tracker"] =
    declare_parameter<std::string>("pedestrian_tracker");
  params_.tracker_type_map["bicycle_tracker"] = declare_parameter<std::string>("bicycle_tracker");
  params_.tracker_type_map["motorcycle_tracker"] =
    declare_parameter<std::string>("motorcycle_tracker");

  params_.processor_config.tracker_lifetime = declare_parameter<double>("tracker_lifetime");
  params_.processor_config.min_known_object_removal_iou =
    declare_parameter<double>("min_known_object_removal_iou");
  params_.processor_config.min_unknown_object_removal_iou =
    declare_parameter<double>("min_unknown_object_removal_iou");

  // pruning parameters
  params_.pruning_giou_thresholds =
    declare_parameter<std::vector<double>>("pruning_generalized_iou_thresholds");
  params_.processor_config.pruning_static_object_speed =
    declare_parameter<double>("pruning_static_object_speed");
  params_.processor_config.pruning_moving_object_speed =
    declare_parameter<double>("pruning_moving_object_speed");
  params_.processor_config.pruning_static_iou_threshold =
    declare_parameter<double>("pruning_static_iou_threshold");

  // overlap distance threshold
  params_.pruning_distance_thresholds =
    declare_parameter<std::vector<double>>("pruning_distance_thresholds");
  params_.processor_config.enable_unknown_object_velocity_estimation =
    declare_parameter<bool>("enable_unknown_object_velocity_estimation");
  params_.processor_config.enable_unknown_object_motion_output =
    declare_parameter<bool>("enable_unknown_object_motion_output");

  // Parameters for associator
  params_.can_assign_matrix = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  params_.max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  params_.max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  params_.min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  params_.min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");

  // Set the unknown-unknown association GIoU threshold
  params_.associator_config.unknown_association_giou_threshold =
    declare_parameter<double>("unknown_association_giou_threshold");

  // process parameters
  core::process_parameters(params_);

  ////// Initialize state
  state_.init(params_, *this, std::bind(&MultiObjectTracker::onTrigger, this));

  ////// Create subscriptions and publishers
  // subscriptions
  sub_objects_array_.resize(params_.input_channels_config.size());
  for (const auto & input_channel : params_.input_channels_config) {
    if (!input_channel.is_enabled) {
      continue;
    }

    const auto & index = input_channel.index;
    std::ostringstream oss;
    oss << "~/input/detection" << std::setfill('0') << std::setw(2) << (index + 1) << "/objects";
    std::string input_channel_topic = oss.str();

    std::function<void(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)>
      func = std::bind(
        &InputManager::onMessage, state_.input_manager.get(), input_channel.index,
        std::placeholders::_1);

    sub_objects_array_.at(index) =
      create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
        input_channel_topic, rclcpp::QoS{1}, func);
  }

  // publishers
  tracked_objects_pub_ = create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "~/output/objects", rclcpp::QoS{1});
  if (params_.publish_merged_objects) {
    // if the input is multi-channel, export fused merged (detected) objects
    merged_objects_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "~/output/merged_objects", rclcpp::QoS{1});
  }

  ////// callback timer
  if (params_.enable_delay_compensation) {
    constexpr double timer_multiplier = 10.0;  // 10 times frequent for publish timing check
    const auto timer_period = rclcpp::Rate(params_.publish_rate * timer_multiplier).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), timer_period, std::bind(&MultiObjectTracker::onTimer, this));
  }

  ////// Debugger
  debugger_ = std::make_unique<TrackerDebugger>(
    get_logger(), get_clock(), params_.world_frame_id, params_.input_channels_config);
  debugger_->init(*this);
  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);

  if (params_.publish_processing_time_detail) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    time_keeper_ =
      std::make_shared<autoware_utils_debug::TimeKeeper>(detailed_processing_time_publisher_);
    state_.processor->setTimeKeeper(time_keeper_);
  }
}

void MultiObjectTracker::onTrigger()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const rclcpp::Time current_time = this->now();
  // get objects from the input manager and run process
  const auto objects_list = core::get_objects(current_time, state_);
  if (!objects_list) return;

  // process start
  const rclcpp::Time latest_time(objects_list->back().header.stamp);
  debugger_->startMeasurementTime(this->now(), latest_time);

  // run process for each DynamicObject
  for (const auto & objects_data : *objects_list) {
    std::unique_ptr<ScopedTimeTrack> st_process_objects_ptr;
    if (time_keeper_)
      st_process_objects_ptr = std::make_unique<ScopedTimeTrack>("process_objects", *time_keeper_);
    core::process_objects(objects_data, current_time, params_, state_, *debugger_, get_logger());
  }

  // process end
  debugger_->endMeasurementTime(this->now());

  // Publish without delay compensation
  if (!publish_timer_) {
    const auto latest_object_time = rclcpp::Time(objects_list->back().header.stamp);
    publish(latest_object_time);
  }
}

void MultiObjectTracker::onTimer()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const rclcpp::Time current_time = this->now();

  if (core::should_publish(current_time, params_, state_)) {
    publish(state_.last_published_time);
  }
}

void MultiObjectTracker::publish(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  /* tracker pruning*/
  core::prune_objects(time, state_);

  // Publish
  const rclcpp::Time current_time = this->now();

  debugger_->startPublishTime(current_time);
  autoware_perception_msgs::msg::TrackedObjects tracked_objects;
  {
    std::unique_ptr<ScopedTimeTrack> st_get_output_ptr;
    if (time_keeper_)
      st_get_output_ptr = std::make_unique<ScopedTimeTrack>("get_output", *time_keeper_);

    tracked_objects = core::get_tracked_objects(time, current_time, params_, state_);
  }
  tracked_objects_pub_->publish(tracked_objects);

  debugger_->endPublishTime(this->now(), time);

  publishOptional(time, current_time, tracked_objects.objects.size());
}

void MultiObjectTracker::publishOptional(
  const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
  const size_t tracked_objects_size)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Publish merged objects
  if (params_.publish_merged_objects) {
    const auto merged_objects =
      core::get_merged_objects(publish_time, current_time, params_, state_, get_logger());
    if (merged_objects) {
      merged_objects_pub_->publish(*merged_objects);
    }
  }

  // Update the diagnostic values
  const double min_extrapolation_time = (publish_time - state_.last_updated_time).seconds();
  debugger_->updateDiagnosticValues(min_extrapolation_time, tracked_objects_size);

  // Publish tentative objects
  const rclcpp::Time object_time = params_.enable_delay_compensation ? current_time : publish_time;
  if (debugger_->shouldPublishTentativeObjects()) {
    autoware_perception_msgs::msg::TrackedObjects tentative_output_msg;
    tentative_output_msg.header.frame_id = params_.world_frame_id;
    state_.processor->getTentativeObjects(object_time, tentative_output_msg);
    debugger_->publishTentativeObjects(tentative_output_msg);
  }

  published_time_publisher_->publish_if_subscribed(tracked_objects_pub_, object_time);

  // Publish debug markers
  debugger_->publishObjectsMarkers();
}

}  // namespace autoware::multi_object_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::multi_object_tracker::MultiObjectTracker)
