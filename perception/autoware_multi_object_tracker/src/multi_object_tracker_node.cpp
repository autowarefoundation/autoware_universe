// Copyright 2020 Tier IV, Inc.
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

#include <boost/optional.hpp>

#include <glog/logging.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using Label = autoware_perception_msgs::msg::ObjectClassification;
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  last_published_time_(this->now()),
  last_updated_time_(this->now())
{
  // glog for debug
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("multi_object_tracker");
    google::InstallFailureSignalHandler();
  }

  // Get parameters
  double publish_rate = declare_parameter<double>("publish_rate");  // [hz]
  world_frame_id_ = declare_parameter<std::string>("world_frame_id");
  bool enable_delay_compensation{declare_parameter<bool>("enable_delay_compensation")};
  bool enable_odometry_uncertainty = declare_parameter<bool>("consider_odometry_uncertainty");

  declare_parameter("selected_input_channels", std::vector<std::string>());
  std::vector<std::string> selected_input_channels =
    get_parameter("selected_input_channels").as_string_array();

  // ROS interface - Publisher
  tracked_objects_pub_ =
    create_publisher<autoware_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});

  // Odometry manager
  odometry_ = std::make_shared<Odometry>(*this, world_frame_id_, enable_odometry_uncertainty);

  // ROS interface - Input channels
  // Get input channels configuration
  if (selected_input_channels.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No input topics are specified.");
    return;
  }

  uint channel_index = 0;
  for (const auto & selected_input_channel : selected_input_channels) {
    types::InputChannel input_channel_config;
    input_channel_config.index = channel_index;
    channel_index++;

    // required parameters, no default value
    const std::string input_topic_name =
      declare_parameter<std::string>("input_channels." + selected_input_channel + ".topic");
    input_channel_config.input_topic = input_topic_name;

    // required parameter, but can set a default value
    input_channel_config.is_spawn_enabled = declare_parameter<bool>(
      "input_channels." + selected_input_channel + ".flags.can_spawn_new_tracker", true);

    // trust object existence probability
    input_channel_config.trust_existence_probability = declare_parameter<bool>(
      "input_channels." + selected_input_channel + ".flags.can_trust_existence_probability", true);

    // trust object extension, size beyond the visible area
    input_channel_config.trust_extension = declare_parameter<bool>(
      "input_channels." + selected_input_channel + ".flags.can_trust_extension", true);

    // trust object classification
    input_channel_config.trust_classification = declare_parameter<bool>(
      "input_channels." + selected_input_channel + ".flags.can_trust_classification", true);

    // trust object orientation(yaw)
    input_channel_config.trust_orientation = declare_parameter<bool>(
      "input_channels." + selected_input_channel + ".flags.can_trust_orientation", true);

    // optional parameters
    const std::string default_name = selected_input_channel;
    const std::string name_long = declare_parameter<std::string>(
      "input_channels." + selected_input_channel + ".optional.name", default_name);
    input_channel_config.long_name = name_long;

    const std::string default_name_short = selected_input_channel.substr(0, 3);
    const std::string name_short = declare_parameter<std::string>(
      "input_channels." + selected_input_channel + ".optional.short_name", default_name_short);
    input_channel_config.short_name = name_short;

    input_channels_config_.push_back(input_channel_config);
  }
  input_channel_size_ = input_channels_config_.size();

  // Initialize input manager
  input_manager_ = std::make_unique<InputManager>(*this, odometry_);
  input_manager_->init(input_channels_config_);  // Initialize input manager, set subscriptions
  input_manager_->setTriggerFunction(
    std::bind(&MultiObjectTracker::onTrigger, this));  // Set trigger function

  // Create ROS time based timer.
  // If the delay compensation is enabled, the timer is used to publish the output at the correct
  // time.
  if (enable_delay_compensation) {
    publisher_period_ = 1.0 / publish_rate;    // [s]
    constexpr double timer_multiplier = 10.0;  // 10 times frequent for publish timing check
    const auto timer_period = rclcpp::Rate(publish_rate * timer_multiplier).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), timer_period, std::bind(&MultiObjectTracker::onTimer, this));
  }

  // Initialize processor
  {
    TrackerProcessorConfig config;
    {
      config.tracker_map.insert(
        std::make_pair(Label::CAR, this->declare_parameter<std::string>("car_tracker")));
      config.tracker_map.insert(
        std::make_pair(Label::TRUCK, this->declare_parameter<std::string>("truck_tracker")));
      config.tracker_map.insert(
        std::make_pair(Label::BUS, this->declare_parameter<std::string>("bus_tracker")));
      config.tracker_map.insert(
        std::make_pair(Label::TRAILER, this->declare_parameter<std::string>("trailer_tracker")));
      config.tracker_map.insert(std::make_pair(
        Label::PEDESTRIAN, this->declare_parameter<std::string>("pedestrian_tracker")));
      config.tracker_map.insert(
        std::make_pair(Label::BICYCLE, this->declare_parameter<std::string>("bicycle_tracker")));
      config.tracker_map.insert(std::make_pair(
        Label::MOTORCYCLE, this->declare_parameter<std::string>("motorcycle_tracker")));

      // Declare parameters
      config.tracker_lifetime = declare_parameter<double>("tracker_lifetime");
      config.min_known_object_removal_iou =
        declare_parameter<double>("min_known_object_removal_iou");
      config.min_unknown_object_removal_iou =
        declare_parameter<double>("min_unknown_object_removal_iou");
      config.distance_threshold = declare_parameter<double>("distance_threshold");

      // Map from class name to label
      std::map<std::string, LabelType> class_name_to_label = {
        {"UNKNOWN", Label::UNKNOWN}, {"CAR", Label::CAR},
        {"TRUCK", Label::TRUCK},     {"BUS", Label::BUS},
        {"TRAILER", Label::TRAILER}, {"MOTORBIKE", Label::MOTORCYCLE},
        {"BICYCLE", Label::BICYCLE}, {"PEDESTRIAN", Label::PEDESTRIAN}};

      // Declare parameters and initialize confident_count_threshold_map
      for (const auto & [class_name, class_label] : class_name_to_label) {
        int64_t value = declare_parameter<int64_t>("confident_count_threshold." + class_name);
        config.confident_count_threshold[class_label] = static_cast<int>(value);
      }
    }

    AssociatorConfig associator_config;
    {
      const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
      const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
      associator_config.can_assign_matrix = can_assign_matrix;
      associator_config.max_dist_matrix =
        this->declare_parameter<std::vector<double>>("max_dist_matrix");
      associator_config.max_area_matrix =
        this->declare_parameter<std::vector<double>>("max_area_matrix");
      associator_config.min_area_matrix =
        this->declare_parameter<std::vector<double>>("min_area_matrix");
      associator_config.max_rad_matrix =
        this->declare_parameter<std::vector<double>>("max_rad_matrix");
      associator_config.min_iou_matrix =
        this->declare_parameter<std::vector<double>>("min_iou_matrix");
    }

    // Initialize processor with parameters
    processor_ =
      std::make_unique<TrackerProcessor>(config, associator_config, input_channels_config_);
  }

  // Debugger
  debugger_ = std::make_unique<TrackerDebugger>(*this, world_frame_id_, input_channels_config_);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void MultiObjectTracker::onTrigger()
{
  const rclcpp::Time current_time = this->now();
  // get objects from the input manager and run process
  ObjectsList objects_list;
  const bool is_objects_ready = input_manager_->getObjects(current_time, objects_list);
  if (!is_objects_ready) return;

  // process start
  last_updated_time_ = current_time;
  const rclcpp::Time latest_time(objects_list.back().header.stamp);
  debugger_->startMeasurementTime(this->now(), latest_time);
  // run process for each DynamicObject
  for (const auto & objects_data : objects_list) {
    runProcess(objects_data);
  }
  // process end
  debugger_->endMeasurementTime(this->now());

  // Publish without delay compensation
  if (!publish_timer_) {
    const auto latest_object_time = rclcpp::Time(objects_list.back().header.stamp);
    checkAndPublish(latest_object_time);
  }
}

void MultiObjectTracker::onTimer()
{
  const rclcpp::Time current_time = this->now();

  // ensure minimum interval: room for the next process(prediction)
  const double minimum_publish_interval = publisher_period_ * minimum_publish_interval_ratio;
  const auto elapsed_time = (current_time - last_published_time_).seconds();
  if (elapsed_time < minimum_publish_interval) {
    return;
  }

  // if there was update after publishing, publish new messages
  bool should_publish = last_published_time_ < last_updated_time_;

  // if there was no update, publish if the elapsed time is longer than the maximum publish latency
  // in this case, it will perform extrapolate/remove old objects
  const double maximum_publish_interval = publisher_period_ * maximum_publish_interval_ratio;
  should_publish = should_publish || elapsed_time > maximum_publish_interval;

  // Publish with delay compensation to the current time
  if (should_publish) checkAndPublish(current_time);
}

void MultiObjectTracker::runProcess(const types::DynamicObjectList & detected_objects)
{
  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    rclcpp::Time(detected_objects.header.stamp, this->now().get_clock_type());

  /* predict trackers to the measurement time */
  processor_->predict(measurement_time);

  /* object association */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  processor_->associate(detected_objects, direct_assignment, reverse_assignment);

  // Collect debug information - tracker list, existence probabilities, association results
  debugger_->collectObjectInfo(
    measurement_time, processor_->getListTracker(), detected_objects, direct_assignment,
    reverse_assignment);

  /* tracker update */
  processor_->update(detected_objects, direct_assignment);

  /* tracker pruning */
  processor_->prune(measurement_time);

  /* spawn new tracker */
  processor_->spawn(detected_objects, reverse_assignment);
}

void MultiObjectTracker::checkAndPublish(const rclcpp::Time & time)
{
  /* tracker pruning*/
  processor_->prune(time);

  // Publish
  publish(time);

  // Update last published time
  last_published_time_ = this->now();
}

void MultiObjectTracker::publish(const rclcpp::Time & time) const
{
  debugger_->startPublishTime(this->now());
  const auto subscriber_count = tracked_objects_pub_->get_subscription_count() +
                                tracked_objects_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }
  // Create output msg
  autoware_perception_msgs::msg::TrackedObjects output_msg;
  output_msg.header.frame_id = world_frame_id_;
  processor_->getTrackedObjects(time, output_msg);

  // Publish
  tracked_objects_pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(tracked_objects_pub_, output_msg.header.stamp);

  // Publish debugger information if enabled
  debugger_->endPublishTime(this->now(), time);

  if (debugger_->shouldPublishTentativeObjects()) {
    autoware_perception_msgs::msg::TrackedObjects tentative_output_msg;
    tentative_output_msg.header.frame_id = world_frame_id_;
    processor_->getTentativeObjects(time, tentative_output_msg);
    debugger_->publishTentativeObjects(tentative_output_msg);
  }
  debugger_->publishObjectsMarkers();
}

}  // namespace autoware::multi_object_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::multi_object_tracker::MultiObjectTracker)
