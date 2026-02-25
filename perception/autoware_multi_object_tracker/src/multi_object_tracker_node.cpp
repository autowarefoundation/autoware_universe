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

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <boost/optional.hpp>

#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <csignal>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using autoware_utils_debug::ScopedTimeTrack;
using Label = autoware_perception_msgs::msg::ObjectClassification;
using LabelType = autoware_perception_msgs::msg::ObjectClassification::_label_type;

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options)
{
  // Get parameters
  params_.publish_rate = declare_parameter<double>("publish_rate");  // [hz]
  params_.world_frame_id = declare_parameter<std::string>("world_frame_id");
  params_.ego_frame_id = declare_parameter<std::string>("ego_frame_id");
  params_.enable_delay_compensation = declare_parameter<bool>("enable_delay_compensation");
  params_.enable_odometry_uncertainty = declare_parameter<bool>("consider_odometry_uncertainty");
  params_.publish_processing_time_detail =
    declare_parameter<bool>("publish_processing_time_detail");
  params_.publish_merged_objects = declare_parameter<bool>("publish_merged_objects");

  // ROS interface - Input channels
  // define input channel parameters
  std::vector<std::string> input_channels;
  std::vector<std::string> input_channel_topics;
  input_channels.resize(types::max_channel_size);
  input_channel_topics.resize(types::max_channel_size);
  for (size_t i = 0; i < types::max_channel_size; i++) {
    // the index number is zero filled two digits format
    const int index = static_cast<int>(i + 1);
    const std::string channel_id =
      std::string("detection") + (index < 10 ? "0" : "") + std::to_string(index);
    input_channels.at(i) = declare_parameter<std::string>("input/" + channel_id + "/channel");
    input_channel_topics.at(i) = declare_parameter<std::string>("input/" + channel_id + "/objects");
  }

  // parse input channels
  uint channel_index = 0;
  for (size_t i = 0; i < types::max_channel_size; i++) {
    const std::string & input_channel = input_channels.at(i);
    const std::string & input_channel_topic = input_channel_topics.at(i);
    if (input_channel.empty() || input_channel == "none") {
      continue;
    }

    types::InputChannel input_channel_config;
    input_channel_config.index = channel_index;
    channel_index++;

    // topic name
    input_channel_config.input_topic = input_channel_topic;
    // required parameter, but can set a default value
    input_channel_config.is_spawn_enabled = declare_parameter<bool>(
      "input_channels." + input_channel + ".flags.can_spawn_new_tracker", true);

    // trust object existence probability
    input_channel_config.trust_existence_probability = declare_parameter<bool>(
      "input_channels." + input_channel + ".flags.can_trust_existence_probability", false);

    // trust object extension, size beyond the visible area
    input_channel_config.trust_extension = declare_parameter<bool>(
      "input_channels." + input_channel + ".flags.can_trust_extension", true);

    // trust object classification
    input_channel_config.trust_classification = declare_parameter<bool>(
      "input_channels." + input_channel + ".flags.can_trust_classification", true);

    // trust object orientation(yaw)
    input_channel_config.trust_orientation = declare_parameter<bool>(
      "input_channels." + input_channel + ".flags.can_trust_orientation", true);

    // optional parameters
    const std::string default_name = input_channel;
    const std::string name_long = declare_parameter<std::string>(
      "input_channels." + input_channel + ".optional.name", default_name);
    input_channel_config.long_name = name_long;

    const std::string default_name_short = input_channel.substr(0, 3);
    const std::string name_short = declare_parameter<std::string>(
      "input_channels." + input_channel + ".optional.short_name", default_name_short);
    input_channel_config.short_name = name_short;

    params_.input_channels_config.push_back(input_channel_config);
  }

  // Parameters for processor
  {
    // convert string to TrackerType
    static const std::unordered_map<std::string, TrackerType> TRACKER_TYPE_MAP = {
      {"multi_vehicle_tracker", TrackerType::MULTIPLE_VEHICLE},
      {"pedestrian_and_bicycle_tracker", TrackerType::PEDESTRIAN_AND_BICYCLE},
      {"normal_vehicle_tracker", TrackerType::NORMAL_VEHICLE},
      {"pedestrian_tracker", TrackerType::PEDESTRIAN},
      {"big_vehicle_tracker", TrackerType::BIG_VEHICLE},
      {"bicycle_tracker", TrackerType::BICYCLE},
      {"pass_through_tracker", TrackerType::PASS_THROUGH}};
    auto getTrackerType = [](const std::string & tracker_name) -> TrackerType {
      auto it = TRACKER_TYPE_MAP.find(tracker_name);
      return it != TRACKER_TYPE_MAP.end() ? it->second : TrackerType::UNKNOWN;
    };

    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::CAR, getTrackerType(this->declare_parameter<std::string>("car_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::TRUCK, getTrackerType(this->declare_parameter<std::string>("truck_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::BUS, getTrackerType(this->declare_parameter<std::string>("bus_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::TRAILER, getTrackerType(this->declare_parameter<std::string>("trailer_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::PEDESTRIAN,
        getTrackerType(this->declare_parameter<std::string>("pedestrian_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::BICYCLE, getTrackerType(this->declare_parameter<std::string>("bicycle_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(
        Label::MOTORCYCLE,
        getTrackerType(this->declare_parameter<std::string>("motorcycle_tracker"))));
    params_.processor_config.tracker_map.insert(
      std::make_pair(Label::UNKNOWN, TrackerType::UNKNOWN));  // Default for unknown objects

    // Declare parameters
    params_.processor_config.tracker_lifetime = declare_parameter<double>("tracker_lifetime");
    params_.processor_config.min_known_object_removal_iou =
      declare_parameter<double>("min_known_object_removal_iou");
    params_.processor_config.min_unknown_object_removal_iou =
      declare_parameter<double>("min_unknown_object_removal_iou");

    // Declare parameters for generalized IoU threshold
    std::vector<double> pruning_giou_thresholds =
      declare_parameter<std::vector<double>>("pruning_generalized_iou_thresholds");
    for (size_t i = 0; i < pruning_giou_thresholds.size(); ++i) {
      const auto label = static_cast<LabelType>(i);
      params_.processor_config.pruning_giou_thresholds[label] = pruning_giou_thresholds.at(i);
    }
    params_.processor_config.pruning_static_object_speed =
      declare_parameter<double>("pruning_static_object_speed");
    params_.processor_config.pruning_moving_object_speed =
      declare_parameter<double>("pruning_moving_object_speed");
    params_.processor_config.pruning_static_iou_threshold =
      declare_parameter<double>("pruning_static_iou_threshold");

    // Declare parameters for overlap distance threshold
    std::vector<double> pruning_distance_threshold_list =
      declare_parameter<std::vector<double>>("pruning_distance_thresholds");
    for (size_t i = 0; i < pruning_distance_threshold_list.size(); ++i) {
      const auto label = static_cast<LabelType>(i);
      params_.processor_config.pruning_distance_thresholds[label] =
        pruning_distance_threshold_list[i];
    }

    params_.processor_config.enable_unknown_object_velocity_estimation =
      declare_parameter<bool>("enable_unknown_object_velocity_estimation");
    params_.processor_config.enable_unknown_object_motion_output =
      declare_parameter<bool>("enable_unknown_object_motion_output");
  }

  {
    auto initializeMatrixInt = [](const std::vector<int64_t> & vector) {
      const int label_num = types::NUM_LABELS;
      if (vector.size() != label_num * label_num) {
        throw std::runtime_error("Invalid can_assign_matrix size");
      }
      std::vector<int> converted_vector(vector.begin(), vector.end());
      // Use row-major mapping to match the YAML layout
      using RowMajorMatrixXi = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
      Eigen::Map<RowMajorMatrixXi> matrix_tmp(converted_vector.data(), label_num, label_num);

      // Convert to column-major (Eigen's default) for consistency
      return Eigen::MatrixXi(matrix_tmp);
    };
    auto initializeMatrixDouble = [](const std::vector<double> & vector) {
      const int label_num = types::NUM_LABELS;
      if (vector.size() != label_num * label_num) {
        throw std::runtime_error("Invalid association matrix configuration size");
      }
      // Use row-major mapping to match the YAML layout
      using RowMajorMatrixXd =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
      Eigen::Map<const RowMajorMatrixXd> matrix_tmp(vector.data(), label_num, label_num);

      // Convert to column-major (Eigen's default) for consistency
      return Eigen::MatrixXd(matrix_tmp);
    };
    Eigen::MatrixXi can_assign_matrix =
      initializeMatrixInt(this->declare_parameter<std::vector<int64_t>>("can_assign_matrix"));
    params_.associator_config.max_dist_matrix =
      initializeMatrixDouble(this->declare_parameter<std::vector<double>>("max_dist_matrix"));
    params_.associator_config.max_area_matrix =
      initializeMatrixDouble(this->declare_parameter<std::vector<double>>("max_area_matrix"));
    params_.associator_config.min_area_matrix =
      initializeMatrixDouble(this->declare_parameter<std::vector<double>>("min_area_matrix"));
    params_.associator_config.min_iou_matrix =
      initializeMatrixDouble(this->declare_parameter<std::vector<double>>("min_iou_matrix"));

    // pre-process
    const int label_num = params_.associator_config.max_dist_matrix.rows();
    for (int i = 0; i < label_num; i++) {
      for (int j = 0; j < label_num; j++) {
        params_.associator_config.max_dist_matrix(i, j) =
          params_.associator_config.max_dist_matrix(i, j) *
          params_.associator_config.max_dist_matrix(i, j);
      }
    }
    // Set the unknown-unknown association GIoU threshold
    params_.associator_config.unknown_association_giou_threshold =
      declare_parameter<double>("unknown_association_giou_threshold");

    // Set the tracker map for associator config
    {
      params_.associator_config.can_assign_map.clear();
      for (const auto & [label, tracker_type] : params_.processor_config.tracker_map) {
        params_.associator_config.can_assign_map[tracker_type].fill(false);
      }
      // can_assign_map : tracker_type that can be assigned to each measurement label
      // relationship is given by tracker_map and can_assign_matrix
      for (int i = 0; i < can_assign_matrix.rows(); ++i) {
        for (int j = 0; j < can_assign_matrix.cols(); ++j) {
          if (can_assign_matrix(i, j) == 1) {
            const auto tracker_type = params_.processor_config.tracker_map.at(i);
            params_.associator_config.can_assign_map[tracker_type][j] = true;
          }
        }
      }
    }
  }

  // Initialize state
  // Odometry manager
  state_.odometry = std::make_shared<Odometry>(
    *this, params_.world_frame_id, params_.ego_frame_id, params_.enable_odometry_uncertainty);

  // Initialize input manager
  state_.input_manager = std::make_unique<InputManager>(state_.odometry, get_logger(), get_clock());
  state_.input_manager->init(
    params_.input_channels_config);  // Initialize input manager, set subscriptions
  state_.input_manager->setTriggerFunction(
    std::bind(&MultiObjectTracker::onTrigger, this));  // Set trigger function

  // Initialize processor
  state_.processor = std::make_unique<TrackerProcessor>(
    params_.processor_config, params_.associator_config, params_.input_channels_config);

  state_.last_published_time = this->now();
  state_.last_updated_time = this->now();

  // Create subscriptions
  const size_t input_size = params_.input_channels_config.size();
  sub_objects_array_.resize(input_size);
  for (size_t i = 0; i < input_size; i++) {
    const auto & channel = params_.input_channels_config[i];
    RCLCPP_INFO(
      get_logger(), "MultiObjectTracker::init Initializing %s input stream from %s",
      channel.long_name.c_str(), channel.input_topic.c_str());

    std::function<void(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)>
      func =
        std::bind(&InputManager::onMessage, state_.input_manager.get(), i, std::placeholders::_1);

    sub_objects_array_.at(i) = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
      channel.input_topic, rclcpp::QoS{1}, func);
  }

  // ROS interface - Publisher
  tracked_objects_pub_ = create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "output/objects", rclcpp::QoS{1});
  if (params_.publish_merged_objects) {
    // if the input is multi-channel, export fused merged (detected) objects
    merged_objects_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "output/merged_objects", rclcpp::QoS{1});
    for (const auto & channel : params_.input_channels_config) {
      // check if merged_objects_pub_ is in topics of input channel
      if (channel.input_topic == merged_objects_pub_->get_topic_name()) {
        RCLCPP_WARN(
          get_logger(),
          "Merged objects publisher topic is set in input channel: %s, topic: %s"
          ", disabling merged objects publisher.",
          channel.long_name.c_str(), channel.input_topic.c_str());
        params_.publish_merged_objects = false;
        merged_objects_pub_ = nullptr;
        break;
      }
    }
  }
  // Create ROS time based timer.
  // If the delay compensation is enabled, the timer is used to publish the output at the correct
  // time.
  if (params_.enable_delay_compensation) {
    // const double publisher_period = 1.0 / params_.publish_rate;    // [s]
    constexpr double timer_multiplier = 10.0;  // 10 times frequent for publish timing check
    const auto timer_period = rclcpp::Rate(params_.publish_rate * timer_multiplier).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), timer_period, std::bind(&MultiObjectTracker::onTimer, this));
  }

  // Debugger
  state_.debugger =
    std::make_unique<TrackerDebugger>(*this, params_.world_frame_id, params_.input_channels_config);
  state_.published_time_publisher =
    std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);

  if (params_.publish_processing_time_detail) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    state_.time_keeper =
      std::make_shared<autoware_utils_debug::TimeKeeper>(detailed_processing_time_publisher_);
    state_.processor->setTimeKeeper(state_.time_keeper);
  }
}

void MultiObjectTracker::onTrigger()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  const rclcpp::Time current_time = this->now();
  // get objects from the input manager and run process
  ObjectsList objects_list;
  const bool is_objects_ready = state_.input_manager->getObjects(current_time, objects_list);
  if (!is_objects_ready) return;

  // process start
  const rclcpp::Time latest_time(objects_list.back().header.stamp);
  state_.debugger->startMeasurementTime(this->now(), latest_time);

  // run process for each DynamicObject
  for (const auto & objects_data : objects_list) {
    // Get ego pose at the measurement time
    const rclcpp::Time measurement_time =
      rclcpp::Time(objects_data.header.stamp, current_time.get_clock_type());

    std::optional<geometry_msgs::msg::Pose> ego_pose;
    if (const auto odometry_info = state_.odometry->getOdometryFromTf(measurement_time)) {
      ego_pose = odometry_info->pose.pose;
    }

    core::process_objects(objects_data, current_time, ego_pose, params_, state_, get_logger());
  }
  // process end
  state_.debugger->endMeasurementTime(this->now());

  // Publish without delay compensation
  if (!publish_timer_) {
    const auto latest_object_time = rclcpp::Time(objects_list.back().header.stamp);
    checkAndPublish(latest_object_time);
  }
}

void MultiObjectTracker::onTimer()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  const rclcpp::Time current_time = this->now();

  if (core::should_publish(current_time, params_, state_)) {
    checkAndPublish(state_.last_published_time);
  }
}

void MultiObjectTracker::checkAndPublish(const rclcpp::Time & time)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  /* tracker pruning*/
  core::prune_objects(time, state_);

  // Publish
  const rclcpp::Time current_time = this->now();

  const auto tf_base_to_world = state_.odometry->getTransform(time);

  auto output =
    core::get_output(time, current_time, tf_base_to_world, params_, state_, get_logger());

  publish(output);
}

void MultiObjectTracker::publish(const core::PublishData & data)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (state_.time_keeper) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *state_.time_keeper);

  // Publish
  tracked_objects_pub_->publish(data.tracked_objects);

  if (params_.publish_merged_objects && data.merged_objects) {
    merged_objects_pub_->publish(*data.merged_objects);
  }

  // Publish debug messages
  {
    std::unique_ptr<ScopedTimeTrack> st_debug_ptr;
    if (state_.time_keeper)
      st_debug_ptr = std::make_unique<ScopedTimeTrack>("debug_publish", *state_.time_keeper);

    state_.published_time_publisher->publish_if_subscribed(
      tracked_objects_pub_, data.tracked_objects.header.stamp);

    if (data.tentative_objects) {
      state_.debugger->publishTentativeObjects(*data.tentative_objects);
    }
    state_.debugger->publishObjectsMarkers();
  }
}

}  // namespace autoware::multi_object_tracker

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::multi_object_tracker::MultiObjectTracker)
