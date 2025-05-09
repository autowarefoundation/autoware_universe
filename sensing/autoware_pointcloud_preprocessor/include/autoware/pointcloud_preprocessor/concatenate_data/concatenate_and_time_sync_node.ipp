// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/utils.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::
  PointCloudConcatenateDataSynchronizerComponentTemplated(const rclcpp::NodeOptions & node_options)
: Node("point_cloud_concatenator_component", node_options)
{
  // initialize debug tool
  using autoware_utils::DebugPublisher;
  using autoware_utils::StopWatch;
  stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<DebugPublisher>(this, "concatenate_data_synchronizer");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  //  initialize parameters
  params_.debug_mode = declare_parameter<bool>("debug_mode");
  params_.rosbag_length = declare_parameter<double>("rosbag_length");
  params_.maximum_queue_size = static_cast<size_t>(declare_parameter<int>("maximum_queue_size"));
  params_.timeout_sec = declare_parameter<double>("timeout_sec");
  params_.is_motion_compensated = declare_parameter<bool>("is_motion_compensated");
  params_.publish_synchronized_pointcloud =
    declare_parameter<bool>("publish_synchronized_pointcloud");
  params_.keep_input_frame_in_synchronized_pointcloud =
    declare_parameter<bool>("keep_input_frame_in_synchronized_pointcloud");
  params_.publish_previous_but_late_pointcloud =
    declare_parameter<bool>("publish_previous_but_late_pointcloud");
  params_.synchronized_pointcloud_postfix =
    declare_parameter<std::string>("synchronized_pointcloud_postfix");
  params_.input_twist_topic_type = declare_parameter<std::string>("input_twist_topic_type");
  params_.input_topics = declare_parameter<std::vector<std::string>>("input_topics");
  params_.output_frame = declare_parameter<std::string>("output_frame");

  if (params_.input_topics.empty()) {
    throw std::runtime_error("Need a 'input_topics' parameter to be set before continuing.");
  }
  if (params_.input_topics.size() == 1) {
    throw std::runtime_error("Only one topic given. Need at least two topics to continue.");
  }

  if (params_.output_frame.empty()) {
    throw std::runtime_error("Need an 'output_frame' parameter to be set before continuing.");
  }

  params_.matching_strategy = declare_parameter<std::string>("matching_strategy.type");

  if (params_.matching_strategy == "naive") {
    collector_matching_strategy_ = std::make_unique<NaiveMatchingStrategy<MsgTraits>>(*this);
  } else if (params_.matching_strategy == "advanced") {
    collector_matching_strategy_ =
      std::make_unique<AdvancedMatchingStrategy<MsgTraits>>(*this, params_.input_topics);
  } else {
    throw std::runtime_error("Matching strategy must be 'advanced' or 'naive'");
  }

  // Implementation independent subscribers
  if (params_.is_motion_compensated) {
    if (params_.input_twist_topic_type == "twist") {
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/input/twist", rclcpp::QoS{100},
        std::bind(
          &PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::twist_callback, this,
          std::placeholders::_1));
    } else if (params_.input_twist_topic_type == "odom") {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/odom", rclcpp::QoS{100},
        std::bind(
          &PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::odom_callback, this,
          std::placeholders::_1));
    } else {
      throw std::runtime_error(
        "input_twist_topic_type is invalid: " + params_.input_twist_topic_type);
    }
  }


  // Combine cloud handler
  combine_cloud_handler_ = std::make_shared<CombineCloudHandler<MsgTraits>>(
    *this, params_.input_topics, params_.output_frame, params_.is_motion_compensated,
    params_.publish_synchronized_pointcloud, params_.keep_input_frame_in_synchronized_pointcloud);

  // Diagnostic Updater
  std::ostringstream hardware_id_stream;
  hardware_id_stream << this->get_fully_qualified_name() << "_checker";
  std::string hardware_id = hardware_id_stream.str();

  std::ostringstream diagnostic_name_stream;
  diagnostic_name_stream << this->get_fully_qualified_name() << "_status";
  std::string diagnostic_name = diagnostic_name_stream.str();

  diagnostic_updater_.setHardwareID(hardware_id);
  diagnostic_updater_.add(
    diagnostic_name, this, &PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::check_concat_status);

  initialize_pub_sub();
}

template <typename MsgTraits>
std::string
PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::replace_sync_topic_name_postfix(
  const std::string & original_topic_name, const std::string & postfix)
{
  std::string replaced_topic_name;
  // separate the topic name by '/' and replace the last element with the new postfix
  size_t pos = original_topic_name.find_last_of('/');
  if (pos == std::string::npos) {
    // not found '/': this is not a namespaced topic
    RCLCPP_WARN_STREAM(
      get_logger(),
      "The topic name is not namespaced. The postfix will be added to the end of the topic name.");
    return original_topic_name + postfix;
  }

  // replace the last element with the new postfix
  replaced_topic_name = original_topic_name.substr(0, pos) + "/" + postfix;

  // if topic name is the same with original topic name, add postfix to the end of the topic name
  if (replaced_topic_name == original_topic_name) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The topic name "
                      << original_topic_name
                      << " have the same postfix with synchronized pointcloud. We use "
                         "the postfix "
                         "to the end of the topic name.");
    replaced_topic_name = original_topic_name + default_sync_topic_postfix;
  }
  return replaced_topic_name;
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::initialize_collector_list()
{
  // Initialize collector list
  for (size_t i = 0; i < num_of_collectors; ++i) {
    cloud_collectors_.emplace_back(std::make_shared<CloudCollector<MsgTraits>>(
      std::dynamic_pointer_cast<PointCloudConcatenateDataSynchronizerComponentTemplated>(shared_from_this()),
      combine_cloud_handler_, params_.input_topics.size(), params_.timeout_sec,
      params_.debug_mode));
  }
  init_collector_list_ = true;
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::cloud_callback(
  const typename MsgTraits::PointCloudMessage::ConstSharedPtr & input_ptr,
  const std::string & topic_name)
{
  stop_watch_ptr_->toc("processing_time", true);

  if (!init_collector_list_) {
    initialize_collector_list();
  }

  double cloud_arrival_time = this->get_clock()->now().seconds();
  manage_collector_list();

  if (!utils::is_data_layout_compatible_with_point_xyzirc(*input_ptr)) {
    RCLCPP_ERROR(
      get_logger(), "The pointcloud layout is not compatible with PointXYZIRC. Aborting");

    if (utils::is_data_layout_compatible_with_point_xyzi(*input_ptr)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy code/data");
    }

    return;
  }

  if (params_.debug_mode) {
    RCLCPP_INFO(
      this->get_logger(), " pointcloud %s  timestamp: %lf arrive time: %lf seconds, latency: %lf",
      topic_name.c_str(), rclcpp::Time(input_ptr->header.stamp).seconds(),
      cloud_arrival_time,
      cloud_arrival_time - rclcpp::Time(input_ptr->header.stamp).seconds());
  }

  if (input_ptr->width * input_ptr->height == 0) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
  }

  // protect cloud collectors list
  std::shared_ptr<CloudCollector<MsgTraits>> selected_collector = nullptr;

  // For each callback, check whether there is an existing collector that matches this cloud
  std::optional<std::shared_ptr<CloudCollector<MsgTraits>>> cloud_collector = std::nullopt;
  MatchingParams matching_params;
  matching_params.topic_name = topic_name;
  matching_params.cloud_arrival_time = cloud_arrival_time;
  matching_params.cloud_timestamp = rclcpp::Time(input_ptr->header.stamp).seconds();

  if (!cloud_collectors_.empty()) {
    cloud_collector =
      collector_matching_strategy_->match_cloud_to_collector(cloud_collectors_, matching_params);
  }

  if (cloud_collector.has_value() && cloud_collector.value()) {
    selected_collector = cloud_collector.value();
  }

  // Didn't find matched collector
  if (!selected_collector || selected_collector->get_status() == CollectorStatus::Finished) {
    // Reuse a collector if one is in the IDLE state
    auto it = std::find_if(
      cloud_collectors_.begin(), cloud_collectors_.end(),
      [](const auto & collector) { return collector->get_status() == CollectorStatus::Idle; });

    if (it != cloud_collectors_.end()) {
      selected_collector = *it;
    } else {
      auto oldest_it = find_and_reset_oldest_collector();
      if (oldest_it != cloud_collectors_.end()) {
        selected_collector = *oldest_it;
      } else {
        // Handle case where no suitable collector is found
        RCLCPP_ERROR(get_logger(), "No available CloudCollector in IDLE state.");
        return;
      }
    }

    if (selected_collector) {
      collector_matching_strategy_->set_collector_info(selected_collector, matching_params);
    }
  }

  selected_collector->process_pointcloud(topic_name, input_ptr);
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::twist_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input)
{
  combine_cloud_handler_->process_twist(input);
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input)
{
  combine_cloud_handler_->process_odometry(input);
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::publish_clouds(
  ConcatenatedCloudResult<MsgTraits> && concatenated_cloud_result,
  std::shared_ptr<CollectorInfoBase> collector_info)
{
  // should never come to this state.
  if (concatenated_cloud_result.concatenate_cloud_ptr == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Concatenated cloud is a nullptr.");
    return;
  }

  if (
    concatenated_cloud_result.concatenate_cloud_ptr->width *
      concatenated_cloud_result.concatenate_cloud_ptr->height ==
    0) {
    RCLCPP_ERROR(this->get_logger(), "Concatenated cloud is an empty pointcloud.");
    is_concatenated_cloud_empty_ = true;
  }

  current_concatenate_cloud_timestamp_ =
    rclcpp::Time(concatenated_cloud_result.concatenate_cloud_ptr->header.stamp).seconds();

  if (
    current_concatenate_cloud_timestamp_ < latest_concatenate_cloud_timestamp_ &&
    !params_.publish_previous_but_late_pointcloud) {
    // Publish the cloud if the rosbag replays in loop
    if (
      latest_concatenate_cloud_timestamp_ - current_concatenate_cloud_timestamp_ >
      params_.rosbag_length) {
      publish_pointcloud_ = true;  // Force publishing in this case
    } else {
      drop_previous_but_late_pointcloud_ = true;  // Otherwise, drop the late pointcloud
    }
  } else {
    // Publish pointcloud if timestamps are valid or the condition doesn't apply
    publish_pointcloud_ = true;
  }

  if (publish_pointcloud_) {
    latest_concatenate_cloud_timestamp_ = current_concatenate_cloud_timestamp_;
    concatenated_cloud_publisher_->publish(std::move(concatenated_cloud_result.concatenate_cloud_ptr));

    // publish transformed raw pointclouds
    if (
      params_.publish_synchronized_pointcloud &&
      concatenated_cloud_result.topic_to_transformed_cloud_map) {
      for (const auto & topic : params_.input_topics) {
        // Get a reference to the internal map
        if (
          (*concatenated_cloud_result.topic_to_transformed_cloud_map).find(topic) !=
          (*concatenated_cloud_result.topic_to_transformed_cloud_map).end()) {
          topic_to_transformed_cloud_publisher_map_[topic]->publish(
            std::move((*concatenated_cloud_result.topic_to_transformed_cloud_map).extract(topic).mapped()));
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "transformed_raw_points[%s] is nullptr, skipping pointcloud publish.", topic.c_str());
        }
      }
    }
  }

  diagnostic_collector_info_ = std::move(collector_info);

  diagnostic_topic_to_original_stamp_map_ = concatenated_cloud_result.topic_to_original_stamp_map;
  diagnostic_updater_.force_update();

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    for (const auto & [topic, stamp] : concatenated_cloud_result.topic_to_original_stamp_map) {
      const auto pipeline_latency_ms = (this->get_clock()->now().seconds() - stamp) * 1000;
      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug" + topic + "/pipeline_latency_ms", pipeline_latency_ms);
    }
  }
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::manage_collector_list()
{
  for (auto & collector : cloud_collectors_) {
    if (collector->get_status() == CollectorStatus::Finished) {
      collector->reset();
    }
  }
}

template <typename MsgTraits>
typename std::list<std::shared_ptr<CloudCollector<MsgTraits>>>::iterator
PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::find_and_reset_oldest_collector()
{
  auto min_it = cloud_collectors_.end();
  constexpr double k_max_timestamp = std::numeric_limits<double>::max();
  double min_timestamp = k_max_timestamp;

  for (auto it = cloud_collectors_.begin(); it != cloud_collectors_.end(); ++it) {
    if ((*it)->get_status() == CollectorStatus::Processing) {
      auto info = (*it)->get_info();
      double timestamp = k_max_timestamp;

      if (auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(info)) {
        timestamp = naive_info->timestamp;
      } else if (auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(info)) {
        timestamp = advanced_info->timestamp;
      } else {
        continue;
      }

      if (timestamp < min_timestamp) {
        min_timestamp = timestamp;
        min_it = it;

      }
    }
  }

  // Reset the processing collector with the oldest timestamp if found
  if (min_it != cloud_collectors_.end()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Reset the oldest collector because the number of processing collectors is equal to the "
      "limit of ("
        << num_of_collectors << ").");
    (*min_it)->reset();

  }

  return min_it;
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::check_concat_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (publish_pointcloud_ || drop_previous_but_late_pointcloud_) {
    stat.add(
      "concatenated_cloud_timestamp", format_timestamp(current_concatenate_cloud_timestamp_));

    if (
      auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(diagnostic_collector_info_)) {
      stat.add("first_cloud_arrival_timestamp", format_timestamp(naive_info->timestamp));
    } else if (
      auto advanced_info =
        std::dynamic_pointer_cast<AdvancedCollectorInfo>(diagnostic_collector_info_)) {
      stat.add(
        "reference_timestamp_min",
        format_timestamp(advanced_info->timestamp - advanced_info->noise_window));
      stat.add(
        "reference_timestamp_max",
        format_timestamp(advanced_info->timestamp + advanced_info->noise_window));
    }

    bool topic_miss = false;

    bool concatenation_success = true;
    for (const auto & topic : params_.input_topics) {
      bool input_cloud_concatenated = true;
      if (
        diagnostic_topic_to_original_stamp_map_.find(topic) !=
        diagnostic_topic_to_original_stamp_map_.end()) {
        stat.add(
          topic + "/timestamp", format_timestamp(diagnostic_topic_to_original_stamp_map_[topic]));
      } else {
        topic_miss = true;
        concatenation_success = false;
        input_cloud_concatenated = false;
      }
      stat.add(topic + "/is_concatenated", input_cloud_concatenated);
    }

    stat.add("cloud_concatenation_success", concatenation_success);

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Concatenated pointcloud is published and include all topics";

    if (drop_previous_but_late_pointcloud_) {
      if (topic_miss) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message =
          "Concatenated pointcloud misses some topics and is not published because it arrived "
          "too late";
      } else {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = "Concatenated pointcloud is not published as it is too late";
      }
    } else {
      if (is_concatenated_cloud_empty_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = "Concatenated pointcloud is empty";
      } else if (topic_miss) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        message = "Concatenated pointcloud is published but misses some topics";
      }
    }

    stat.summary(level, message);

    publish_pointcloud_ = false;
    drop_previous_but_late_pointcloud_ = false;
    is_concatenated_cloud_empty_ = false;
  } else {
    const int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    const std::string message =
      "Concatenate node launch successfully, but waiting for input pointcloud";
    stat.summary(level, message);
  }
}

template <typename MsgTraits>
std::list<std::shared_ptr<CloudCollector<MsgTraits>>>
PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::get_cloud_collectors()
{
  return cloud_collectors_;
}

template <typename MsgTraits>
void PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>::add_cloud_collector(
  const std::shared_ptr<CloudCollector<MsgTraits>> & collector)
{
  cloud_collectors_.push_back(collector);
}


}  // namespace autoware::pointcloud_preprocessor
