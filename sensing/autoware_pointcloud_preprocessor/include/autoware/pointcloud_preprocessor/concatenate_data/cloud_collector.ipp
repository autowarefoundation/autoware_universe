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

#pragma once

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
CloudCollector<MsgTraits>::CloudCollector(
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>> &&
    ros2_parent_node,
  std::shared_ptr<CombineCloudHandler<typename MsgTraits::PointCloudMessage>> &
    combine_cloud_handler,
  int num_of_clouds, double timeout_sec, bool debug_mode)
: ros2_parent_node_(std::move(ros2_parent_node)),
  combine_cloud_handler_(combine_cloud_handler),
  core_(static_cast<std::size_t>(num_of_clouds), timeout_sec),
  debug_mode_(debug_mode)
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(core_.timeout_sec()));

  timer_ =
    rclcpp::create_timer(ros2_parent_node_, ros2_parent_node_->get_clock(), period_ns, [this]() {
      if (core_.status() == CollectorStatus::Finished) return;
      concatenate_callback();
    });

  timer_->cancel();
}

template <typename MsgTraits>
void CloudCollector<MsgTraits>::set_info(std::shared_ptr<CollectorInfoBase> collector_info)
{
  core_.set_info(std::move(collector_info));
}

template <typename MsgTraits>
std::shared_ptr<CollectorInfoBase> CloudCollector<MsgTraits>::get_info() const
{
  return core_.get_info();
}

template <typename MsgTraits>
bool CloudCollector<MsgTraits>::topic_exists(const std::string & topic_name)
{
  return core_.has_topic(topic_name);
}

template <typename MsgTraits>
void CloudCollector<MsgTraits>::process_pointcloud(
  const std::string & topic_name, typename MsgTraits::PointCloudMessage::ConstSharedPtr cloud)
{
  const auto result = core_.add(topic_name, std::move(cloud));

  if (result.started) {
    // First cloud of a new group: start counting towards the timeout.
    timer_->reset();
  }
  if (result.duplicate_topic) {
    // Shouldn't happen if the parameter 'lidar_timestamp_noise_window' is set correctly.
    RCLCPP_WARN_STREAM_THROTTLE(
      ros2_parent_node_->get_logger(), *ros2_parent_node_->get_clock(),
      std::chrono::milliseconds(10000).count(),
      "Topic '" << topic_name
                << "' already exists in the collector. Check the timestamp of the pointcloud.");
  }
  if (result.ready_to_concatenate) {
    concatenate_callback();
  }
}

template <typename MsgTraits>
CollectorStatus CloudCollector<MsgTraits>::get_status() const
{
  return core_.status();
}

template <typename MsgTraits>
void CloudCollector<MsgTraits>::concatenate_callback()
{
  if (debug_mode_) {
    show_debug_message();
  }

  // All pointclouds are received or the timer has timed out, cancel the timer and concatenate the
  // pointclouds in the collector.
  timer_->cancel();

  auto concatenated_cloud_result = concatenate_pointclouds(core_.topic_to_cloud_map());

  ros2_parent_node_->publish_clouds(std::move(concatenated_cloud_result), core_.get_info());

  // Optional allocation happens immediately after the publisher
  // since it is one of th heavier operations.
  combine_cloud_handler_->allocate_pointclouds();

  core_.mark_finished();
}

template <typename MsgTraits>
ConcatenatedCloudResult<typename MsgTraits::PointCloudMessage>
CloudCollector<MsgTraits>::concatenate_pointclouds(
  std::unordered_map<std::string, typename MsgTraits::PointCloudMessage::ConstSharedPtr>
    topic_to_cloud_map)
{
  return combine_cloud_handler_->combine_pointclouds(topic_to_cloud_map, core_.get_info());
}

template <typename MsgTraits>
std::unordered_map<std::string, typename MsgTraits::PointCloudMessage::ConstSharedPtr>
CloudCollector<MsgTraits>::get_topic_to_cloud_map()
{
  return core_.topic_to_cloud_map();
}

template <typename MsgTraits>
void CloudCollector<MsgTraits>::show_debug_message()
{
  auto time_until_trigger = timer_->time_until_trigger();
  std::stringstream log_stream;
  log_stream << std::fixed << std::setprecision(6);
  log_stream << "Collector's concatenate callback time: "
             << ros2_parent_node_->get_clock()->now().seconds() << " seconds\n";

  const auto collector_info = core_.get_info();
  if (auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(collector_info)) {
    log_stream << "Advanced strategy:\n Collector's reference time min: "
               << advanced_info->timestamp - advanced_info->noise_window
               << " to max: " << advanced_info->timestamp + advanced_info->noise_window
               << " seconds\n";
  } else if (auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(collector_info)) {
    log_stream << "Naive strategy:\n Collector's timestamp: " << naive_info->timestamp
               << " seconds\n";
  }

  log_stream << "Time until trigger: " << (time_until_trigger.count() / 1e9) << " seconds\n";

  log_stream << "Pointclouds: [";
  std::string separator = "";
  for (const auto & [topic, cloud] : core_.topic_to_cloud_map()) {
    log_stream << separator;
    log_stream << "[" << topic << ", " << rclcpp::Time(cloud->header.stamp).seconds() << "]";
    separator = ", ";
  }

  log_stream << "]\n";

  const std::string & str = log_stream.str();
  RCLCPP_INFO(ros2_parent_node_->get_logger(), "%s", str.c_str());
}

template <typename MsgTraits>
void CloudCollector<MsgTraits>::reset()
{
  core_.reset();

  if (timer_ && !timer_->is_canceled()) {
    timer_->cancel();
  }
}

}  // namespace autoware::pointcloud_preprocessor
