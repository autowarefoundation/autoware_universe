// Copyright 2025 TIER IV, Inc.
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

#include "cloud_collector_core.hpp"
#include "collector_info.hpp"
#include "combine_cloud_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
class PointCloudConcatenateDataSynchronizerComponentTemplated;

template <typename MsgTraits>
class CombineCloudHandler;

// The ROS half of the collector: a timeout timer, the throttled warnings, and the hand-off to the
// node's publisher. The state machine itself lives in CloudCollectorCore.
template <typename MsgTraits>
class CloudCollector
{
public:
  CloudCollector(
    std::shared_ptr<PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>> &&
      ros2_parent_node,
    std::shared_ptr<CombineCloudHandler<typename MsgTraits::PointCloudMessage>> &
      combine_cloud_handler,
    int num_of_clouds, double timeout_sec, bool debug_mode);
  bool topic_exists(const std::string & topic_name);
  void process_pointcloud(
    const std::string & topic_name, typename MsgTraits::PointCloudMessage::ConstSharedPtr cloud);
  void concatenate_callback();

  ConcatenatedCloudResult<typename MsgTraits::PointCloudMessage> concatenate_pointclouds(
    std::unordered_map<std::string, typename MsgTraits::PointCloudMessage::ConstSharedPtr>
      topic_to_cloud_map);

  std::unordered_map<std::string, typename MsgTraits::PointCloudMessage::ConstSharedPtr>
  get_topic_to_cloud_map();

  [[nodiscard]] CollectorStatus get_status() const;

  void set_info(std::shared_ptr<CollectorInfoBase> collector_info);
  [[nodiscard]] std::shared_ptr<CollectorInfoBase> get_info() const;
  void show_debug_message();
  void reset();

private:
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponentTemplated<MsgTraits>>
    ros2_parent_node_;
  std::shared_ptr<CombineCloudHandler<typename MsgTraits::PointCloudMessage>>
    combine_cloud_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  CloudCollectorCore<typename MsgTraits::PointCloudMessage> core_;
  bool debug_mode_;
};

}  // namespace autoware::pointcloud_preprocessor

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.ipp"
