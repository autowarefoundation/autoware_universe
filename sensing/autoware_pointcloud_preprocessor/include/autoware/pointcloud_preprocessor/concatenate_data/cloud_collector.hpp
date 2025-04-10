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

#include "combine_cloud_handler.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent;
class CombineCloudHandler;

struct CollectorInfoBase
{
  virtual ~CollectorInfoBase() = default;
};

struct NaiveCollectorInfo : public CollectorInfoBase
{
  double timestamp;

  explicit NaiveCollectorInfo(double timestamp = 0.0) : timestamp(timestamp) {}
};

struct AdvancedCollectorInfo : public CollectorInfoBase
{
  double timestamp;
  double noise_window;

  explicit AdvancedCollectorInfo(double timestamp = 0.0, double noise_window = 0.0)
  : timestamp(timestamp), noise_window(noise_window)
  {
  }
};

enum class CollectorStatus { Idle, Processing, Finished };

class CloudCollector
{
public:
  CloudCollector(
    std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> && ros2_parent_node,
    std::shared_ptr<CombineCloudHandler> & combine_cloud_handler, int num_of_clouds,
    double timeout_sec, bool debug_mode);
  bool topic_exists(const std::string & topic_name);
  void process_pointcloud(
    const std::string & topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void concatenate_callback();

  ConcatenatedCloudResult concatenate_pointclouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map);

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
  get_topic_to_cloud_map();

  [[nodiscard]] CollectorStatus get_status() const;

  void set_info(std::shared_ptr<CollectorInfoBase> collector_info);
  [[nodiscard]] std::shared_ptr<CollectorInfoBase> get_info() const;
  void show_debug_message();
  void reset();

private:
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> ros2_parent_node_;
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map_;
  uint64_t num_of_clouds_;
  double timeout_sec_;
  bool debug_mode_;
  std::shared_ptr<CollectorInfoBase> collector_info_;
  CollectorStatus status_;
};

}  // namespace autoware::pointcloud_preprocessor
