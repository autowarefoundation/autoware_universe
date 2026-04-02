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

#ifndef PROCESSOR__INPUT_MANAGER_HPP_
#define PROCESSOR__INPUT_MANAGER_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::multi_object_tracker
{

class InputStream
{
public:
  InputStream(
    const types::InputChannel & input_channel, std::shared_ptr<Odometry> odometry,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  void setTriggerFunction(std::function<void(const size_t)> func_trigger)
  {
    func_trigger_ = func_trigger;
  }

  std::optional<types::DynamicObjectList> processMessage(
    const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);
  void push(const types::DynamicObjectList & objects, const types::AssociationResult & association);
  void updateTimingStatus(const rclcpp::Time & now, const rclcpp::Time & objects_time);

  bool isTimeInitialized() const { return initial_count_ > 0; }
  uint getIndex() const { return channel_.index; }
  void getObjectsOlderThan(
    const rclcpp::Time & object_latest_time, const rclcpp::Time & object_earliest_time,
    types::ObjectsWithAssociationList & objects_with_associations);
  bool isSpawnEnabled() const { return channel_.is_spawn_enabled; }

  void getTimeStatistics(
    double & latency_mean, double & latency_var, double & interval_mean,
    double & interval_var) const
  {
    latency_mean = latency_mean_;
    latency_var = latency_var_;
    interval_mean = interval_mean_;
    interval_var = interval_var_;
  }
  rclcpp::Time getLatestMeasurementTime() const { return latest_measurement_time_; }

private:
  const types::InputChannel channel_;
  std::shared_ptr<Odometry> odometry_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  size_t que_size_{30};
  std::deque<types::ObjectsWithAssociation> objects_que_;

  std::function<void(const size_t)> func_trigger_;

  int initial_count_{0};
  double latency_mean_{};
  double latency_var_{};
  double interval_mean_{};
  double interval_var_{};

  rclcpp::Time latest_measurement_time_;
  rclcpp::Time latest_message_time_;
};

class InputManager
{
public:
  InputManager(
    std::shared_ptr<Odometry> odometry, rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);
  void init(const std::vector<types::InputChannel> & input_channels);

  void setTriggerFunction(std::function<void(size_t)> func_trigger);
  size_t getTargetChannelIdx() const { return target_stream_idx_; }
  std::optional<types::DynamicObjectList> processMessage(
    const size_t channel_index,
    const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);
  void push(
    const size_t channel_index, const types::DynamicObjectList & objects,
    const types::AssociationResult & association);

  bool getObjects(
    const rclcpp::Time & now, types::ObjectsWithAssociationList & objects_with_associations);

private:
  std::shared_ptr<Odometry> odometry_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  bool is_initialized_{false};
  rclcpp::Time latest_exported_object_time_;

  size_t input_size_{};
  std::vector<std::shared_ptr<InputStream>> input_streams_;

  std::function<void(size_t)> func_trigger_;
  uint target_stream_idx_{0};
  double target_stream_latency_{0.2};        // [s]
  double target_stream_latency_std_{0.04};   // [s]
  double target_stream_interval_{0.1};       // [s]
  double target_stream_interval_std_{0.02};  // [s]

private:
  void getObjectTimeInterval(
    const rclcpp::Time & now, rclcpp::Time & object_latest_time,
    rclcpp::Time & object_earliest_time) const;
  void optimizeTimings();
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__INPUT_MANAGER_HPP_
