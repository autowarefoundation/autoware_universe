// Copyright 2023 TIER IV, Inc.
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

#ifndef SIMPLE_OBJECT_MERGER_NODE_HPP_
#define SIMPLE_OBJECT_MERGER_NODE_HPP_

#include "autoware_utils/ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::simple_object_merger
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

class SimpleObjectMergerNode : public rclcpp::Node
{
public:
  explicit SimpleObjectMergerNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    double timeout_threshold{};
    std::vector<std::string> topic_names{};
    std::string new_frame_id{};
  };

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> sub_objects_array{};
  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;

  // Subscriber by message_filter
  message_filters::Subscriber<DetectedObjects> input0_{};
  message_filters::Subscriber<DetectedObjects> input1_{};
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<DetectedObjects, DetectedObjects>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // Process callbacks
  void approximateMerger(
    const DetectedObjects::ConstSharedPtr & object_msg0,
    const DetectedObjects::ConstSharedPtr & object_msg1);

  void onData(const DetectedObjects::ConstSharedPtr msg, size_t array_number);

  // Data Buffer
  std::vector<DetectedObjects::ConstSharedPtr> objects_data_{};
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};
  void onTimer();
  bool isDataReady();
  bool shouldLogThrottle(
    size_t index, const rclcpp::Time & now, std::vector<rclcpp::Time> & last_log_times,
    double throttle_interval_sec);

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  size_t input_topic_size;
};

}  // namespace autoware::simple_object_merger

#endif  // SIMPLE_OBJECT_MERGER_NODE_HPP_
