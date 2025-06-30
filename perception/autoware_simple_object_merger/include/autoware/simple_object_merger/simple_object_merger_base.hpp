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

#ifndef AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_BASE_HPP_
#define AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_BASE_HPP_

#include "autoware_utils/ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::simple_object_merger
{

template <class ObjsMsgType>
class SimpleObjectMergerBase : public rclcpp::Node
{
public:
  explicit SimpleObjectMergerBase(
    const std::string & node_name, const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    double timeout_threshold{};
    std::vector<std::string> topic_names{};
    std::string new_frame_id{};
  };

private:
  // Subscriber
  typename rclcpp::Subscription<ObjsMsgType>::SharedPtr sub_objects_{};
  std::vector<typename rclcpp::Subscription<ObjsMsgType>::SharedPtr> sub_objects_array{};

  // Subscriber by message_filter
  typename message_filters::Subscriber<ObjsMsgType> input0_{};
  typename message_filters::Subscriber<ObjsMsgType> input1_{};
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ObjsMsgType, ObjsMsgType>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Process callbacks
  virtual void approximateMerger(
    const typename ObjsMsgType::ConstSharedPtr & object_msg0,
    const typename ObjsMsgType::ConstSharedPtr & object_msg1);

  virtual void onTimer();

  void onData(const typename ObjsMsgType::ConstSharedPtr msg, size_t array_number);

  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

protected:
  // Publisher
  typename rclcpp::Publisher<ObjsMsgType>::SharedPtr pub_objects_{};

  std::shared_ptr<autoware_utils::TransformListener> transform_listener_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Data Buffer
  std::vector<typename ObjsMsgType::ConstSharedPtr> objects_data_{};

  // Core
  size_t input_topic_size_;

  // Parameter
  NodeParam node_param_{};

  bool isDataReady();
  bool shouldLogThrottle(
    size_t index, const rclcpp::Time & now, std::vector<rclcpp::Time> & last_log_times,
    double throttle_interval_sec);

  typename ObjsMsgType::SharedPtr getTransformedObjects(
    typename ObjsMsgType::ConstSharedPtr objects, const std::string & target_frame_id,
    geometry_msgs::msg::TransformStamped::ConstSharedPtr transform);
};

}  // namespace autoware::simple_object_merger

#endif  // AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_BASE_HPP_
