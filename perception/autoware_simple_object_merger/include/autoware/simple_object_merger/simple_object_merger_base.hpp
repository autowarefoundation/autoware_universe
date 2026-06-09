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

#include <autoware/agnocast_wrapper/message_filters.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::simple_object_merger
{

template <class ObjsMsgType>
class SimpleObjectMergerBase : public autoware::agnocast_wrapper::Node
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
  AUTOWARE_SUBSCRIPTION_PTR(ObjsMsgType) sub_objects_ {};
  std::vector<AUTOWARE_SUBSCRIPTION_PTR(ObjsMsgType)> sub_objects_array{};

  // Subscriber by message_filter
  autoware::agnocast_wrapper::message_filters::Subscriber<ObjsMsgType> input0_{};
  autoware::agnocast_wrapper::message_filters::Subscriber<ObjsMsgType> input1_{};
  using SyncPolicy = autoware::agnocast_wrapper::message_filters::sync_policies::ApproximateTime<
    ObjsMsgType, ObjsMsgType>;
  using Sync = autoware::agnocast_wrapper::message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // Timer
  AUTOWARE_TIMER_PTR timer_{};

  // Parameter Server
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // Process callbacks
  virtual void approximateMerger(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(ObjsMsgType) & object_msg0,
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(ObjsMsgType) & object_msg1);

  virtual void onTimer();

  void onData(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(ObjsMsgType) & msg, size_t array_number);

  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

protected:
  // Publisher
  AUTOWARE_PUBLISHER_PTR(ObjsMsgType) pub_objects_ {};

  std::shared_ptr<autoware_utils::WrapperTransformListener> tf_listener_;

  // Data Buffer
  std::vector<AUTOWARE_MESSAGE_CONST_SHARED_PTR(ObjsMsgType)> objects_data_{};

  // Core
  size_t input_topic_size_;

  // Parameter
  NodeParam node_param_{};

  bool shouldLogThrottle(
    size_t index, const rclcpp::Time & now, std::vector<rclcpp::Time> & last_log_times,
    double throttle_interval_sec);

  typename ObjsMsgType::SharedPtr getTransformedObjects(
    AUTOWARE_MESSAGE_CONST_SHARED_PTR(ObjsMsgType) objects, const std::string & target_frame_id,
    geometry_msgs::msg::TransformStamped::ConstSharedPtr transform);
};

}  // namespace autoware::simple_object_merger

#endif  // AUTOWARE__SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_BASE_HPP_
