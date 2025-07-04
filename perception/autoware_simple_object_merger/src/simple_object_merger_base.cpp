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

#include "autoware/simple_object_merger/simple_object_merger_base.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace autoware::simple_object_merger
{
using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

template <class ObjsMsgType>
SimpleObjectMergerBase<ObjsMsgType>::SimpleObjectMergerBase(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SimpleObjectMergerBase::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz");
  node_param_.new_frame_id = declare_parameter<std::string>("new_frame_id");
  node_param_.timeout_threshold = declare_parameter<double>("timeout_threshold");

  declare_parameter("input_topics", std::vector<std::string>());
  node_param_.topic_names = get_parameter("input_topics").as_string_array();
  if (node_param_.topic_names.empty()) {
    RCLCPP_ERROR(get_logger(), "Need a 'input_topics' parameter to be set before continuing");
    return;
  }

  input_topic_size_ = node_param_.topic_names.size();
  if (input_topic_size_ == 1) {
    RCLCPP_ERROR(get_logger(), "Only one topic given. Need at least two topics to continue.");
    return;
  }

  // Subscriber
  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);
  if (input_topic_size_ == 2) {
    // Trigger the process and publish by message_filter
    input0_.subscribe(
      this, node_param_.topic_names.at(0), rclcpp::QoS{1}.best_effort().get_rmw_qos_profile());
    input1_.subscribe(
      this, node_param_.topic_names.at(1), rclcpp::QoS{1}.best_effort().get_rmw_qos_profile());
    sync_ptr_ = std::make_shared<Sync>(SyncPolicy(10), input0_, input1_);
    sync_ptr_->registerCallback(
      std::bind(
        &SimpleObjectMergerBase::approximateMerger, this, std::placeholders::_1,
        std::placeholders::_2));
  } else {
    // Trigger the process by timer
    sub_objects_array.resize(input_topic_size_);
    objects_data_.resize(input_topic_size_);

    // subscriber
    for (size_t i = 0; i < input_topic_size_; i++) {
      std::function<void(const typename ObjsMsgType::ConstSharedPtr msg)> func =
        std::bind(&SimpleObjectMergerBase::onData, this, std::placeholders::_1, i);
      sub_objects_array.at(i) = create_subscription<ObjsMsgType>(
        node_param_.topic_names.at(i), rclcpp::QoS{1}.best_effort(), func);
    }

    // process callback
    const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), update_period_ns, std::bind(&SimpleObjectMergerBase::onTimer, this));
  }

  // Publisher
  pub_objects_ = create_publisher<ObjsMsgType>("~/output/objects", rclcpp::QoS{1}.reliable());
}

template <class ObjsMsgType>
typename ObjsMsgType::SharedPtr SimpleObjectMergerBase<ObjsMsgType>::getTransformedObjects(
  typename ObjsMsgType::ConstSharedPtr objects, const std::string & target_frame_id,
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform)
{
  typename ObjsMsgType::SharedPtr output_objects = std::const_pointer_cast<ObjsMsgType>(objects);

  if (objects->header.frame_id == target_frame_id) {
    return output_objects;
  }

  output_objects->header.frame_id = target_frame_id;
  for (auto & object : output_objects->objects) {
    // convert by tf
    geometry_msgs::msg::PoseStamped pose_stamped{};
    pose_stamped.pose = object.kinematics.pose_with_covariance.pose;
    geometry_msgs::msg::PoseStamped transformed_pose_stamped{};
    tf2::doTransform(pose_stamped, transformed_pose_stamped, *transform);
    object.kinematics.pose_with_covariance.pose = transformed_pose_stamped.pose;
  }

  return output_objects;
}

template <class ObjsMsgType>
void SimpleObjectMergerBase<ObjsMsgType>::approximateMerger(
  [[maybe_unused]] const typename ObjsMsgType::ConstSharedPtr & object_msg0,
  [[maybe_unused]] const typename ObjsMsgType::ConstSharedPtr & object_msg1)
{
  // This must be overridden
}

template <class ObjsMsgType>
void SimpleObjectMergerBase<ObjsMsgType>::onData(
  typename ObjsMsgType::ConstSharedPtr msg, const size_t array_number)
{
  objects_data_.at(array_number) = msg;
}

template <class ObjsMsgType>
rcl_interfaces::msg::SetParametersResult SimpleObjectMergerBase<ObjsMsgType>::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  try {
    // Node Parameter
    {
      auto & p = node_param_;
      // Update params
      update_param(params, "update_rate_hz", p.update_rate_hz);
      update_param(params, "timeout_threshold", p.timeout_threshold);
      update_param(params, "new_frame_id", p.new_frame_id);
      update_param(params, "topic_names", p.topic_names);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  result.successful = true;
  result.reason = "success";
  return result;
}

template <class ObjsMsgType>
bool SimpleObjectMergerBase<ObjsMsgType>::isDataReady()
{
  for (size_t i = 0; i < input_topic_size_; i++) {
    if (!objects_data_.at(i)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for object msg...");
      return false;
    }
  }

  return true;
}

template <class ObjsMsgType>
void SimpleObjectMergerBase<ObjsMsgType>::onTimer()
{
  // This must be overridden
}

template <class ObjsMsgType>
bool SimpleObjectMergerBase<ObjsMsgType>::shouldLogThrottle(
  size_t index, const rclcpp::Time & now, std::vector<rclcpp::Time> & last_log_times,
  double throttle_interval_sec)
{
  if ((now - last_log_times[index]) > rclcpp::Duration::from_seconds(throttle_interval_sec)) {
    last_log_times[index] = now;
    return true;
  }
  return false;
}

// explicit instantiation
template class SimpleObjectMergerBase<autoware_perception_msgs::msg::DetectedObjects>;
template class SimpleObjectMergerBase<autoware_perception_msgs::msg::TrackedObjects>;

}  // namespace autoware::simple_object_merger
