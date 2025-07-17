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

#include "object_sorter_base.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

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

namespace autoware::object_sorter
{
using autoware_perception_msgs::msg::DetectedObjects;

template <typename ObjsMsgType>
ObjectSorterBase<ObjsMsgType>::ObjectSorterBase(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ObjectSorterBase::onSetParam, this, std::placeholders::_1));

  // Node Parameter
  range_calc_frame_id = declare_parameter<std::string>("range_calc_frame_id");
  node_param_.min_velocity_threshold = declare_parameter<double>("min_velocity_threshold");
  node_param_.min_range_threshold = declare_parameter<double>("min_range_threshold");

  min_range_threshold_sq_ = node_param_.min_range_threshold * node_param_.min_range_threshold;

  // Subscriber
  sub_objects_ = create_subscription<ObjsMsgType>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObjectSorterBase::objectCallback, this, std::placeholders::_1));

  // Publisher
  pub_output_objects_ = create_publisher<ObjsMsgType>("~/output/objects", rclcpp::QoS{1});
}

template <typename ObjsMsgType>
void ObjectSorterBase<ObjsMsgType>::objectCallback(
  const typename ObjsMsgType::ConstSharedPtr input_msg)
{
  // Guard
  if (pub_output_objects_->get_subscription_count() < 1) {
    return;
  }

  ObjsMsgType output_objects;
  output_objects.header = input_msg->header;

  bool transform_success = false;
  geometry_msgs::msg::Vector3 ego_pos;
  try {
    const geometry_msgs::msg::TransformStamped ts = tf_buffer_.lookupTransform(
      input_msg->header.frame_id, range_calc_frame_id, input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
    // Use the ego's position in the topic's frame id for computing the distance
    ego_pos = ts.transform.translation;
    transform_success = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
  }

  for (const auto & object : input_msg->objects) {
    // Filter by velocity
    if (
      // NOTE: use the raw velocity in the topic for now
      std::abs(
        autoware_utils_geometry::calc_norm(object.kinematics.twist_with_covariance.twist.linear)) <
      node_param_.min_velocity_threshold) {
      // Low velocity object
      continue;
    }

    if (transform_success) {
      const double object_pos_x =
        object.kinematics.pose_with_covariance.pose.position.x - ego_pos.x;
      const double object_pos_y =
        object.kinematics.pose_with_covariance.pose.position.y - ego_pos.y;

      // Filter by range
      const auto object_sq_dist = object_pos_x * object_pos_x + object_pos_y * object_pos_y;
      if (object_sq_dist < min_range_threshold_sq_) {
        // Short range object
        continue;
      }
    }

    output_objects.objects.push_back(object);
  }

  // Publish
  pub_output_objects_->publish(output_objects);
}

template <typename ObjsMsgType>
rcl_interfaces::msg::SetParametersResult ObjectSorterBase<ObjsMsgType>::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    {
      auto & p = node_param_;

      update_param(params, "min_velocity_threshold", p.min_velocity_threshold);
      update_param(params, "min_range_threshold", p.min_range_threshold);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  min_range_threshold_sq_ = node_param_.min_range_threshold * node_param_.min_range_threshold;

  result.successful = true;
  result.reason = "success";
  return result;
}

// Explicit instantiation
template class ObjectSorterBase<autoware_perception_msgs::msg::DetectedObjects>;
template class ObjectSorterBase<autoware_perception_msgs::msg::TrackedObjects>;

}  // namespace autoware::object_sorter
