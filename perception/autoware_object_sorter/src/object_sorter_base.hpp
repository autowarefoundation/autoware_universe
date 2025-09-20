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

#ifndef OBJECT_SORTER_BASE_HPP_
#define OBJECT_SORTER_BASE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>

namespace autoware::object_sorter
{

template <typename ObjsMsgType>
class ObjectSorterBase : public rclcpp::Node
{
public:
  explicit ObjectSorterBase(
    const std::string & node_name, const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double min_velocity_threshold{};
    double min_range_threshold{};
  };

private:
  void objectCallback(const typename ObjsMsgType::ConstSharedPtr input_msg);
  void splitByVelocity(const ObjsMsgType & input_object);
  void splitByRange(const ObjsMsgType & input_object);

  // Subscriber
  typename rclcpp::Subscription<ObjsMsgType>::SharedPtr sub_objects_{};

  // Publisher
  typename rclcpp::Publisher<ObjsMsgType>::SharedPtr pub_output_objects_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameter
  NodeParam node_param_{};
  std::string range_calc_frame_id;
  double min_range_threshold_sq_;
};

}  // namespace autoware::object_sorter

#endif  // OBJECT_SORTER_BASE_HPP_
