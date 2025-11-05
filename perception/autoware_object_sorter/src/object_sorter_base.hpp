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

#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::object_sorter
{
using Label = autoware_perception_msgs::msg::ObjectClassification;

template <typename ObjsMsgType>
class ObjectSorterBase : public rclcpp::Node
{
public:
  explicit ObjectSorterBase(
    const std::string & node_name, const rclcpp::NodeOptions & node_options);

  struct LabelSettings
  {
    bool publish;
    double min_velocity;

    // assign function based on the mode later
    std::function<bool(double, double)> isInTargetRange;

    bool isInTargetVelocity(const geometry_msgs::msg::Vector3 vec) const
    {
      if (std::abs(autoware_utils_geometry::calc_norm(vec)) < min_velocity) {
        // Low velocity object
        return false;
      } else {
        return true;
      }
    }
  };  // struct LabelSettings

private:
  void setupSortTarget(bool use_distance_thresholding);
  void objectCallback(const typename ObjsMsgType::ConstSharedPtr input_msg);
  void splitByVelocity(const ObjsMsgType & input_object);
  void splitByRange(const ObjsMsgType & input_object);

  // Subscriber
  typename rclcpp::Subscription<ObjsMsgType>::SharedPtr sub_objects_{};

  // Publisher
  typename rclcpp::Publisher<ObjsMsgType>::SharedPtr pub_output_objects_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameter
  std::string range_calc_frame_id_;
  double range_calc_offset_x_;
  double range_calc_offset_y_;
  std::unordered_map<uint8_t, LabelSettings> label_settings_;
};

}  // namespace autoware::object_sorter

#endif  // OBJECT_SORTER_BASE_HPP_
