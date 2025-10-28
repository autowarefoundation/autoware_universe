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

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::object_sorter
{
using autoware_perception_msgs::msg::DetectedObjects;

template <typename ObjsMsgType>
ObjectSorterBase<ObjsMsgType>::ObjectSorterBase(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // Node Parameter
  range_calc_frame_id_ = declare_parameter<std::string>("range_calc_frame_id");
  range_calc_offset_x_ = declare_parameter<double>("range_calc_offset.x");
  range_calc_offset_y_ = declare_parameter<double>("range_calc_offset.y");

  bool use_distance_thresholding =
    declare_parameter<std::string>("range_thresholding_mode") == "distance";

  // Read the class dependent parameters
  setupSortTarget(use_distance_thresholding);

  // Subscriber
  sub_objects_ = create_subscription<ObjsMsgType>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObjectSorterBase::objectCallback, this, std::placeholders::_1));

  // Publisher
  pub_output_objects_ = create_publisher<ObjsMsgType>("~/output/objects", rclcpp::QoS{1});
}

template <typename ObjsMsgType>
void ObjectSorterBase<ObjsMsgType>::setupSortTarget(bool use_distance_thresholding)
{
  const std::array<std::string, 8> label_names{"UNKNOWN", "CAR",        "TRUCK",   "BUS",
                                               "TRAILER", "MOTORCYCLE", "BICYCLE", "PEDESTRIAN"};
  const std::array<uint8_t, 8> label_number{Label::UNKNOWN, Label::CAR,       Label::TRUCK,
                                            Label::BUS,     Label::TRAILER,   Label::MOTORCYCLE,
                                            Label::BICYCLE, Label::PEDESTRIAN};

  // read each label settings
  for (size_t i = 0; i < label_names.size(); i++) {
    std::string sort_target_label = "sort_target." + label_names[i];
    LabelSettings label_sttings;

    label_sttings.publish = declare_parameter<bool>(sort_target_label + ".publish");
    label_sttings.min_velocity =
      declare_parameter<double>(sort_target_label + ".min_velocity_threshold");

    if (use_distance_thresholding) {
      const double max_dist =
        declare_parameter<double>(sort_target_label + ".range_threshold.max_distance");
      const double min_dist =
        declare_parameter<double>(sort_target_label + ".range_threshold.min_distance");

      const double max_dist_sq = max_dist * max_dist;
      const double min_dist_sq = min_dist * min_dist;

      // Check distance
      label_sttings.isInTargetRange = [min_dist_sq, max_dist_sq](double dx, double dy) {
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq || dist_sq > max_dist_sq) {
          // Outside the target distance
          return false;
        } else {
          return true;
        }
      };
    } else {
      const double max_x = declare_parameter<double>(sort_target_label + ".range_threshold.max_x");
      const double min_x = declare_parameter<double>(sort_target_label + ".range_threshold.min_x");
      const double max_y = declare_parameter<double>(sort_target_label + ".range_threshold.max_y");
      const double min_y = declare_parameter<double>(sort_target_label + ".range_threshold.min_y");

      // Check object's relative position
      label_sttings.isInTargetRange = [min_x, max_x, min_y, max_y](double dx, double dy) {
        if (
          // Outside x
          dx < min_x || dx > max_x ||
          // Outside y
          dy < min_y || dy > max_y) {
          // Outside the target area
          return false;
        } else {
          return true;
        }
      };
    }

    label_sttings_[label_number[i]] = label_sttings;
  }
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
      input_msg->header.frame_id, range_calc_frame_id_, input_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
    // Use the ego's position in the topic's frame id for computing the distance
    ego_pos = ts.transform.translation;
    transform_success = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
  }

  double dist_origin_x = ego_pos.x + range_calc_offset_x_;
  double dist_origin_y = ego_pos.y + range_calc_offset_y_;

  for (const auto & object : input_msg->objects) {
    const LabelSettings & label_sttings = label_sttings_[object.classification.front().label];

    if (!label_sttings.publish) {
      continue;
    }

    if (!label_sttings.isInTargetVelocity(object.kinematics.twist_with_covariance.twist.linear)) {
      continue;
    }

    if (transform_success) {
      const double object_diff_x =
        object.kinematics.pose_with_covariance.pose.position.x - dist_origin_x;
      const double object_diff_y =
        object.kinematics.pose_with_covariance.pose.position.y - dist_origin_y;

      if (!label_sttings.isInTargetRange(object_diff_x, object_diff_y)) {
        continue;
      }
    }

    output_objects.objects.push_back(object);
  }

  // Publish
  pub_output_objects_->publish(output_objects);
}

// Explicit instantiation
template class ObjectSorterBase<autoware_perception_msgs::msg::DetectedObjects>;
template class ObjectSorterBase<autoware_perception_msgs::msg::TrackedObjects>;

}  // namespace autoware::object_sorter
