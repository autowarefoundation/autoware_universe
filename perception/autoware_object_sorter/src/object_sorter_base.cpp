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

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

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
    LabelSettings label_settings;

    label_settings.publish = declare_parameter<bool>(sort_target_label + ".publish");
    label_settings.min_velocity =
      declare_parameter<double>(sort_target_label + ".min_velocity_threshold");

    if (use_distance_thresholding) {
      const double max_dist =
        declare_parameter<double>(sort_target_label + ".range_threshold.max_distance");
      const double min_dist =
        declare_parameter<double>(sort_target_label + ".range_threshold.min_distance");

      const double max_dist_sq = max_dist * max_dist;
      const double min_dist_sq = min_dist * min_dist;

      // Check distance
      label_settings.isInTargetRange = [min_dist_sq, max_dist_sq](double dx, double dy) {
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
      label_settings.isInTargetRange = [min_x, max_x, min_y, max_y](double dx, double dy) {
        return (min_x <= dx && dx <= max_x && min_y <= dy && dy <= max_y);
      };
    }

    label_settings_[label_number[i]] = label_settings;
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

  double tx;
  double ty;
  double cos_yaw;
  double sin_yaw;
  // Even when it failed to get the transform, we still can do the velocity check
  bool transform_success = false;
  try {
    const geometry_msgs::msg::TransformStamped tf_input_frame_to_target_frame =
      tf_buffer_.lookupTransform(
        range_calc_frame_id_,        // target frame
        input_msg->header.frame_id,  // source frame
        input_msg->header.stamp, rclcpp::Duration::from_seconds(0.5));

    // Extract translation
    tx = tf_input_frame_to_target_frame.transform.translation.x;
    ty = tf_input_frame_to_target_frame.transform.translation.y;

    // Extract yaw from quaternion
    const geometry_msgs::msg::Quaternion & q = tf_input_frame_to_target_frame.transform.rotation;
    const tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    const double yaw = tf2::getYaw(tf_q);

    cos_yaw = std::cos(yaw);
    sin_yaw = std::sin(yaw);

    transform_success = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
  }

  for (const auto & object : input_msg->objects) {
    const uint8_t label =
      autoware::object_recognition_utils::getHighestProbLabel(object.classification);
    const LabelSettings & label_settings = label_settings_[label];

    if (!label_settings.publish) {
      continue;
    }

    if (!label_settings.isInTargetVelocity(object.kinematics.twist_with_covariance.twist.linear)) {
      continue;
    }

    if (transform_success) {
      // We will check the condition in 2D (x-y)
      const double object_x = object.kinematics.pose_with_covariance.pose.position.x;
      const double object_y = object.kinematics.pose_with_covariance.pose.position.y;
      const double object_x_in_target_frame = object_x * cos_yaw - object_y * sin_yaw + tx;
      const double object_y_in_target_frame = object_x * sin_yaw + object_y * cos_yaw + ty;

      const double object_diff_x = object_x_in_target_frame - range_calc_offset_x_;
      const double object_diff_y = object_y_in_target_frame - range_calc_offset_y_;

      if (!label_settings.isInTargetRange(object_diff_x, object_diff_y)) {
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
