// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros_utils.hpp"

#include "bevformer_node.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

void box3DToDetectedObjects(
  const std::vector<Box3D> & boxes, autoware_perception_msgs::msg::DetectedObjects & objects_msg,
  const std::vector<std::string> & class_names, float score_threshold, bool add_twist)
{
  // Convert Box3D vector to Autoware perception message
  for (const auto & box : boxes) {
    // Skip low confidence detections
    if (box.score < score_threshold) {
      continue;
    }

    autoware_perception_msgs::msg::DetectedObject object;

    autoware_perception_msgs::msg::ObjectClassification classification;
    if (box.label >= 0 && static_cast<size_t>(box.label) < class_names.size()) {
      const std::string & class_name = class_names[box.label];

      if (class_name == "car") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
      } else if (class_name == "truck") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      } else if (class_name == "construction_vehicle") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      } else if (class_name == "bus") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      } else if (class_name == "trailer") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::TRUCK;
      } else if (class_name == "bicycle") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
      } else if (class_name == "motorcycle") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
      } else if (class_name == "pedestrian") {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      } else {
        classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      }
    } else {
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    }
    classification.probability = box.score;
    object.classification.push_back(classification);

    geometry_msgs::msg::Point p;
    p.x = box.x;
    p.y = box.y;
    p.z = box.z + box.h * 0.5f;
    object.kinematics.pose_with_covariance.pose.position = p;

    tf2::Quaternion q;
    double yaw = -(box.r) + M_PI;
    q.setRPY(0, 0, yaw);
    q.normalize();
    object.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(q);
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

    geometry_msgs::msg::Vector3 v;
    v.x = box.w;
    v.y = box.l;
    v.z = box.h;
    object.shape.dimensions = v;

    if (add_twist) {
      object.kinematics.twist_with_covariance.twist.linear.x = box.vx;
      object.kinematics.twist_with_covariance.twist.linear.y = box.vy;
    }
    objects_msg.objects.push_back(object);
  }
}

void getCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, Eigen::Matrix3d & intrinsics)
{
  intrinsics = Eigen::Matrix3d::Zero();
  intrinsics(0, 0) = static_cast<double>(msg->k[0]);  // fx
  intrinsics(0, 2) = static_cast<double>(msg->k[2]);  // cx
  intrinsics(1, 1) = static_cast<double>(msg->k[4]);  // fy
  intrinsics(1, 2) = static_cast<double>(msg->k[5]);  // cy
  intrinsics(2, 2) = 1.0;
}

void getTransform(
  const geometry_msgs::msg::TransformStamped & transform_stamped,
  Eigen::Quaternion<double> & rotation, Eigen::Translation3d & translation)
{
  rotation = Eigen::Quaternion<double>(
    static_cast<double>(transform_stamped.transform.rotation.w),
    static_cast<double>(transform_stamped.transform.rotation.x),
    static_cast<double>(transform_stamped.transform.rotation.y),
    static_cast<double>(transform_stamped.transform.rotation.z));

  translation = Eigen::Translation3d(
    static_cast<double>(transform_stamped.transform.translation.x),
    static_cast<double>(transform_stamped.transform.translation.y),
    static_cast<double>(transform_stamped.transform.translation.z));
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
