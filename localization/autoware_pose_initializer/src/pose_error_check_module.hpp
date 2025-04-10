// Copyright 2024 The Autoware Contributors
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

#ifndef POSE_ERROR_CHECK_MODULE_HPP_
#define POSE_ERROR_CHECK_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace autoware::pose_initializer
{
class PoseErrorCheckModule
{
public:
  explicit PoseErrorCheckModule(rclcpp::Node * node);
  bool check_pose_error(
    const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & result_pose,
    double & error_2d);

private:
  rclcpp::Node * node_;
  double pose_error_threshold_;
};
}  // namespace autoware::pose_initializer

#endif  // POSE_ERROR_CHECK_MODULE_HPP_
