// Copyright 2022 The Autoware Contributors
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

#include "pose_error_check_module.hpp"

namespace autoware::pose_initializer
{
PoseErrorCheckModule::PoseErrorCheckModule(rclcpp::Node * node) : node_(node)
{
  pose_error_threshold_ = node_->declare_parameter<double>("pose_error_threshold");
}

bool PoseErrorCheckModule::check_pose_error(
  const geometry_msgs::msg::Pose & reference_pose, const geometry_msgs::msg::Pose & result_pose,
  double & error_2d)
{
  const double diff_pose_x = reference_pose.position.x - result_pose.position.x;
  const double diff_pose_y = reference_pose.position.y - result_pose.position.y;
  error_2d = std::sqrt(std::pow(diff_pose_x, 2) + std::pow(diff_pose_y, 2));

  if (pose_error_threshold_ <= error_2d) {
    RCLCPP_INFO(node_->get_logger(), "Pose Error is Large. Error is %f", error_2d);
    return false;
  }

  return true;
}

}  // namespace autoware::pose_initializer
