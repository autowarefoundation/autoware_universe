// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "goal_pose_visualizer.hpp"

#include <utility>

namespace autoware::mission_planner_universe
{
GoalPoseVisualizer::GoalPoseVisualizer(const rclcpp::NodeOptions & node_options)
: Node("goal_pose_visualizer", node_options)
{
  sub_route_ = create_subscription<LaneletRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&GoalPoseVisualizer::echo_back_route_callback, this, std::placeholders::_1));
  pub_goal_pose_ =
    create_publisher<PoseStamped>("output/goal_pose", rclcpp::QoS{1}.transient_local());
}

void GoalPoseVisualizer::echo_back_route_callback(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) & msg)
{
  auto goal_pose = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_goal_pose_);
  goal_pose->header = msg->header;
  goal_pose->pose = msg->goal_pose;
  pub_goal_pose_->publish(std::move(goal_pose));
}
}  // namespace autoware::mission_planner_universe

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner_universe::GoalPoseVisualizer)
