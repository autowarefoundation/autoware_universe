//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_NODE_HPP_

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace simple_trajectory_follower
{
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

// Thin ROS layer around SimpleTrajectoryFollower.
class SimpleTrajectoryFollowerNode : public rclcpp::Node
{
public:
  explicit SimpleTrajectoryFollowerNode(const rclcpp::NodeOptions & options);

private:
  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "~/input/kinematics"};
  autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{
    this, "~/input/trajectory"};
  rclcpp::Publisher<Control>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  Trajectory::ConstSharedPtr trajectory_;
  Odometry::ConstSharedPtr odometry_;

  SimpleTrajectoryFollower follower_;

  void on_timer();
  bool process_data();
};

}  // namespace simple_trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_NODE_HPP_
