// Copyright 2021 Tier IV, Inc.
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

#ifndef ROS_INTERFACE_HPP_
#define ROS_INTERFACE_HPP_

#include "main.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/self_pose_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

namespace autoware::path_distance_calculator
{

// Thin ROS wrapper: polls map/route/pose on a timer and forwards data to
// RouteDistanceCalculator, which holds all the lanelet2 route logic.
class PathDistanceCalculator : public rclcpp::Node
{
public:
  explicit PathDistanceCalculator(const rclcpp::NodeOptions & options);

private:
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;

  void on_timer();

  autoware_utils::InterProcessPollingSubscriber<HADMapBin> sub_map_{
    this, "~/input/map", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<LaneletRoute> sub_route_{
    this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr pub_dist_;
  rclcpp::TimerBase::SharedPtr timer_;
  autoware_utils::SelfPoseListener self_pose_listener_;

  // Last map/route handed to the calculator, so a re-polled but unchanged message (the polling
  // subscriber keeps returning the latest sample every tick) does not trigger a redundant
  // (and, for the route, expensive) recomputation.
  HADMapBin::ConstSharedPtr last_map_;
  LaneletRoute::ConstSharedPtr last_route_;

  RouteDistanceCalculator calculator_;
};

}  // namespace autoware::path_distance_calculator

#endif  // ROS_INTERFACE_HPP_
