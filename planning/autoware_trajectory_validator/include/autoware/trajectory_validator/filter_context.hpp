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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTER_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTER_CONTEXT_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::trajectory_validator
{

// Base context that all filters can access
struct FilterContext
{
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects;
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_light_signals;
};
}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTER_CONTEXT_HPP_
