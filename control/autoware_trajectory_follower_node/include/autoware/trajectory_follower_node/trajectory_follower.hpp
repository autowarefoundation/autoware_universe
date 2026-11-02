// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_interface.hpp"

#include <rclcpp/time.hpp>

#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <optional>

namespace autoware::motion::control::trajectory_follower_node
{
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/**
 * @brief merge lateral and longitudinal horizons
 * @details If one of the commands has only one control, repeat the control to match the other
 *          horizon. If each horizon has different time intervals, resample them to match the size
 *          with the greatest common divisor.
 * @param lateral_horizon lateral horizon
 * @param longitudinal_horizon longitudinal horizon
 * @param stamp stamp
 * @return merged control horizon
 */
std::optional<autoware_control_msgs::msg::ControlHorizon> mergeLatLonHorizon(
  const trajectory_follower::LateralHorizon & lateral_horizon,
  const trajectory_follower::LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp);

/// Draw how far the steering of the vehicle is from the command, for rviz.
visualization_msgs::msg::MarkerArray createDebugMarker(
  const trajectory_follower::InputData & input_data,
  const trajectory_follower::LateralOutput & lat_out, const rclcpp::Time & stamp);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
