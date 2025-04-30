// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_DATA_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_DATA_HPP_

#include "autoware_planning_validator/msg/planning_validator_status.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::planning_validator
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_validator::msg::PlanningValidatorStatus;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct PlanningValidatorData
{
  explicit PlanningValidatorData(rclcpp::Node & node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
  {
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  Trajectory::ConstSharedPtr current_trajectory;
  Trajectory::ConstSharedPtr resampled_current_trajectory;
  Trajectory::ConstSharedPtr last_valid_trajectory;

  std::optional<size_t> nearest_point_index;
  std::optional<size_t> nearest_segment_index;

  PlanningValidatorStatus validation_status;

  Odometry::ConstSharedPtr current_kinematics;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration;

  bool is_ready(std::string & msg)
  {
    if (!current_trajectory) {
      msg = "current_trajectory";
      return false;
    }
    if (!current_kinematics) {
      msg = "current_kinematics";
      return false;
    }
    if (!current_acceleration) {
      msg = "current_acceleration";
      return false;
    }
    return true;
  }

  std::optional<size_t> get_current_nearest_point_index()
  {
    if (!current_trajectory || !current_kinematics) {
      return std::nullopt;
    }
    if (!nearest_point_index) {
      nearest_point_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        current_trajectory->points, current_kinematics->pose.pose);
    }
    return nearest_point_index;
  }

  std::optional<size_t> get_current_nearest_segment_index()
  {
    if (!current_trajectory || !current_kinematics) {
      return std::nullopt;
    }
    if (!nearest_segment_index && current_kinematics) {
      nearest_segment_index = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        current_trajectory->points, current_kinematics->pose.pose);
    }
    return nearest_segment_index;
  }
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_DATA_HPP_
