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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autoware::trajectory_processor
{

/// @brief Stable vehicle and transform services shared by trajectory processor plugins.
struct TrajectoryProcessorContext
{
  /// @brief Construct shared services from the hosting ROS node.
  explicit TrajectoryProcessorContext(rclcpp::Node * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer}
  {
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};

}  // namespace autoware::trajectory_processor

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_PROCESSOR_CONTEXT_HPP_
