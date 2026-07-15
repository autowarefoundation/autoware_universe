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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__TYPES_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__TYPES_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_lane_event_classifier/msg/driving_state.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/primitives/Point.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::lane_event_classifier
{

using autoware_lane_event_classifier::msg::DrivingState;

/** @brief Per-cycle snapshot of the node's subscribed inputs plus the derived ego footprint. */
struct LaneEventInput
{
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory_ptr;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_ptr;
  std::unique_ptr<autoware::vehicle_info_utils::VehicleInfo> vehicle_info_ptr;
  std::vector<lanelet::BasicPoint2d> footprint;
  // Turn-indicator report (DISABLE when unavailable); the lane-change blinker confidence signal.
  uint8_t turn_indicator{autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE};
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__TYPES_HPP_
