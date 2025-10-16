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

#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware/boundary_departure_checker/utils.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using nav_msgs::msg::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using BoundaryThreshold = boundary_departure_checker::Side<double>;
using autoware_control_msgs::msg::Control;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_utils_geometry::LinearRing2d;
using autoware_utils_geometry::Segment2d;
using autoware_vehicle_msgs::msg::SteeringReport;
using boundary_departure_checker::BoundarySideWithIdx;
using boundary_departure_checker::EgoSide;
using boundary_departure_checker::EgoSides;
using boundary_departure_checker::Footprint;
using boundary_departure_checker::Footprints;
using boundary_departure_checker::g_side_keys;
using boundary_departure_checker::ProjectionsToBound;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using BDCParam = boundary_departure_checker::Param;
using SegmentWithIdx = boundary_departure_checker::SegmentWithIdx;
using UncrossableBoundRTree = boundary_departure_checker::UncrossableBoundRTree;
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace bg = boost::geometry;                             // NOLINT
namespace bgi = boost::geometry::index;                     // NOLINT
namespace trajectory = autoware::experimental::trajectory;  // NOLINT
namespace bdc_utils = boundary_departure_checker::utils;    // NOLINT
namespace polling_policy = autoware_utils::polling_policy;  // NOLINT

using autoware_utils::create_default_marker;     // NOLINT
using autoware_utils::create_marker_color;       // NOLINT
using autoware_utils::create_marker_scale;       // NOLINT
using autoware_utils::get_or_declare_parameter;  // NOLINT
using autoware_utils::ProcessingTimePublisher;   // NOLINT
using autoware_utils::StopWatch;                 // NOLINT
using autoware_utils::to_msg;                    // NOLINT
using autoware_utils_geometry::Point2d;          // NOLINT
using vehicle_info_utils::VehicleInfo;           // NOLINT

using boundary_departure_checker::Abnormalities;             // NOLINT
using boundary_departure_checker::AbnormalityType;           // NOLINT
using boundary_departure_checker::BoundaryDepartureChecker;  // NOLINT
using boundary_departure_checker::CriticalDeparturePoints;   // NOLINT
using boundary_departure_checker::DepartureInterval;         // NOLINT
using boundary_departure_checker::DepartureIntervals;        // NOLINT
using boundary_departure_checker::DeparturePoint;            // NOLINT
using boundary_departure_checker::DeparturePoints;           // NOLINT
using boundary_departure_checker::DepartureType;             // NOLINT
using boundary_departure_checker::FootprintMargin;           // NOLINT
using boundary_departure_checker::ProjectionToBound;         // NOLINT
using boundary_departure_checker::Side;                      // NOLINT
using boundary_departure_checker::SideKey;                   // NOLINT
using boundary_departure_checker::UncrossableBoundRTree;     // NOLINT

using boundary_departure_checker::AbnormalitiesConfigs;
using boundary_departure_checker::AbnormalitiesData;
using boundary_departure_checker::AbnormalityConfig;
using boundary_departure_checker::ClosestProjectionsToBound;
using boundary_departure_checker::ClosestProjectionToBound;
using boundary_departure_checker::CriticalDeparturePoint;
using boundary_departure_checker::LocalizationConfig;
using boundary_departure_checker::LongitudinalConfig;
using boundary_departure_checker::NormalConfig;
using boundary_departure_checker::SteeringConfig;
using boundary_departure_checker::TriggerThreshold;
}  // namespace autoware::motion_velocity_planner::experimental

#endif  // TYPE_ALIAS_HPP_
