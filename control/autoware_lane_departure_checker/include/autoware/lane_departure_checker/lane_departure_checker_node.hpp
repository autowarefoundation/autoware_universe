// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_

#include "autoware/deprecated/boundary_departure_checker/boundary_departure_checker.hpp"
#include "autoware/lane_departure_checker/parameters.hpp"

#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/processing_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::lane_departure_checker
{
using autoware_map_msgs::msg::LaneletMapBin;
using namespace boundary_departure_checker;  // NOLINT;

class LaneDepartureCheckerNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit LaneDepartureCheckerNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  AUTOWARE_POLLING_SUBSCRIBER_PTR(nav_msgs::msg::Odometry)
  sub_odom_ = create_polling_subscriber<nav_msgs::msg::Odometry>("~/input/odometry");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(LaneletMapBin, autoware::agnocast_wrapper::polling_policy::Newest)
  sub_lanelet_map_bin_ =
    create_polling_subscriber<LaneletMapBin, autoware::agnocast_wrapper::polling_policy::Newest>(
      "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local());
  AUTOWARE_POLLING_SUBSCRIBER_PTR(LaneletRoute)
  sub_route_ =
    create_polling_subscriber<LaneletRoute>("~/input/route", rclcpp::QoS{1}.transient_local());
  AUTOWARE_POLLING_SUBSCRIBER_PTR(Trajectory)
  sub_reference_trajectory_ =
    create_polling_subscriber<Trajectory>("~/input/reference_trajectory");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(Trajectory)
  sub_predicted_trajectory_ =
    create_polling_subscriber<Trajectory>("~/input/predicted_trajectory");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_adapi_v1_msgs::msg::OperationModeState)
  sub_operation_mode_ = create_polling_subscriber<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_vehicle_msgs::msg::ControlModeReport)
  sub_control_mode_ = create_polling_subscriber<autoware_vehicle_msgs::msg::ControlModeReport>(
    "/vehicle/status/control_mode");

  // Data Buffer
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(nav_msgs::msg::Odometry) current_odom_;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) route_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr cov_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(LaneletRoute) last_route_;
  lanelet::ConstLanelets route_lanelets_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(Trajectory) reference_trajectory_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(Trajectory) predicted_trajectory_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_adapi_v1_msgs::msg::OperationModeState)
  operation_mode_;
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_vehicle_msgs::msg::ControlModeReport) control_mode_;

  // Publisher
  autoware_utils::BasicDebugPublisher<autoware::agnocast_wrapper::Node> debug_publisher_{
    this, "~/debug"};
  autoware_utils::BasicProcessingTimePublisher<autoware::agnocast_wrapper::Node>
    processing_diag_publisher_{this, "~/debug/processing_time_ms_diag"};
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  processing_time_publisher_;

  // Timer
  AUTOWARE_TIMER_PTR timer_;

  bool isDataReady();
  bool isDataTimeout();
  bool isDataValid();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;
  double vehicle_length_m_;

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  Input input_{};
  Output output_{};
  std::unique_ptr<BoundaryDepartureChecker> boundary_departure_checker_;

  // Diagnostic Updater
  autoware::agnocast_wrapper::diagnostic_updater::Updater updater_{this};

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::msg::MarkerArray createMarkerArray() const;

  // Lanelet Neighbor Search
  lanelet::ConstLanelets getAllSharedLineStringLanelets(
    const lanelet::ConstLanelet & current_lane, const bool is_right, const bool is_left,
    const bool is_opposite, const bool is_conflicting, const bool & invert_opposite);

  lanelet::ConstLanelets getAllRightSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false);

  lanelet::ConstLanelets getAllLeftSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false);

  boost::optional<lanelet::ConstLanelet> getLeftLanelet(const lanelet::ConstLanelet & lanelet);

  lanelet::Lanelets getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet);
  boost::optional<lanelet::ConstLanelet> getRightLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  lanelet::Lanelets getRightOppositeLanelets(const lanelet::ConstLanelet & lanelet);
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
