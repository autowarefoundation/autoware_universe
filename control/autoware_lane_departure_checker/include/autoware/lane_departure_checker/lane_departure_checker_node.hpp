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
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
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
  autoware::agnocast_wrapper::polling::PollingSubscriber<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<nav_msgs::msg::Odometry>(
        this, "~/input/odometry");
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    LaneletMapBin, autoware::agnocast_wrapper::polling::polling_policy::Newest>::SharedPtr
    sub_lanelet_map_bin_ = autoware::agnocast_wrapper::polling::create_polling_subscriber<
      LaneletMapBin, autoware::agnocast_wrapper::polling::polling_policy::Newest>(
      this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local());
  autoware::agnocast_wrapper::polling::PollingSubscriber<LaneletRoute>::SharedPtr sub_route_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<LaneletRoute>(
      this, "~/input/route", rclcpp::QoS{1}.transient_local());
  autoware::agnocast_wrapper::polling::PollingSubscriber<Trajectory>::SharedPtr
    sub_reference_trajectory_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<Trajectory>(
        this, "~/input/reference_trajectory");
  autoware::agnocast_wrapper::polling::PollingSubscriber<Trajectory>::SharedPtr
    sub_predicted_trajectory_ =
      autoware::agnocast_wrapper::polling::create_polling_subscriber<Trajectory>(
        this, "~/input/predicted_trajectory");
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr sub_operation_mode_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<
      autoware_adapi_v1_msgs::msg::OperationModeState>(this, "/api/operation_mode/state");
  autoware::agnocast_wrapper::polling::PollingSubscriber<
    autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr sub_control_mode_ =
    autoware::agnocast_wrapper::polling::create_polling_subscriber<
      autoware_vehicle_msgs::msg::ControlModeReport>(this, "/vehicle/status/control_mode");

  // Data Buffer
  std::shared_ptr<const nav_msgs::msg::Odometry> current_odom_;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  std::shared_ptr<const LaneletRoute> route_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr cov_;
  std::shared_ptr<const LaneletRoute> last_route_;
  lanelet::ConstLanelets route_lanelets_;
  std::shared_ptr<const Trajectory> reference_trajectory_;
  std::shared_ptr<const Trajectory> predicted_trajectory_;
  std::shared_ptr<const autoware_adapi_v1_msgs::msg::OperationModeState> operation_mode_;
  std::shared_ptr<const autoware_vehicle_msgs::msg::ControlModeReport> control_mode_;

  // Publisher
  autoware_utils::BasicDebugPublisher<autoware::agnocast_wrapper::Node> debug_publisher_{
    this, "~/debug"};
  AUTOWARE_PUBLISHER_PTR(diagnostic_msgs::msg::DiagnosticStatus)
  processing_diag_publisher_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  processing_time_publisher_;

  void publishProcessingTimeDiag(const std::map<std::string, double> & processing_time_map);

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
