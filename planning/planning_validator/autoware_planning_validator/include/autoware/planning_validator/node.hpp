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

#ifndef AUTOWARE__PLANNING_VALIDATOR__NODE_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__NODE_HPP_

#include "autoware/planning_validator/debug_marker.hpp"
#include "autoware/planning_validator/manager.hpp"
#include "autoware/planning_validator/types.hpp"
#include "autoware_planning_validator/msg/planning_validator_status.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_validator::msg::PlanningValidatorStatus;
using autoware_utils::StopWatch;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

class PlanningValidatorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit PlanningValidatorNode(const rclcpp::NodeOptions & options);

private:
  void onTrajectory(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(Trajectory) & traj_msg);
  void setupParameters();
  void setData(const Trajectory::ConstSharedPtr & traj_msg);
  bool isDataReady();

  void validate(const std::shared_ptr<const PlanningValidatorData> & data);

  void publishProcessingTime(const double processing_time_ms);
  void publishTrajectory();
  void publishDebugInfo();
  void displayStatus();

  // subscriber
  AUTOWARE_SUBSCRIPTION_PTR(Trajectory) sub_trajectory_;
  AUTOWARE_POLLING_SUBSCRIBER_PTR(LaneletRoute, autoware::agnocast_wrapper::polling_policy::Newest)
  sub_route_ =
    create_polling_subscriber<LaneletRoute, autoware::agnocast_wrapper::polling_policy::Newest>(
      "~/input/route", rclcpp::QoS{1}.transient_local());
  AUTOWARE_POLLING_SUBSCRIBER_PTR(LaneletMapBin, autoware::agnocast_wrapper::polling_policy::Newest)
  sub_lanelet_map_bin_ =
    create_polling_subscriber<LaneletMapBin, autoware::agnocast_wrapper::polling_policy::Newest>(
      "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local());
  AUTOWARE_POLLING_SUBSCRIBER_PTR(PointCloud2)
  sub_pointcloud_ = create_polling_subscriber<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)));
  AUTOWARE_POLLING_SUBSCRIBER_PTR(Odometry)
  sub_kinematics_ = create_polling_subscriber<Odometry>("~/input/kinematics");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(AccelWithCovarianceStamped)
  sub_acceleration_ = create_polling_subscriber<AccelWithCovarianceStamped>("~/input/acceleration");
  AUTOWARE_POLLING_SUBSCRIBER_PTR(OperationModeState)
  sub_operational_state_ = create_polling_subscriber<OperationModeState>(
    "~/input/operational_mode_state", rclcpp::QoS{1}.transient_local());
  AUTOWARE_POLLING_SUBSCRIBER_PTR(TrafficLightGroupArray)
  sub_traffic_signals_ =
    create_polling_subscriber<TrafficLightGroupArray>("~/input/traffic_signals");

  // publisher
  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_traj_;
  AUTOWARE_PUBLISHER_PTR(PlanningValidatorStatus) pub_status_;
  AUTOWARE_PUBLISHER_PTR(Float64Stamped) pub_processing_time_ms_;
  AUTOWARE_PUBLISHER_PTR(visualization_msgs::msg::MarkerArray) pub_markers_;

  PlanningValidatorManager manager_;

  std::shared_ptr<PlanningValidatorContext> context_;

  bool is_critical_error_ = false;
  bool flag_autonomous_control_enabled_ = false;

  bool isAllValid(const PlanningValidatorStatus & status) const;
  bool infer_autonomous_control_state(const OperationModeState::ConstSharedPtr msg);

  Trajectory::ConstSharedPtr soft_stop_trajectory_;

  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  std::unique_ptr<
    autoware_utils_debug::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;

  StopWatch<std::chrono::milliseconds> stop_watch_;
};
}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__NODE_HPP_
