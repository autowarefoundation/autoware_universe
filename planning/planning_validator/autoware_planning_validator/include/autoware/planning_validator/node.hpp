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

#include <autoware_utils/ros/logger_level_configure.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using autoware_internal_debug_msgs::msg::Float64Stamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_validator::msg::PlanningValidatorStatus;
using autoware_utils::StopWatch;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class PlanningValidatorNode : public rclcpp::Node
{
public:
  explicit PlanningValidatorNode(const rclcpp::NodeOptions & options);

private:
  void onTimer();
  void setupParameters();
  void setData();
  bool isDataReady();

  void validate(const std::shared_ptr<const PlanningValidatorData> & data);

  void publishProcessingTime(const double processing_time_ms);
  void publishTrajectory();
  void publishDebugInfo();
  void displayStatus();

  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "~/input/kinematics"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> sub_acceleration_{
    this, "~/input/acceleration"};
  autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{
    this, "~/input/trajectory"};
  autoware_utils::InterProcessPollingSubscriber<PointCloud2> sub_pointcloud_{
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    sub_route_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    sub_lanelet_map_bin_{this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local()};
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<PlanningValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_ms_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  PlanningValidatorManager manager_;

  std::shared_ptr<PlanningValidatorContext> context_;

  bool is_critical_error_ = false;

  bool isAllValid(const PlanningValidatorStatus & status) const;

  Trajectory::ConstSharedPtr soft_stop_trajectory_;

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  StopWatch<std::chrono::milliseconds> stop_watch_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__NODE_HPP_
