// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_HPP_

#include "autoware/planning_validator/debug_marker.hpp"
#include "autoware/planning_validator/parameters.hpp"
#include "autoware_planning_validator/msg/planning_validator_status.hpp"
#include "autoware_utils/ros/logger_level_configure.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_utils/system/stop_watch.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware_utils/ros/published_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
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

class PlanningValidator : public rclcpp::Node
{
public:
  explicit PlanningValidator(const rclcpp::NodeOptions & options);

  void onTrajectory(const Trajectory::ConstSharedPtr msg);

  bool checkValidSize(const Trajectory & trajectory);
  bool checkValidFiniteValue(const Trajectory & trajectory);
  bool checkValidInterval(const Trajectory & trajectory);
  bool checkValidRelativeAngle(const Trajectory & trajectory);
  bool checkValidCurvature(const Trajectory & trajectory);
  bool checkValidLateralAcceleration(const Trajectory & trajectory);
  bool checkValidMaxLongitudinalAcceleration(const Trajectory & trajectory);
  bool checkValidMinLongitudinalAcceleration(const Trajectory & trajectory);
  bool checkValidSteering(const Trajectory & trajectory);
  bool checkValidSteeringRate(const Trajectory & trajectory);
  bool checkValidVelocityDeviation(const Trajectory & trajectory);
  bool checkValidDistanceDeviation(const Trajectory & trajectory);
  bool checkValidLongitudinalDistanceDeviation(const Trajectory & trajectory);
  bool checkValidForwardTrajectoryLength(const Trajectory & trajectory);
  bool checkValidLatency(const Trajectory & trajectory);
  bool checkValidYawDeviation(const Trajectory & trajectory);
  bool checkTrajectoryShift(
    const Trajectory & trajectory, const Trajectory & prev_trajectory,
    const geometry_msgs::msg::Pose & ego_pose);

private:
  void setupDiag();

  void setupParameters();

  bool isDataReady();

  void validate(
    const Trajectory & trajectory, const std::optional<Trajectory> & prev_trajectory = {});

  void publishProcessingTime(const double processing_time_ms);
  void publishTrajectory();
  void publishDebugInfo();
  void displayStatus();

  void setStatus(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg,
    const bool is_critical = false);

  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "~/input/kinematics"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped> sub_acceleration_{
    this, "~/input/acceleration"};
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
  rclcpp::Publisher<PlanningValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_ms_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  bool is_critical_error_ = false;

  std::shared_ptr<Updater> diag_updater_ = nullptr;

  PlanningValidatorStatus validation_status_;
  Params params_;  // for thresholds

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  bool isAllValid(const PlanningValidatorStatus & status) const;

  Trajectory::ConstSharedPtr current_trajectory_;
  Trajectory::ConstSharedPtr previous_published_trajectory_;
  Trajectory::ConstSharedPtr soft_stop_trajectory_;

  Odometry::ConstSharedPtr current_kinematics_;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_;

  std::shared_ptr<PlanningValidatorDebugMarkerPublisher> debug_pose_publisher_;

  std::unique_ptr<autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  StopWatch<std::chrono::milliseconds> stop_watch_;
};
}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR__PLANNING_VALIDATOR_HPP_
