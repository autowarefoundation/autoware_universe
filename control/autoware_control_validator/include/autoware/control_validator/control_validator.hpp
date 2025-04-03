// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_

#include "autoware/control_validator/debug_marker.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"
#include "autoware_vehicle_info_utils/vehicle_info.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_control_validator/msg/control_validator_status.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <utility>

class AccelerationValidatorTest;

namespace autoware::control_validator
{
using autoware_control_msgs::msg::Control;
using autoware_control_validator::msg::ControlValidatorStatus;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

struct ValidationParams
{
  double max_distance_deviation_threshold;
  double rolling_back_velocity;
  double over_velocity_ratio;
  double over_velocity_offset;
  double overrun_stop_point_dist;
  double nominal_latency_threshold;
};

/**
 * @class AccelerationValidator
 * @brief Validates deviation between output acceleration and measured acceleration.
 */
class AccelerationValidator
{
public:
  friend class ::AccelerationValidatorTest;
  explicit AccelerationValidator(rclcpp::Node & node)
  {
    e_offset =
      autoware_utils::get_or_declare_parameter<double>(node, "thresholds.acc_error_offset");
    e_scale = autoware_utils::get_or_declare_parameter<double>(node, "thresholds.acc_error_scale");
    const double acc_lpf_gain =
      autoware_utils::get_or_declare_parameter<double>(node, "acc_lpf_gain");
    desired_acc_lpf.setGain(acc_lpf_gain);
    measured_acc_lpf.setGain(acc_lpf_gain);
  };

  void validate(
    ControlValidatorStatus & res, const Odometry & kinematic_state, const Control & control_cmd,
    const AccelWithCovarianceStamped & loc_acc);

private:
  bool is_in_error_range() const;
  double e_offset;
  double e_scale;
  autoware::signal_processing::LowpassFilter1d desired_acc_lpf{0.0};
  autoware::signal_processing::LowpassFilter1d measured_acc_lpf{0.0};
};

/**
 * @class ControlValidator
 * @brief Validates control commands by comparing predicted trajectories against reference
 * trajectories.
 */
class ControlValidator : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit ControlValidator(const rclcpp::NodeOptions & options);

  /**
   * @brief Callback function for the control component output.
   * @param msg Control message
   */
  void on_control_cmd(const Control::ConstSharedPtr msg);

  /**
   * @brief Callback function for the predicted trajectory.
   * @param msg Predicted trajectory message
   */
  void on_predicted_trajectory(const Trajectory::ConstSharedPtr msg);

  /**
   * @brief Calculate the maximum lateral distance between the reference trajectory and predicted
   * trajectory.
   * @param predicted_trajectory Predicted trajectory
   * @param reference_trajectory Reference trajectory
   * @return A pair consisting of the maximum lateral deviation and a boolean indicating validity
   */
  std::pair<double, bool> calc_lateral_deviation_status(
    const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory) const;

  void calc_velocity_deviation_status(
    const Trajectory & reference_trajectory, const Odometry & kinematics);

  /**
   * @brief Calculate whether the vehicle has overrun a stop point in the trajectory.
   * @param reference_trajectory Reference trajectory
   * @param kinematics Current vehicle odometry including pose and twist
   */
  void calc_stop_point_overrun_status(
    const Trajectory & reference_trajectory, const Odometry & kinematics);

private:
  /**
   * @brief Setup diagnostic updater
   */
  void setup_diag();

  /**
   * @brief Setup parameters from the parameter server
   */
  void setup_parameters();

  /**
   * @brief Check if all required data is ready for validation
   * @return Boolean indicating readiness of data
   */
  bool is_data_ready();

  /**
   * @brief Validate the predicted trajectory against the reference trajectory and current
   * kinematics
   * @param predicted_trajectory Predicted trajectory
   * @param reference_trajectory Reference trajectory
   * @param kinematics Current vehicle kinematics
   */
  void validate(
    const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory,
    const Odometry & kinematics, const Control & control_cmd,
    const AccelWithCovarianceStamped & measured_acc);

  /**
   * @brief Publish debug information
   */
  void publish_debug_info();

  /**
   * @brief Display validation status on terminal
   */
  void display_status();

  /**
   * @brief Set the diagnostic status
   * @param stat Diagnostic status wrapper
   * @param is_ok Boolean indicating if the status is okay
   * @param msg Status message
   */
  void set_status(
    DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const;

  // Subscribers and publishers
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
  autoware_utils::InterProcessPollingSubscriber<Odometry>::SharedPtr sub_kinematics_;
  autoware_utils::InterProcessPollingSubscriber<Trajectory>::SharedPtr sub_reference_traj_;
  autoware_utils::InterProcessPollingSubscriber<Trajectory>::SharedPtr sub_predicted_traj_;
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    sub_measured_acc_;
  rclcpp::Publisher<ControlValidatorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_processing_time_;

  // system parameters
  int64_t diag_error_count_threshold_ = 0;
  bool display_on_terminal_ = true;

  Updater diag_updater_{this};

  ControlValidatorStatus validation_status_;
  ValidationParams validation_params_;  // for thresholds
  autoware::signal_processing::LowpassFilter1d vehicle_vel_{0.0};
  autoware::signal_processing::LowpassFilter1d target_vel_{0.0};
  AccelerationValidator acceleration_validator{*this};

  bool hold_velocity_error_until_stop_{false};

  vehicle_info_utils::VehicleInfo vehicle_info_;

  /**
   * @brief Check if all validation criteria are met
   * @param status Validation status
   * @return Boolean indicating if all criteria are met
   */
  static bool is_all_valid(const ControlValidatorStatus & status);

  Trajectory::ConstSharedPtr current_reference_trajectory_;
  Trajectory::ConstSharedPtr current_predicted_trajectory_;

  Odometry::ConstSharedPtr current_kinematics_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_msg_;
  Control::ConstSharedPtr control_cmd_msg_;

  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  std::shared_ptr<ControlValidatorDebugMarkerPublisher> debug_pose_publisher_;
};
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__CONTROL_VALIDATOR_HPP_
