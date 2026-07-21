// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_

#include "autoware/pid_longitudinal_controller/debug_values.hpp"
#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"
#include "autoware/pid_longitudinal_controller/lowpass_filter.hpp"
#include "autoware/pid_longitudinal_controller/pid.hpp"
#include "autoware/pid_longitudinal_controller/smooth_stop.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware_utils/ros/marker_helper.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include "autoware_control_msgs/msg/longitudinal.hpp"
#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using visualization_msgs::msg::MarkerArray;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };

/// \brief parameters that configure the longitudinal control algorithm, read once at
/// construction and updatable at runtime through the parameter callback
struct PidLongitudinalControllerConfig
{
  enum class SlopeSource {
    RAW_PITCH = 0,
    TRAJECTORY_PITCH,
    TRAJECTORY_ADAPTIVE,
    TRAJECTORY_GOAL_ADAPTIVE
  };

  struct StateTransitionParams
  {
    // drive
    double drive_state_stop_dist;
    double drive_state_offset_stop_dist;
    // stopping
    double stopping_state_stop_dist;
    // stop
    double stopped_state_entry_duration_time;
    double stopped_state_entry_vel;
    double stopped_state_entry_acc;
    // emergency
    double emergency_state_overshoot_stop_dist;
  };

  struct StoppedStateParams
  {
    double vel;
    double acc;
  };

  struct EmergencyStateParams
  {
    double vel;
    double acc;
    double jerk;
  };

  // vehicle info
  double wheel_base{0.0};
  double front_overhang{0.0};

  // control period
  double longitudinal_ctrl_period;

  // delay compensation
  double delay_compensation_time;
  bool use_temporal_trajectory{false};

  // enable flags
  bool enable_smooth_stop;
  bool enable_overshoot_emergency;
  bool enable_slope_compensation;
  bool enable_large_tracking_error_emergency;
  bool enable_keep_stopped_until_steer_convergence;

  // smooth stop transition
  StateTransitionParams state_transition_params;

  // drive
  bool enable_integration_at_low_speed;
  double current_vel_threshold_pid_integrate;
  double time_threshold_before_pid_integrate;
  double ff_scale_min{0.5};
  double ff_scale_max{2.0};
  bool enable_brake_keeping_before_stop;
  double brake_keeping_acc;

  // stop
  StoppedStateParams stopped_state_params;

  // emergency
  EmergencyStateParams emergency_state_params;

  // acc feedback
  double acc_feedback_gain;

  // acceleration limit
  double max_acc;
  double min_acc;

  // jerk limit
  double max_jerk;
  double min_jerk;
  double max_acc_cmd_diff;

  // slope compensation
  SlopeSource slope_source{SlopeSource::RAW_PITCH};
  double adaptive_trajectory_velocity_th;
  double max_pitch_rad;
  double min_pitch_rad;

  // ego nearest index search
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;
};

/// \class PidLongitudinalController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class PidLongitudinalController : public trajectory_follower::LongitudinalControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit PidLongitudinalController(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);

private:
  struct Motion
  {
    double vel{0.0};
    double acc{0.0};
  };
  struct StateAfterDelay
  {
    StateAfterDelay(const double velocity, const double acceleration, const double distance)
    : vel(velocity), acc(acceleration), running_distance(distance)
    {
    }
    double vel{0.0};
    double acc{0.0};
    double running_distance{0.0};
  };
  enum class Shift { Forward = 0, Reverse };

  struct ControlData
  {
    autoware_planning_msgs::msg::Trajectory interpolated_traj{};
    size_t nearest_idx{0};  // nearest_idx = 0 when nearest_idx is not found with findNearestIdx
    size_t target_idx{0};
    StateAfterDelay state_after_delay{0.0, 0.0, 0.0};
    Motion current_motion{};
    geometry_msgs::msg::Pose current_pose{};
    OperationModeState operation_mode{};
    Shift shift{Shift::Forward};  // shift is used only to calculate the sign of pitch compensation
    double stop_dist{0.0};  // signed distance that is positive when car is before the stopline
    double slope_angle{0.0};
    double dt{0.0};
    double temporal_predicted_time{std::numeric_limits<double>::quiet_NaN()};
    double temporal_fused_time{std::numeric_limits<double>::quiet_NaN()};
    rclcpp::Time current_time{};  // time captured once per control cycle in run()
  };
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  // ros variables
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_slope;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr
    m_pub_debug;
  rclcpp::Publisher<MarkerArray>::SharedPtr m_pub_virtual_wall_marker;
  std::optional<MarkerArray> m_virtual_wall_marker{std::nullopt};

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // pointers for ros topic
  autoware_planning_msgs::msg::Trajectory m_last_valid_trajectory;

  // configuration parameters
  PidLongitudinalControllerConfig config;

  bool m_prev_vehicle_is_under_control{false};
  std::shared_ptr<rclcpp::Time> m_under_control_starting_time{nullptr};

  // control state
  ControlState m_control_state{ControlState::STOPPED};

  std::optional<double> m_prev_nearest_time{std::nullopt};

  // drive
  PIDController m_pid_vel;
  std::shared_ptr<LowpassFilter1d> m_lpf_vel_error{nullptr};

  // smooth stop
  std::optional<SmoothStop> m_smooth_stop;

  std::shared_ptr<LowpassFilter1d> m_lpf_acc_error{nullptr};

  // slope compensation
  std::shared_ptr<LowpassFilter1d> m_lpf_pitch{nullptr};
  std::optional<double> m_previous_slope_angle{std::nullopt};

  // buffer of send command
  std::vector<autoware_control_msgs::msg::Longitudinal> m_ctrl_cmd_vec;

  // for calculating dt
  std::shared_ptr<rclcpp::Time> m_prev_control_time{nullptr};

  // shift mode
  Shift m_prev_shift{Shift::Forward};

  // diff limit
  Motion m_prev_ctrl_cmd{};      // with slope compensation
  Motion m_prev_raw_ctrl_cmd{};  // without slope compensation

  // debug values
  DebugValues m_debug_values;

  std::optional<bool> m_prev_keep_stopped_condition{std::nullopt};

  std::shared_ptr<rclcpp::Time> m_last_running_time{std::make_shared<rclcpp::Time>(clock_->now())};

  // Diagnostic
  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_{};  // Diagnostic updater for publishing diagnostic data.
  void setupDiagnosticUpdater();
  void checkControlState(diagnostic_updater::DiagnosticStatusWrapper & stat);

  struct ResultWithReason
  {
    bool result{false};
    std::string reason{""};
  };

  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief compute control command, and publish periodically
   */
  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief calculate data for controllers whose type is ControlData
   * @param [in] input_data input data containing current odometry, acceleration, and operation
   * mode
   * @param [in] current_time time captured once per control cycle in run()
   */
  ControlData getControlData(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & current_time);

  /**
   * @brief calculate control command in emergency state
   * @param [in] control_data data for control calculation
   */
  Motion calcEmergencyCtrlCmd(const ControlData & control_data);

  /**
   * @brief change control state
   * @param [in] new state
   * @param [in] reason to change control state
   */
  void changeControlState(const ControlState & control_state, const std::string & reason = "");

  /**
   * @brief update control state according to the current situation
   * @param [in] control_data control data
   */
  void updateControlState(const ControlData & control_data);

  /**
   * @brief calculate control command based on the current control state
   * @param [in] control_data control data
   */
  Motion calcCtrlCmd(const ControlData & control_data);

  /**
   * @brief publish control command
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] current_time time captured once per control cycle in run()
   */
  autoware_control_msgs::msg::Longitudinal createCtrlCmdMsg(
    const Motion & ctrl_cmd, const rclcpp::Time & current_time);

  /**
   * @brief publish debug data
   * @param [in] ctrl_cmd calculated control command to control velocity
   * @param [in] control_data data for control calculation
   */
  void publishDebugData(const Motion & ctrl_cmd, const ControlData & control_data);

  /**
   * @brief publish the virtual wall marker created during this cycle, if any
   */
  void publishVirtualWallMarker();

  /**
   * @brief calculate time between current and previous one
   * @param [in] current_time time captured once per control cycle in run()
   */
  double getDt(const rclcpp::Time & current_time);

  /**
   * @brief calculate current velocity and acceleration
   */
  Motion getCurrentMotion() const;

  /**
   * @brief calculate direction (forward or backward) that vehicle moves
   * @param [in] control_data data for control calculation
   */
  enum Shift getCurrentShift(const ControlData & control_data) const;

  /**
   * @brief filter acceleration command with limitation of acceleration and jerk, and slope
   * compensation
   * @param [in] raw_acc acceleration before filtered
   * @param [in] control_data data for control calculation
   */
  double calcFilteredAcc(const double raw_acc, const ControlData & control_data);

  /**
   * @brief store acceleration command before slope compensation
   * @param [in] accel command before slope compensation
   * @param [in] current_time time captured once per control cycle in run()
   */
  void storeAccelCmd(const double accel, const rclcpp::Time & current_time);

  /**
   * @brief add acceleration to compensate for slope
   * @param [in] acc acceleration before slope compensation
   * @param [in] pitch pitch angle (upward is negative)
   * @param [in] shift direction that vehicle move (forward or backward)
   */
  double applySlopeCompensation(const double acc, const double pitch, const Shift shift) const;

  /**
   * @brief keep target motion acceleration negative before stop
   * @param [in] traj reference trajectory
   * @param [in] motion delay compensated target motion
   */
  Motion keepBrakeBeforeStop(
    const ControlData & control_data, const Motion & target_motion, const size_t nearest_idx) const;

  /**
   * @brief interpolate trajectory point that is nearest to vehicle
   * @param [in] traj reference trajectory
   * @param [in] point vehicle position
   * @param [in] nearest_idx index of the trajectory point nearest to the vehicle position
   */
  std::pair<autoware_planning_msgs::msg::TrajectoryPoint, size_t>
  calcInterpolatedTrajPointAndSegment(
    const autoware_planning_msgs::msg::Trajectory & traj,
    const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief calculate predicted velocity after time delay based on past control commands
   * @param [in] control_data data for control calculation
   * @param [in] delay_compensation_time predicted time delay
   */
  StateAfterDelay predictedStateAfterDelay(
    const ControlData & control_data, const double delay_compensation_time) const;

  /**
   * @brief calculate velocity feedback with feed forward and pid controller
   * @param [in] control_data data for control calculation
   */
  double applyVelocityFeedback(const ControlData & control_data);

  /**
   * @brief update variables for debugging about pitch
   * @param [in] pitch_using
   * @param [in] traj_pitch
   * @param [in] localization_pitch
   * @param [in] localization_pitch_lpf
   */
  void updatePitchDebugValues(
    const double pitch_using, const double traj_pitch, const double localization_pitch,
    const double localization_pitch_lpf);

  /**
   * @brief update variables for velocity and acceleration
   * @param [in] ctrl_cmd latest calculated control command
   * @param [in] control_data data for control calculation
   */
  void updateDebugVelAcc(const ControlData & control_data);

  /**
   * @brief calculate elapsed time since the vehicle entered autoware control
   * @param [in] current_time time captured once per control cycle in run()
   */
  double getTimeUnderControl(const rclcpp::Time & current_time);
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__PID_LONGITUDINAL_CONTROLLER_HPP_
