// Copyright 2021 The Autoware Foundation
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_

#include "autoware/mpc_lateral_controller/log_writer.hpp"
#include "autoware/mpc_lateral_controller/lowpass_filter.hpp"
#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/trajectory_follower_base/control_horizon.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_internal_debug_msgs/msg/float32_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
using autoware_control_msgs::msg::Lateral;
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using trajectory_follower::LateralHorizon;

/// Everything MpcLateralController reads to set itself up. The node layer fills it from
/// the parameters it declares, so that the controller itself needs no node.
struct MpcLateralControllerConfig
{
  double ctrl_period{0.0};

  TrajectoryFilteringParam trajectory_filtering{};

  // Stop state
  double stop_state_entry_ego_speed{0.0};
  double stop_state_entry_target_speed{0.0};
  double converged_steer_rad{0.0};
  bool keep_steer_control_until_converged{true};
  double new_traj_duration_time{0.0};
  double new_traj_end_dist{0.0};
  double mpc_converged_threshold_rps{0.0};

  // Nearest index search
  double ego_nearest_dist_threshold{0.0};
  double ego_nearest_yaw_threshold{0.0};

  // Steering offset compensation
  bool enable_auto_steering_offset_removal{false};
  double steer_offset_max_update_th{0.0};
  double steer_offset_filter_cutoff_hz{0.0};

  // Low pass filters inside MPC
  double steering_lpf_cutoff_hz{0.0};
  double error_deriv_lpf_cutoff_hz{0.0};

  // Vehicle
  double wheelbase{0.0};
  double steer_lim{0.0};
  std::string vehicle_model_type{};
  double mass_fl{0.0};
  double mass_fr{0.0};
  double mass_rl{0.0};
  double mass_rr{0.0};
  double cf{0.0};
  double cr{0.0};

  // Steering rate limit, given as pairs of (curvature or velocity, limit in rad/s)
  std::vector<std::pair<double, double>> steer_rate_lim_by_curvature{};
  std::vector<std::pair<double, double>> steer_rate_lim_by_velocity{};

  // MPC
  std::string qp_solver_type{};
  MPCParam mpc_param{};
  double input_delay{0.0};
  bool use_steer_prediction{false};
  bool use_delayed_initial_state{false};
  bool use_temporal_trajectory{false};
  bool publish_debug_trajectories{false};
};

/// What one cycle of MpcLateralController produced, beside the command itself. The node
/// layer publishes these messages.
struct MpcLateralControllerResult
{
  trajectory_follower::LateralOutput output;
  MpcResult mpc;
  double steering_offset{0.0};
};

class MpcLateralController
{
public:
  /// \param writer Where the controller reports what it meets. It has to outlive this
  /// object.
  MpcLateralController(const MpcLateralControllerConfig & config, const LogWriter & writer);
  ~MpcLateralController();

  void set_steering_offset(double offset) { m_steering_offset_ = offset; }

  /// The parameters of the optimisation, which the node layer lets a caller change while
  /// the controller runs.
  const MPCParam & mpcParam() const { return m_mpc->m_param; }
  void setMpcParam(const MPCParam & param);

  /// The result of the last cycle, which the node layer reports as a diagnostic.
  const MpcResult & lastMpcResult() const { return m_mpc_solved_status; }

private:
  const LogWriter & writer_;

  std::shared_ptr<Butterworth2dFilter> lpf_steer_offset_;
  double m_steering_offset_{0.0};
  double m_steering_offset_target_{0.0};
  double m_steering_offset_filtered_{0.0};

  //!< @brief parameters for path smoothing
  TrajectoryFilteringParam m_trajectory_filtering_param;

  // Ego vehicle speed threshold to enter the stop state.
  double m_stop_state_entry_ego_speed;

  // Target vehicle speed threshold to enter the stop state.
  double m_stop_state_entry_target_speed;

  // Convergence threshold for steering control.
  double m_converged_steer_rad;

  // max mpc output change threshold for 1 sec
  double m_mpc_converged_threshold_rps;

  // Time duration threshold to check if the trajectory shape has changed.
  double m_new_traj_duration_time;

  // Distance threshold to check if the trajectory shape has changed.
  double m_new_traj_end_dist;

  // Flag indicating whether to keep the steering control until it converges.
  bool m_keep_steer_control_until_converged;

  // MPC solver checker.
  MpcResult m_mpc_solved_status{true};

  // trajectory buffer for detecting new trajectory
  std::deque<Trajectory> m_trajectory_buffer;

  std::unique_ptr<MPC> m_mpc;  // MPC object for trajectory following.

  // Check is mpc output converged
  bool m_is_mpc_history_filled{false};

  // store the last mpc outputs for 1 sec
  std::vector<std::pair<Lateral, rclcpp::Time>> m_mpc_steering_history{};

  // set the mpc steering output to history
  void setSteeringToHistory(const Lateral & steering, const rclcpp::Time & stamp);

  // check if the mpc steering output is converged
  bool isMpcConverged();

  // measured kinematic state
  Odometry m_current_kinematic_state;

  SteeringReport m_current_steering;  // Measured steering information.

  Trajectory m_current_trajectory;  // Current reference trajectory for path following.

  double m_steer_cmd_prev = 0.0;  // MPC output in the previous period.

  // Flag indicating whether the previous control command is initialized.
  bool m_is_ctrl_cmd_prev_initialized = false;

  // Previous control command for path following.
  Lateral m_ctrl_cmd_prev;

  //  Flag indicating whether the first trajectory has been received.
  bool m_has_received_first_trajectory = false;

  // Threshold distance for the ego vehicle in nearest index search.
  double m_ego_nearest_dist_threshold;

  // Threshold yaw for the ego vehicle in nearest index search.
  double m_ego_nearest_yaw_threshold;

  // Flag indicating whether auto steering offset removal is enabled.
  bool enable_auto_steering_offset_removal_;

  // Threshold for maximum change in steering offset to prevent large updates.
  double m_steer_offset_max_update_th_;

  /**
   * @brief Create the vehicle model named by the configuration.
   * @return Pointer to the created vehicle model, or nullptr when the name is unknown.
   */
  static std::shared_ptr<VehicleModelInterface> createVehicleModel(
    const MpcLateralControllerConfig & config, const LogWriter & writer);

  /**
   * @brief Create the quadratic problem solver named by the configuration.
   * @return Pointer to the created solver, or nullptr when the name is unknown.
   */
  static std::shared_ptr<QPSolverInterface> createQPSolverInterface(
    const MpcLateralControllerConfig & config);

public:
  /**
   * @brief Check if all necessary data is received and ready to run the control.
   * @param input_data Input data required for control calculation.
   * @return True if the data is ready, false otherwise.
   */
  bool isReady(const trajectory_follower::InputData & input_data);

  /**
   * @brief Compute the control command for path following with a constant control period.
   * @param input_data Input data required for control calculation.
   * @return Lateral output control command.
   */
  MpcLateralControllerResult run(
    trajectory_follower::InputData const & input_data, const rclcpp::Time & stamp);

private:
  /**
   * @brief Set the current trajectory using the received message.
   * @param msg Received trajectory message.
   */
  void setTrajectory(const Trajectory & msg, const Odometry & current_kinematics);

  /**
   * @brief Check if the received data is valid.
   * @return True if the data is valid, false otherwise.
   */
  [[nodiscard]] bool checkData() const;

  /**
   * @brief Create the control command.
   * @param ctrl_cmd Control command to be created.
   * @param stamp Timestamp of this control cycle.
   * @return Created control command.
   */
  Lateral createCtrlCmdMsg(const Lateral & ctrl_cmd, const builtin_interfaces::msg::Time & stamp);

  /**
   * @brief Create the control command horizon message.
   * @param ctrl_cmd_horizon Control command horizon to be created.
   * @param stamp Timestamp of this control cycle.
   * @return Created control command horizon.
   */
  [[nodiscard]] LateralHorizon createCtrlCmdHorizonMsg(
    const LateralHorizon & ctrl_cmd_horizon, const builtin_interfaces::msg::Time & stamp) const;

  /**
   * @brief Get the stop control command.
   * @return Stop control command.
   */
  [[nodiscard]] Lateral getStopControlCommand() const;

  /**
   * @brief Get the control command applied before initialization.
   * @return Initial control command.
   */
  [[nodiscard]] Lateral getInitialControlCommand() const;

  /**
   * @brief Check if the ego car is in a stopped state.
   * @return True if the ego car is stopped, false otherwise.
   */
  [[nodiscard]] bool isStoppedState() const;

  /**
   * @brief Check if the trajectory has a valid value.
   * @param traj Trajectory to be checked.
   * @return True if the trajectory is valid, false otherwise.
   */
  [[nodiscard]] bool isValidTrajectory(const Trajectory & traj) const;

  /**
   * @brief Check if the trajectory shape has changed.
   * @return True if the trajectory shape has changed, false otherwise.
   */
  [[nodiscard]] bool isTrajectoryShapeChanged() const;

  /**
   * @brief Check if the steering control is converged and stable now.
   * @param cmd Steering control command to be checked.
   * @return True if the steering control is converged and stable, false otherwise.
   */
  [[nodiscard]] bool isSteerConverged(const Lateral & cmd) const;
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_
