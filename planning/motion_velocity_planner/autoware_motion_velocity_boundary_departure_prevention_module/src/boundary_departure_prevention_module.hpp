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

#ifndef BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
#define BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_

#include "parameters.hpp"
#include "slow_down_interpolator.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <tl_expected/expected.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
class BoundaryDeparturePreventionModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const TrajectoryPoints & raw_trajectory_points,
    const TrajectoryPoints & smoothed_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; };
  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  // === Interface and inputs validation ====
  void subscribe_topics(rclcpp::Node & node);
  void publish_topics(rclcpp::Node & node);
  void take_data();
  std::optional<std::string> is_data_invalid(const TrajectoryPoints & raw_trajectory_points) const;
  std::optional<std::string> is_data_timeout(const Odometry & odom) const;
  std::optional<std::string> is_route_changed();
  bool is_autonomous_mode() const;

  // === Internal logic

  tl::expected<VelocityPlanningResult, std::string> plan_slow_down_intervals(
    const TrajectoryPoints & raw_trajectory_points,
    const std::shared_ptr<const PlannerData> & planner_data);

  /**
   * @brief Update the list of critical departure points.
   *
   * Projects existing critical departure points onto the updated reference trajectory
   * and removes points that are outdated (i.e., passed by the ego or shifted significantly).
   *
   * @param aw_ref_traj Current reference trajectory.
   * @param offset_from_ego Minimum distance from ego to keep a point; points closer than this are
   * removed.
   */
  void update_critical_departure_points(
    const trajectory::Trajectory<TrajectoryPoint> & aw_ref_traj, const double offset_from_ego);

  std::unordered_map<DepartureType, bool> get_diagnostics(
    const double curr_vel, const double dist_with_offset_m);

  /**
   * @brief Check if critical departure has been continuously observed.
   *
   * Determines whether critical departure points have been continuously present
   * for longer than the configured buffer time (`on_time_buffer_s.critical_departure`).
   *
   * This prevents re-adding critical departure points too frequently due to transient noise.
   * The last time *found* critical departure is stored to measure the continuous duration.
   *
   * @return True if critical departure has been continuously observed long enough, false otherwise.
   */
  bool is_continuous_critical_departure();

  rclcpp::Clock::SharedPtr clock_ptr_;

  std::string module_name_;
  Output output_;
  NodeParam node_param_;
  rclcpp::TimerBase::SharedPtr timer_ptr_;
  std::unique_ptr<utils::SlowDownInterpolator> slow_down_interpolator_ptr_;
  MarkerArray debug_marker_;
  MarkerArray slow_down_wall_marker_;
  std::unique_ptr<LaneletRoute> prev_route_ptr_;
  static constexpr auto throttle_duration_ms{5000};

  Trajectory::ConstSharedPtr ego_pred_traj_ptr_;
  Control::ConstSharedPtr control_cmd_ptr_;
  SteeringReport::ConstSharedPtr steering_angle_ptr_;
  OperationModeState::ConstSharedPtr op_mode_state_ptr_;
  LaneletRoute::ConstSharedPtr route_ptr_;
  std::unordered_map<std::string, double> processing_times_ms_;

  double last_abnormality_fp_overlap_bound_time_{0.0};
  double last_abnormality_fp_no_overlap_bound_time_{0.0};
  double last_no_critical_dpt_time_{0.0};

  autoware_utils::InterProcessPollingSubscriber<Trajectory>::SharedPtr ego_pred_traj_polling_sub_;
  autoware_utils::InterProcessPollingSubscriber<Control>::SharedPtr control_cmd_polling_sub_;
  autoware_utils::InterProcessPollingSubscriber<SteeringReport>::SharedPtr
    steering_angle_polling_sub_;
  autoware_utils::InterProcessPollingSubscriber<OperationModeState>::SharedPtr
    op_mode_state_polling_sub_;
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>::SharedPtr route_polling_sub_;

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr processing_time_detail_pub_;

  std::unique_ptr<BoundaryDepartureChecker> boundary_departure_checker_ptr_;
  std::unique_ptr<diagnostic_updater::Updater> updater_ptr_;

  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::motion_velocity_planner::experimental
#endif  // BOUNDARY_DEPARTURE_PREVENTION_MODULE_HPP_
