// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_
#define AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_

#include "autoware/pure_pursuit/autoware_pure_pursuit.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_viz.hpp"
#include "autoware/pure_pursuit/log_writer.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/time.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <optional>
#include <vector>

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace autoware::pure_pursuit
{

struct PpOutput
{
  double curvature;
  double velocity;
};

struct PurePursuitLateralControllerConfig
{
  // Global Parameters
  double wheel_base;
  double max_steering_angle;  // [rad]

  // Algorithm Parameters
  double ld_velocity_ratio;
  double ld_lateral_error_ratio;
  double ld_curvature_ratio;
  double min_lookahead_distance;
  double max_lookahead_distance;
  double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
  double converged_steer_rad_;
  double prediction_ds;
  double prediction_distance_length;  // Total distance of prediction trajectory
  double resampling_ds;
  double curvature_calculation_distance;
  double long_ld_lateral_error_threshold;
  bool enable_path_smoothing;
  int path_filter_moving_ave_num;
};

struct DebugData
{
  geometry_msgs::msg::Point next_target;
};

/// What one cycle of PurePursuitLateralController returns. The node publishes the messages.
struct PurePursuitLateralControllerResult
{
  LateralOutput output;
  std::optional<autoware_internal_debug_msgs::msg::Float32MultiArrayStamped> lookahead_debug_values;
  std::optional<Trajectory> predicted_trajectory;
};

class PurePursuitLateralController
{
public:
  /// \param writer Where the controller writes its log. It has to outlive this object.
  PurePursuitLateralController(
    const PurePursuitLateralControllerConfig & config, const LogWriter & writer);

  /// Compute the command of one control cycle.
  /// \param stamp Time of the cycle. The command and the lookahead debug values have this stamp.
  PurePursuitLateralControllerResult run(const InputData & input_data, const rclcpp::Time & stamp);

private:
  const LogWriter & writer_;
  std::vector<TrajectoryPoint> output_tp_array_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr trajectory_resampled_;
  autoware_planning_msgs::msg::Trajectory trajectory_;
  nav_msgs::msg::Odometry current_odometry_;
  autoware_vehicle_msgs::msg::SteeringReport current_steering_;
  std::optional<Lateral> prev_cmd_;

  void setResampledTrajectory();

  geometry_msgs::msg::Pose current_pose_;

  void publishDebugMarker() const;

  Lateral generateCtrlCmdMsg(const double target_curvature, const rclcpp::Time & stamp);

  // Parameter
  PurePursuitLateralControllerConfig param_{};

  // Algorithm
  std::unique_ptr<PurePursuit> pure_pursuit_;

  std::optional<PpOutput> calcTargetCurvature(
    bool is_control_output, geometry_msgs::msg::Pose pose, const rclcpp::Time & stamp,
    PurePursuitLateralControllerResult & result);

  /**
   * @brief It takes current pose, control command, and delta distance. Then it calculates next pose
   * of vehicle.
   */

  TrajectoryPoint calcNextPose(const double ds, TrajectoryPoint & point, Lateral cmd) const;

  std::optional<Trajectory> generatePredictedTrajectory(
    const rclcpp::Time & stamp, PurePursuitLateralControllerResult & result);

  Lateral generateOutputControlCmd(
    const rclcpp::Time & stamp, PurePursuitLateralControllerResult & result);

  bool calcIsSteerConverged(const Lateral & cmd);

  double calcLookaheadDistance(
    const double lateral_error, const double curvature, const double velocity, const double min_ld,
    const bool is_control_cmd, const rclcpp::Time & stamp,
    PurePursuitLateralControllerResult & result);

  double calcCurvature(const size_t closest_idx);

  void averageFilterTrajectory(autoware_planning_msgs::msg::Trajectory & u);

  // Debug
  mutable DebugData debug_data_;
};

}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__AUTOWARE_PURE_PURSUIT_LATERAL_CONTROLLER_HPP_
