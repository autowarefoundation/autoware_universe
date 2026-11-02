// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_

#include "autoware/trajectory_follower_base/control_horizon.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_interface.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_interface.hpp"
#include "autoware/trajectory_follower_base/sync_data.hpp"

#include <rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/control_horizon.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <optional>
#include <string>

namespace autoware::motion::control::trajectory_follower_node
{
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

struct TrajectoryFollowerConfig
{
  /// How old a command may be before the cycle publishes none [s].
  double timeout_thr_sec{0.0};
  bool enable_control_cmd_horizon{false};
};

struct TrajectoryFollowerResult
{
  autoware_control_msgs::msg::Control control_cmd;
  std::optional<autoware_planning_msgs::msg::Trajectory> predicted_trajectory{std::nullopt};

  /// Holds nothing when the config does not ask for the horizon, or when the two horizons cannot
  /// be merged.
  std::optional<autoware_control_msgs::msg::ControlHorizon> control_cmd_horizon{std::nullopt};

  /// How far the steering of the vehicle is from the command, drawn for rviz.
  visualization_msgs::msg::MarkerArray debug_marker;
};

/// Why a cycle produced no command, and how loudly the node reports it.
struct TrajectoryFollowerError
{
  enum class Level { info, error };

  Level level;
  std::string message;
};

/// Runs the lateral and the longitudinal controller for one cycle and merges their outputs into
/// one command. It reads no clock, parameter or topic, so the same input data and the same stamp
/// always give the same result.
class TrajectoryFollower
{
public:
  TrajectoryFollower(
    const TrajectoryFollowerConfig & config,
    std::unique_ptr<trajectory_follower::LateralControllerInterface> lateral_controller,
    std::unique_ptr<trajectory_follower::LongitudinalControllerInterface> longitudinal_controller);

  /// \param stamp The time of this cycle, read once by the caller.
  tl::expected<TrajectoryFollowerResult, TrajectoryFollowerError> run(
    const trajectory_follower::InputData & input_data, const rclcpp::Time & stamp);

private:
  TrajectoryFollowerConfig config_;
  std::unique_ptr<trajectory_follower::LateralControllerInterface> lateral_controller_;
  std::unique_ptr<trajectory_follower::LongitudinalControllerInterface> longitudinal_controller_;

  // output of the lateral controller in the previous cycle, passed to the longitudinal controller
  trajectory_follower::LateralSyncData previous_lateral_sync_data_;
};

/**
 * @brief merge lateral and longitudinal horizons
 * @details If one of the commands has only one control, repeat the control to match the other
 *          horizon. If each horizon has different time intervals, resample them to match the size
 *          with the greatest common divisor.
 * @param lateral_horizon lateral horizon
 * @param longitudinal_horizon longitudinal horizon
 * @param stamp stamp
 * @return merged control horizon
 */
std::optional<autoware_control_msgs::msg::ControlHorizon> mergeLatLonHorizon(
  const trajectory_follower::LateralHorizon & lateral_horizon,
  const trajectory_follower::LongitudinalHorizon & longitudinal_horizon,
  const rclcpp::Time & stamp);

/// Draw how far the steering of the vehicle is from the command, for rviz.
visualization_msgs::msg::MarkerArray createDebugMarker(
  const trajectory_follower::InputData & input_data,
  const trajectory_follower::LateralOutput & lat_out, const rclcpp::Time & stamp);

}  // namespace autoware::motion::control::trajectory_follower_node

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__TRAJECTORY_FOLLOWER_HPP_
